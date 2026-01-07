/*
 * Encoder for the Epworth organ.
 *
 * Currently this only scans the stop tabs (i.e. keys) and displays the stop state using the
 * panel-mounted LEDs. Soon enough we'll add logic for the entire combination action (pistons,
 * toe spades, etc.).
 */

#include <MIDI.h>
#include <Wire.h>
#include <MCP23017.h>

constexpr uint8_t PISTON_LOAD_PIN = 10;
constexpr uint8_t PISTON_DATA_PIN = 9;
constexpr uint8_t PISTON_CLOCK_PIN = 8;

// To save on CPU cycles, don't poll the stop tab keys too often. We could keep adjusting this
// number upward as long as we don't drop any key presses.
constexpr uint16_t STOP_TAB_BUTTON_POLL_ITVL_MICROS = 1500;
constexpr uint16_t PISTON_POLL_ITVL_MICROS = 2500;


MCP23017 stopTabMcps[4][2] = {
  {MCP23017(0x20), MCP23017(0x21)},
  {MCP23017(0x22), MCP23017(0x23)},
  {MCP23017(0x24), MCP23017(0x25)},
  {MCP23017(0x26), MCP23017(0x27)},
};

uint16_t stopState[4] = {0, 0, 0, 0};
bool awaitingSavePreset = false;

// TODO: Put these in EEPROM.
uint16_t presets[8][4] = {0};

// A reusable debouncer class. This is necessary because we need to debounce readings that don't
// correspond to pins on the MCU GPIO; they correspond to pins on external shift registers/port
// expanders.
// Type parameters:
// T: The integer type (uint8_t or uint16_t)
// DEPTH: How many history steps to keep
template<typename T, uint8_t DEPTH>
class Debouncer {
private:
  T history[DEPTH];
  uint8_t index = 0;
  T stableState;
  T prevStableState;

  // Initialize history to all 1s (assuming active low buttons)
  void resetHistory() {
    // Fill with max value for type T (0xFF or 0xFFFF)
    T allOnes = (T)~0;
    for (int i = 0; i < DEPTH; i++) {
      history[i] = allOnes;
    }
    stableState = allOnes;
    prevStableState = stableState;
  }

public:
  Debouncer() {
    resetHistory();
  }

  // Returns true if state changed, so we know when to check falling or rising edges.
  void update(T rawReading) {
    history[index] = rawReading;
    index = (index + 1) % DEPTH;

    // Run through history, compute stable state. At the end of this loop, a 1 in position i of
    // stableHigh means that the reading at position i has been 1 at every step in recorded
    // history, and a 0 in position i of stableLow means that the reading at position i has been
    // 0 at every step. The other 2 possibilities are noisy, and hence ignored.
    T stableHigh = (T)~0;
    T stableLow = 0;

    for (uint8_t i = 0; i < DEPTH; i++) {
      stableHigh &= history[i];
      stableLow |= history[i];
    }

    // Save old state in order to detect whether state has changed.
    prevStableState = stableState;

    // Update debounced state.
    stableState |= stableHigh;
    stableState &= stableLow;
  }

  // Applies user lambda to bits that went from 1 -> 0 this time around.
  template<typename AcceptsUint8>
  void forEachFallingEdge(AcceptsUint8 userFn) {
    // There are no falling edges if previous and current states are equal.
    if (prevStableState != stableState) {
      applyToMaskBits(prevStableState & ~stableState, userFn);
    }
  }

  // Applies user lambda to bits that went from 0 -> 1 this time around.
  template<typename AcceptsUint8>
  void forEachRisingEdge(AcceptsUint8 userFn) {
    // There are no rising edges if previous and current states are equal.
    if (prevStableState != stableState) {
      applyToMaskBits(~prevStableState & stableState, userFn);
    }
  }

private:
  template<typename AcceptsUint8>
  void applyToMaskBits(T mask, AcceptsUint8 userFn) {
    while (mask != 0) {
      // Fancy compiler builtins...
      int i = __builtin_ctz(mask);
      userFn(i);

      // Clear the bit we just processed so we can find the next one
      mask &= ~(static_cast<T>(1) << i);
    }
  }
};

Debouncer<uint16_t, 5> stopTabButtonDebouncers[4];  // 4 divisions of 15 buttons
Debouncer<uint8_t, 5> pistonDebouncer;              // 1 set of 8 buttons

MIDI_CREATE_INSTANCE(HardwareSerial, Serial1, MIDI);

uint16_t reverseByte(uint16_t b) {
  b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
  b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
  b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
  return b;
}

// Returns the raw, noisy 16-bit state of a division.
uint16_t readRawStopTabButtons(uint8_t div) {
  uint16_t reading = 0;

  // In the following lines, for both ports A and B, the MSB corresponds to pin 7 and LSB to pin 0.

  // For the first chip of every pair, need to read both ports, as button 8 is routed
  // to GPA0, though buttons 1-7 are on GPB0-6 (in reverse order).
  reading |= stopTabMcps[div][0].readPort(MCP23017Port::B) & 0x7F;
  reading <<= 1;
  reading |= stopTabMcps[div][0].readPort(MCP23017Port::A) & 0x01;
  reading <<= 7;
  // For the second chip of every pair, only need to read port B, ignoring GPB7 (MSB), since
  // there are only 7 buttons hooked up to this one (GPB0-6).
  reading |= stopTabMcps[div][1].readPort(MCP23017Port::B) & 0x7F;
  reading <<= 1;
  // By convention, LSB needs to be 1 to represent the imaginary 16th button, never pressed.
  reading |= 0x0001;

  return reading;
}

uint8_t readRawPistons() {
  // Load button state into the 74HC165 with a pulse.
  digitalWrite(PISTON_LOAD_PIN, LOW);
  digitalWrite(PISTON_LOAD_PIN, HIGH);

  uint8_t reading = 0;
  // Ensure clock pin low first...
  digitalWrite(PISTON_CLOCK_PIN, LOW);

  // ...then clock the data into the piston readings. We do this manually because the arduino
  // shiftIn implementation pulses the clock *before* reading, which is the wrong order for the
  // 74HC165N.
  for (uint8_t i = 0; i < 8; i++) {
    if (digitalRead(PISTON_DATA_PIN)) {
      reading |= (1 << i);
    }
    digitalWrite(PISTON_CLOCK_PIN, HIGH);
    digitalWrite(PISTON_CLOCK_PIN, LOW);
  }

  return reading;
}

void writeStopStateToLeds(uint8_t division) {
  // The schematic makes this clear, but it may be counterintuitive: Because of the pin layout on
  // the MCP23017, the LEDs are effectively in reverse order from the switches. That is, although
  // GPA1 -> D1 ... GPA7 -> D7, we actually have GPB0 -> Sw7In ... GPB6 -> Sw1In. Ergo, because the
  // stop state bits correspond L to R to the LEDs, we need to reverse them before writing out.
  // Finally, stop on == LED on == logic level LOW because we are sinking current through the LEDs,
  // so need to invert all bits.
  uint8_t leftHalf = reverseByte(~stopState[division] >> 8);
  uint8_t rightHalf = reverseByte(~stopState[division] & 0xFF);

  stopTabMcps[division][0].writePort(MCP23017Port::A, (leftHalf & 0x7F) << 1);  // LSB is input
  stopTabMcps[division][0].writePort(MCP23017Port::B, leftHalf & 0x80);         // MSB only; rest are inputs
  stopTabMcps[division][1].writePort(MCP23017Port::A, rightHalf & 0x7F);        // MSB is not connected
}

void onStopTabButtonPressed(uint8_t division, uint8_t buttonIdx) {
  uint16_t stopMask = 1 << (15 - buttonIdx);
  stopState[division] ^= stopMask;

  writeStopStateToLeds(division);

  // This is part of the MIDI contract that needs to be documented: Stop tabs use the undefined
  // MIDI CCs 102-116 (inclusive), with channel equal to 1-indexed division, value 127 for ON, 0
  // for OFF.
  uint8_t controlNumber = 102 + buttonIdx;
  uint8_t controlValue = stopState[division] & stopMask ? 127 : 0;

  // Humans talk about MIDI channels 1-16. I would have expected computers to speak of MIDI
  // channels 0-15, but the library we're using does not do this; it expects 1-16. Hence the +1.
  uint8_t channel = division + 1;

  MIDI.sendControlChange(controlNumber, controlValue, channel);
}

void emitStopState() {
  for (uint8_t i = 0; i < 4; i++) {
    for (uint8_t j = 0; j < 15; j++) {
      uint16_t bitMask = 1 << (15 - j);
      uint8_t controlValue = (stopState[i] & bitMask) ? 127 : 0;
      MIDI.sendControlChange(102 + j, controlValue, i + 1);
    }
  }
}

void onPistonPressed(uint8_t i) {
  if (i == 0) {  // SAVE piston
    awaitingSavePreset = true;
  } else if (i == 1) {  // GC piston
    for (uint8_t div = 0; div < 4; div++) {
      stopState[div] = 0;
      writeStopStateToLeds(div);
    }
    emitStopState();
  } else {  // presets
    if (awaitingSavePreset) {
      for (uint8_t div = 0; div < 4; div++) {
        presets[i][div] = stopState[div];
      }
    } else {
      for (uint8_t div = 0; div < 4; div++) {
        stopState[div] = presets[i][div];
        writeStopStateToLeds(div);
      }
      emitStopState();
    }
  }
}

void onPistonReleased(uint8_t i) {
  // The only piston we really care about when released is the "save" button.
  if (i == 0) {
    awaitingSavePreset = false;
  }
}

void setup() {
  Wire.begin();
  MIDI.begin(MIDI_CHANNEL_OMNI);

  // Pins 7 on both ports can only be used as output, due to a bug in the MCP23017 chip. For the
  // first chip of each pair, we swapped one of the inputs onto port A. For the second chip of
  // each pair, we are not using GPIO pins A7 or B7, because each division (swell, great, choir,
  // pedal) only has 15 input buttons and 15 output LEDs. So for each division, we will have to
  // read 3 GPIO ports: A and B on the first MCP23017, but only B on the second chip.
  for (int i = 0; i < 4; i++) {
    // A few subtle differences between the 1st and 2nd of each group, hence the explicitness.
    stopTabMcps[i][0].init();
    stopTabMcps[i][1].init();
    stopTabMcps[i][0].portMode(MCP23017Port::A, 0x01);
    stopTabMcps[i][1].portMode(MCP23017Port::A, 0x00);
    stopTabMcps[i][0].portMode(MCP23017Port::B, 0x7F);
    stopTabMcps[i][1].portMode(MCP23017Port::B, 0x7F);
    stopTabMcps[i][0].writeRegister(MCP23017Register::GPIO_A, 0xFE);
    stopTabMcps[i][1].writeRegister(MCP23017Register::GPIO_A, 0x7F);
    stopTabMcps[i][0].writeRegister(MCP23017Register::GPIO_B, 0x80);
    stopTabMcps[i][1].writeRegister(MCP23017Register::GPIO_B, 0x00);
  }

  // Set up initial state for 74HC165 shift register that reads pistons
  pinMode(PISTON_LOAD_PIN, OUTPUT);
  pinMode(PISTON_CLOCK_PIN, OUTPUT);
  pinMode(PISTON_DATA_PIN, INPUT_PULLUP);
  digitalWrite(PISTON_LOAD_PIN, HIGH);
  digitalWrite(PISTON_CLOCK_PIN, LOW);
}

uint32_t lastStopTabKeyScan = 0;
uint32_t lastPistonScan = 0;
uint32_t now = 0;

void loop() {
  now = micros();

  // Scan stop tab buttons.
  if (now - lastStopTabKeyScan > STOP_TAB_BUTTON_POLL_ITVL_MICROS) {
    lastStopTabKeyScan = now;

    // Stop tab buttons for each division are logically separate, so this looks like 4 scans.
    for (uint8_t div = 0; div < 4; div++) {
      uint16_t rawButtonReadings = readRawStopTabButtons(div);

      stopTabButtonDebouncers[div].update(rawButtonReadings);
      // Button 0 == MSB, hence the 15 - btn.
      stopTabButtonDebouncers[div].forEachFallingEdge([div](int i) {
        onStopTabButtonPressed(div, 15 - i);
      });
    }
  }

  // Scan pistons.
  now = micros();
  if (now - lastPistonScan > PISTON_POLL_ITVL_MICROS) {
    lastPistonScan = now;

    uint8_t rawPistonReading = readRawPistons();

    pistonDebouncer.update(rawPistonReading);
    pistonDebouncer.forEachFallingEdge([](int i) {
      onPistonPressed(i);
    });
    pistonDebouncer.forEachRisingEdge([](int i) {
      onPistonReleased(i);
    });
  }

  // MIDI Controllers should discard incoming MIDI messages.
  while (MIDI.read()) {
  }
}
