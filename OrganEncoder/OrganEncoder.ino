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
constexpr uint16_t STOP_TAB_KEY_POLL_ITVL_MICROS = 1500;
constexpr uint16_t PISTON_POLL_ITVL_MICROS = 2500;

constexpr uint8_t N_DEBOUNCE_STEPS_STOP_TABS = 5;
constexpr uint8_t N_DEBOUNCE_STEPS_PISTONS = 5;


MCP23017 stopTabMcps[4][2] = {
  {MCP23017(0x20), MCP23017(0x21)},
  {MCP23017(0x22), MCP23017(0x23)},
  {MCP23017(0x24), MCP23017(0x25)},
  {MCP23017(0x26), MCP23017(0x27)},
};

// Not using a debouncing library because they all seem to need direct pin access. We are
// debouncing readings from GPIO expanders. We will keep the last 5 readings from each button. All
// the buttons are active low, so the "natural" initial state is all 1s. Note however that each
// bank has *15* buttons, not 16, so we hereby decree that the LSB will always be 1 by convention.
// Any other code that detects key presses must take care to assume this.
uint16_t stopTabKeyReadings[4][N_DEBOUNCE_STEPS_STOP_TABS] = {
  {0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF},
  {0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF},
  {0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF},
  {0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF},
};
uint8_t stopTabPollCtr = 0;
uint16_t debouncedStopTabKeyState[4] = {0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF};
uint16_t stopState[4] = {0, 0, 0, 0};

uint8_t pistonReadings[N_DEBOUNCE_STEPS_PISTONS] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
uint8_t pistonPollCtr = 0;
uint8_t debouncedPistonState = 0xFF;

unsigned long int lastStopTabKeyScan = 0;
unsigned long int lastPistonScan = 0;
unsigned long int now = 0;

bool awaitingSavePreset = 0;

// TODO: Put these in EEPROM.
uint16_t presets[8][4] = {
  {0, 0, 0, 0},
  {0, 0, 0, 0},
  {0, 0, 0, 0},
  {0, 0, 0, 0},
  {0, 0, 0, 0},
  {0, 0, 0, 0},
  {0, 0, 0, 0},
  {0, 0, 0, 0},
};

MIDI_CREATE_INSTANCE(HardwareSerial, Serial1, MIDI);

uint16_t reverseByte(uint16_t b) {
  b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
  b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
  b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
  return b;
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

void stopTabKeyPressed(uint8_t division, uint8_t button) {
  uint16_t stopMask = 1 << (15 - button);
  stopState[division] ^= stopMask;

  writeStopStateToLeds(division);

  uint8_t controlNumber = 102 + button;
  uint8_t controlValue = stopState[division] & stopMask ? 127 : 0;

  // Humans talk about MIDI channels 1-16. I would have expected computers to speak of MIDI
  // channels 0-15, but the library we're using does not do this; it expects 1-16. Hence the +1.
  uint8_t channel = division + 1;

  MIDI.sendControlChange(controlNumber, controlValue, channel);
}

void pollStopTabKeys() {
  for (uint8_t i = 0; i < 4; i++) {
    // Overwrite oldest history entry with current.
    stopTabKeyReadings[i][stopTabPollCtr] = 0;

    // In the following lines, for both ports A and B, the MSB corresponds to pin 7 and LSB to pin 0.

    // For the first chip of every pair, need to read both ports, as button 8 is routed
    // to GPA0, though buttons 1-7 are on GPB0-6 (in reverse order).
    stopTabKeyReadings[i][stopTabPollCtr] |= stopTabMcps[i][0].readPort(MCP23017Port::B) & 0x7F;
    stopTabKeyReadings[i][stopTabPollCtr] <<= 1;
    stopTabKeyReadings[i][stopTabPollCtr] |= stopTabMcps[i][0].readPort(MCP23017Port::A) & 0x01;
    stopTabKeyReadings[i][stopTabPollCtr] <<= 7;
    // For the second chip of every pair, only need to read port B, ignoring GPB7 (MSB), since
    // there are only 7 buttons hooked up to this one (GPB0-6).
    stopTabKeyReadings[i][stopTabPollCtr] |= stopTabMcps[i][1].readPort(MCP23017Port::B) & 0x7F;
    stopTabKeyReadings[i][stopTabPollCtr] <<= 1;
    // By convention, LSB needs to be 1 to represent the imaginary 16th button, never pressed.
    stopTabKeyReadings[i][stopTabPollCtr] |= 0x0001;

    // Run through history, compute stable state. At the end of this loop, a 1 in position i of
    // stableHigh means that the reading at position i has been 1 at every step in recorded
    // history, and a 0 in position i of stableLow means that the reading at position i has been
    // 0 at every step. The other 2 possibilities are noisy, and hence ignored.
    uint16_t stableHigh = 0xFFFF;
    uint16_t stableLow = 0;
    for (uint8_t h = 0; h < N_DEBOUNCE_STEPS_STOP_TABS; h++) {
      stableHigh &= stopTabKeyReadings[i][h];
      stableLow |= stopTabKeyReadings[i][h];
    }

    // Iterate over the falling edge bits. Each 1 here represents a button press, where the
    // current debounced state is 1 (high) and the last several readings have been 0 (low).
    uint16_t fallingEdges = (debouncedStopTabKeyState[i] & ~stableLow);
    // Do one bit shift before we start reading so that we ignore the meaningless LSB.
    fallingEdges >>= 1;
    for (uint8_t j = 0; j < 15; j++) {
      if (fallingEdges & 0x0001) {
        // TODO: Separate concerns better. This function shouldn't be known about in this context.
        stopTabKeyPressed(i, 14 - j);
      }
      fallingEdges >>= 1;
    }

    // Finally, update the debounced state.
    debouncedStopTabKeyState[i] |= stableHigh;
    debouncedStopTabKeyState[i] &= stableLow;
  }
  stopTabPollCtr = (stopTabPollCtr + 1) % N_DEBOUNCE_STEPS_STOP_TABS;
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

void generalCancel() {
  // Clear stop state internally
  for (uint8_t i = 0; i < 4; i++) {
    stopState[i] = 0;
    writeStopStateToLeds(i);
  }
  // Clear stop state "externally"
  emitStopState();
}

void pistonPressed(uint8_t i) {
  if (i == 0) {
    // piston 0 is "save" button
    awaitingSavePreset = true;
  } else if (i == 1) {
    // piston 1 is "GC" button
    generalCancel();
  } else {
    // the rest are presets
    if (awaitingSavePreset) {
      for (uint8_t j = 0; j < 4; j++) {
        presets[i][j] = stopState[j];
      }
    } else {
      for (uint8_t j = 0; j < 4; j++) {
        stopState[j] = presets[i][j];
        writeStopStateToLeds(j);
      }
      emitStopState();
    }
  }
}

void pistonReleased(uint8_t i) {
  // The only piston we really care about when released is the "save" button.
  if (i == 0) {
    awaitingSavePreset = false;
  }
}

void pollPistons() {
  // Load button state into the 74HC165 with a pulse.
  digitalWrite(PISTON_LOAD_PIN, LOW);
  digitalWrite(PISTON_LOAD_PIN, HIGH);

  // Overwrite oldest history entry with current.
  pistonReadings[pistonPollCtr] = 0;
  // Ensure clock pin low first...
  digitalWrite(PISTON_CLOCK_PIN, LOW);
  // ...then clock the data into the piston readings. We do this manually because the arduino
  // shiftIn implementation pulses the clock *before* reading, which is the wrong order for the
  // 74HC165N.
  for (uint8_t i = 0; i < 8; i++) {
    pistonReadings[pistonPollCtr] |= (digitalRead(PISTON_DATA_PIN) << i);
    digitalWrite(PISTON_CLOCK_PIN, HIGH);
    digitalWrite(PISTON_CLOCK_PIN, LOW);
  }

  // Run through history, compute stable state. At the end of this loop, a 1 in position i of
  // stableHigh means that the reading at position i has been 1 at every step in recorded
  // history, and a 0 in position i of stableLow means that the reading at position i has been
  // 0 at every step. The other 2 possibilities are noisy, and hence ignored.
  uint8_t stableHigh = 0xFF;
  uint8_t stableLow = 0;
  for (uint8_t h = 0; h < N_DEBOUNCE_STEPS_PISTONS; h++) {
    stableHigh &= pistonReadings[h];
    stableLow |= pistonReadings[h];
  }

  // Iterate over the falling edge bits. Each 1 here represents a button press, where the
  // current debounced state is 1 (high) and the last several readings have been 0 (low).
  uint8_t fallingEdges = (debouncedPistonState & ~stableLow);
  for (uint8_t j = 0; j < 8; j++) {
    if (fallingEdges & 0x01) {
      pistonPressed(j);
    }
    fallingEdges >>= 1;
  }
  uint8_t risingEdges = (~debouncedPistonState & stableHigh);
  for (uint8_t j = 0; j < 8; j++) {
    if (risingEdges & 0x01) {
      pistonReleased(j);
    }
    risingEdges >>= 1;
  }

  // Finally, update the debounced state.
  debouncedPistonState |= stableHigh;
  debouncedPistonState &= stableLow;

  pistonPollCtr = (pistonPollCtr + 1) % N_DEBOUNCE_STEPS_PISTONS;
}

void setup() {
  Wire.begin();

  // Pins 7 on both ports can only be used as output, due to a bug in the MCP23017 chip. For the
  // first chip of each pair, we swapped one of the inputs onto port A. For the second chip of
  // each pair, we are not using GPIO pins A7 or B7, because each division (swell, great, choir,
  // pedal) only has 15 input buttons and 15 output LEDs. So for each division, we will have to
  // read 3 GPIO ports: A and B on the first MCP23017, but only B on the second chip.
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 2; j++) {
      stopTabMcps[i][j].init();
      stopTabMcps[i][j].portMode(MCP23017Port::A, 0x01);
      stopTabMcps[i][j].portMode(MCP23017Port::B, 0x7F);
      stopTabMcps[i][j].writeRegister(MCP23017Register::GPIO_A, 0xFE);
      stopTabMcps[i][j].writeRegister(MCP23017Register::GPIO_B, 0x80);
    }
  }

  // Set up initial state for 74HC165 shift register that reads pistons
  pinMode(PISTON_LOAD_PIN, OUTPUT);
  pinMode(PISTON_CLOCK_PIN, OUTPUT);
  pinMode(PISTON_DATA_PIN, INPUT_PULLUP);
  digitalWrite(PISTON_LOAD_PIN, HIGH);
  digitalWrite(PISTON_CLOCK_PIN, LOW);

  MIDI.begin(MIDI_CHANNEL_OMNI);
}

void loop() {
  now = micros();
  if (now - lastStopTabKeyScan > STOP_TAB_KEY_POLL_ITVL_MICROS) {
    pollStopTabKeys();
    lastStopTabKeyScan = now;
  }

  now = micros();
  if (now - lastPistonScan > PISTON_POLL_ITVL_MICROS) {
    pollPistons();
    lastPistonScan = now;
  }

  // MIDI Controllers should discard incoming MIDI messages.
  while (MIDI.read()) {
  }
}
