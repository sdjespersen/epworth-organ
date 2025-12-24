/* Send MIDI over serial. */

#include <MIDI.h>
#include <Wire.h>
#include <MCP23017.h>


// To save on CPU cycles, don't poll the stop tab keys too often. We could keep adjusting this
// number upward as long as we don't drop any key presses.
constexpr uint8_t STOP_TAB_KEY_POLL_ITVL_MICROS = 500;

constexpr uint8_t N_DEBOUNCE_STEPS_STOP_TABS = 5;

MIDI_CREATE_INSTANCE(HardwareSerial, Serial1, MIDI);


MCP23017 stopTabMcps[4][2] = {
  {MCP23017(0x20), MCP23017(0x21)},
  {MCP23017(0x22), MCP23017(0x23)},
  {MCP23017(0x24), MCP23017(0x25)},
  {MCP23017(0x26), MCP23017(0x27)},
};

// Not using a debouncing library because they all seem to need direct pin access.
// We are debouncing readings from GPIO expanders.
// Keep the last 5 readings from each button. All the buttons are active low, so the
// "natural" initial state is all 1s. Note however that each bank has *15* buttons,
// not 16, so we hereby decree that the LSB of will always be 1 by convention. Any
// other code that detects key presses must take care to assume this.
uint16_t stopTabKeyReadings[4][N_DEBOUNCE_STEPS_STOP_TABS] = {
  {0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF},
  {0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF},
  {0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF},
  {0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF},
};
uint8_t stopTabPollCtr = 0;
uint16_t debouncedState[4] = {0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF};
uint16_t stopState[4] = {0, 0, 0, 0};

elapsedMicros sinceLastStopTabKeyScan = 0;

void stopTabKeyPressed(uint8_t division, uint8_t button) {
  // Button j, division i was pressed...
  // TODO: Take meaningful action in this condition (emit MIDI, toggle stop state)
  Serial.print("We have a falling edge for division ");
  Serial.print(division);
  Serial.print(", button #");
  Serial.println(button);

  uint8_t controlNumber = 102 + button;
  uint8_t controlValue = 127; // or 0; depends on stopState!

  Serial.print("Sending MIDI CC {number: ");
  Serial.print(controlNumber);
  Serial.print(", value: ");
  Serial.print(controlValue);
  Serial.print(", channel: ");
  Serial.print(division);
  Serial.println("}");

  // MIDI.sendControlChange(controlNumber, controlValue, division);
}

void pollStopTabKeys() {
  for (uint8_t i = 0; i < 4; i++) {
    // Overwrite oldest history entry with current.
    stopTabKeyReadings[i][stopTabPollCtr] = 0;

    // TODO: Determine whether pin GPA7/GPB7 is the MSB or LSB! It matters greatly! The
    // current implementation assumes it is the LSB.

    // For the first chip of every pair, need to read both ports, as button 8 is routed
    // to GPA0, though buttons 1-7 are on GPB0-6.
    stopTabKeyReadings[i][stopTabPollCtr] |= stopTabMcps[i][0].readPort(MCP23017Port::B) & 0xFE;
    stopTabKeyReadings[i][stopTabPollCtr] |= stopTabMcps[i][0].readPort(MCP23017Port::A) & 0x01;
    stopTabKeyReadings[i][stopTabPollCtr] <<= 8;
    // For the second chip of every pair, only need to read port B, ignoring last pin, since
    // there are only 7 buttons hooked up to this one (GPB0-6).
    stopTabKeyReadings[i][stopTabPollCtr] |= stopTabMcps[i][1].readPort(MCP23017Port::B) & 0xFE;
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
    uint16_t fallingEdges = (debouncedState[i] & ~stableLow);
    // Do one bit shift before we start reading so that we ignore the meaningless LSB.
    fallingEdges >>= 1;
    for (uint8_t j = 14; j >= 0; j--) {
      if (fallingEdges & 0x0001) {
        // TODO: Separate concerns better. This function shouldn't be known about in this context.
        stopTabKeyPressed(i, j);
      }
      fallingEdges >>= 1;
    }

    // Finally, update the debounced state.
    debouncedState[i] |= stableHigh;
    debouncedState[i] &= stableLow;
  }
  stopTabPollCtr = (stopTabPollCtr + 1) % N_DEBOUNCE_STEPS_STOP_TABS;
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
      stopTabMcps[i][j].portMode(MCP23017Port::A, 0b10000000);
      stopTabMcps[i][j].portMode(MCP23017Port::B, 0b11111110);
      stopTabMcps[i][j].writeRegister(MCP23017Register::GPIO_A, 0x00);
      stopTabMcps[i][j].writeRegister(MCP23017Register::GPIO_B, 0x00);
    }
  }

  Serial.begin(9600);
  MIDI.begin(MIDI_CHANNEL_OMNI);
}

void loop() {
  if (sinceLastStopTabKeyScan > STOP_TAB_KEY_POLL_ITVL_MICROS) {
    pollStopTabKeys();
  }
}
