#include <MIDI.h>

constexpr int FLUSH_STOP_STATE_ITVL_MS = 2;

constexpr int CLK_PIN = 17;
constexpr int DTA_PIN = 18;
constexpr int LATCH_PIN = 19;
constexpr int OUTPUT_ENABLE_PIN = 33;

static const uint32_t B[] = {0x55555555, 0x33333333, 0x0F0F0F0F, 0x00FF00FF};
static const uint32_t S[] = {1, 2, 4, 8};

// This table is used to invert a sequence of 4 bits.
static const unsigned char NIBBLE_REVERSE_LOOKUP[16] = {
  0x0, 0x8, 0x4, 0xc, 0x2, 0xa, 0x6, 0xe, 0x1, 0x9, 0x5, 0xd, 0x3, 0xb, 0x7, 0xf,
};

// These crescendo pedal settings are in the following format:
//   {division, stop number}
// where division 0 = swell, 1 = great, 2 = choir, 3 = pedal and
// stop numbers range from 0-15 arranged organist's L to R.
static const uint8_t CRESCENDO_PEDAL_ORDER[38][2] = {
  {3, 10}, // Swell to Pedal 8'
  {3, 11}, // Swell to Pedal 4'
  {3, 12}, // Great to Pedal 8'
  {3, 13}, // Choir to Pedal 8'
  {2, 14}, // Choir to Choir 4'
  {1, 9},  // Swell to Great 8'
  {1, 12}, // Choir to Great 8'
  {1, 13}, // Choir to Great 4'
  {1, 2},  // Dulciana 8'
  {0, 0},  // Gedeckt Flute 8'
  {1, 4},  // Flute Ouverte 4'
  {1, 1},  // Bourdon 8'
  {3, 1},  // Gedekct 16'
  {0, 1},  // Salicional 8'
  {2, 1},  // Gemshorn 8'
  {0, 2},  // Voix Celeste 8'
  {0, 4},  // Harmonic Flute 4'
  {0, 5},  // Block Flute 2'
  {2, 4},  // Gemshorn 4'
  {3, 2},  // Gemshorn 16'
  {3, 3},  // Principal 8'
  {2, 3},  // Koppel Flute 4'
  {2, 0},  // Koppel Flute 8'
  {1, 0},  // Principal 8'
  {1, 3},  // Octave 4'
  {1, 5},  // Fifteenth 2'
  {2, 6},  // Koppel Flute 2'
  {0, 3},  // Geigen Principal 8'
  {3, 0},  // Open Diapason 16'
  {1, 6},  // Plein Jeu IV (Fourniture)
  {3, 7},  // Flute 4'
  {0, 8},  // Clarion 4'
  {3, 8},  // Contra Trompette 16'
  {1, 10}, // Swell to Great 4'
  {0, 13}, // Swell to Swell 4'
  {2, 12}, // Choir to Choir 16'
  {0, 7},  // Trompette 8'
  {0, 6},  // Scharf III-IV
};
// TODO: Make this constexpr! It's not supposed to be mutable.
uint16_t CRESCENDO_INDUCED_STATES[39][4] = {0};
// static constexpr *uint16_t generateCrescendoInducedStates() {
  // uint16_t crescendoInducedStates[39][4] = {0};

  // return crescendoInducedStates;
// }
// uint16_t CRESCENDO_INDUCED_STATES[39][4] = {
//   generateCrescendoInducedStates()
// };

MIDI_CREATE_INSTANCE(HardwareSerial, Serial1, MIDI);

elapsedMillis sinceLastStopStateFlush = 0;
bool pendingStopChanges = false;

// Each int stores 16 bits of stop state. Order: swell, great, choir, pedal.
uint16_t stopState[4] = {0, 0, 0, 0};
uint16_t crescendoInducedStopState[4] = {0, 0, 0, 0};

uint32_t interleaveBits(uint32_t a, uint32_t b) {
  uint32_t x = a;
  uint32_t y = b;

  x = (x | (x << S[3])) & B[3];
  x = (x | (x << S[2])) & B[2];
  x = (x | (x << S[1])) & B[1];
  x = (x | (x << S[0])) & B[0];

  y = (y | (y << S[3])) & B[3];
  y = (y | (y << S[2])) & B[2];
  y = (y | (y << S[1])) & B[1];
  y = (y | (y << S[0])) & B[0];

  return (x | (y << 1));
}

void handleControlChange(byte channel, byte controlNumber, byte controlValue) {
  // Channel 1: swell, 2: great, 3: choir, 4: pedal (not the order they appear on the panel!)

  // Handle stop tab messages.
  if (102 <= controlNumber && controlNumber <= 117 && 1 <= channel && channel <= 4) {
    bitWrite(stopState[channel-1], controlNumber - 102, controlValue > 63);
    pendingStopChanges = true;
  }

  // Handle crescendo pedal messages
  if (controlNumber == 11 && channel == 5) {
    // map it down to 0-38 (divide by 3.28)
    uint8_t crescStep = static_cast<uint8_t>(std::round(controlValue / 3.28));
    for (int div = 0; div < 4; div++) {
      crescendoInducedStopState[div] = CRESCENDO_INDUCED_STATES[crescStep][div];
    }
    pendingStopChanges = true;
  }
}

void flushStopState() {
  // We need to interleave stop state, because from L to R, we actually have
  // - ch, gt, ch, gt, ... (15x) then sw, ped, sw, ped, ... (14x)
  // - 116, 116, 115, 115, ..., 102, 102, 115, 115, ..., 102, 102
  // (Note that 116 on sw and ped are not hooked up; that's why they're absent here

  // Interleaving choir and great stop states
  uint16_t greatState = stopState[1] | crescendoInducedStopState[1];
  uint16_t choirState = stopState[2] | crescendoInducedStopState[2];
  uint32_t leftHalfStopState = interleaveBits(greatState, choirState);
  // Interleaving swell and pedal stop states
  uint16_t pedalState = stopState[3] | crescendoInducedStopState[3];
  uint16_t swellState = stopState[0] | crescendoInducedStopState[0];
  uint32_t rightHalfStopState = interleaveBits(pedalState, swellState);

  byte toWrite;

  // Reverse the bottom nibble (4 bits) to compensate for our having reversed the order of pin outputs.
  for (int i = 0; i < 4; i++) {
    toWrite = 0xFF & (rightHalfStopState >> (8 * i));
    toWrite = (0xF0 & toWrite) + NIBBLE_REVERSE_LOOKUP[toWrite & 0x0F];
    shiftOut(DTA_PIN, CLK_PIN, LSBFIRST, toWrite);
  }

  for (int i = 0; i < 4; i++) {
    toWrite = 0xFF & (leftHalfStopState >> (8 * i));
    toWrite = (0xF0 & toWrite) + NIBBLE_REVERSE_LOOKUP[toWrite & 0x0F];
    shiftOut(DTA_PIN, CLK_PIN, LSBFIRST, toWrite);
  }

  digitalWrite(LATCH_PIN, HIGH);
  digitalWrite(LATCH_PIN, LOW);

  sinceLastStopStateFlush = 0;
  pendingStopChanges = false;
}

void setup() {
  // TODO: Move this into a compile-time constant expression for crescendo states!
  for (int i = 0; i < 38; i++) {
    for (int j = 0; j < 4; j++) {
      CRESCENDO_INDUCED_STATES[i+1][j] = CRESCENDO_INDUCED_STATES[i][j];
    }
    CRESCENDO_INDUCED_STATES[i+1][CRESCENDO_PEDAL_ORDER[i][0]] |= 1 << CRESCENDO_PEDAL_ORDER[i][1];
  }

  pinMode(CLK_PIN, OUTPUT);
  pinMode(DTA_PIN, OUTPUT);
  pinMode(LATCH_PIN, OUTPUT);
  pinMode(OUTPUT_ENABLE_PIN, OUTPUT);

  digitalWrite(CLK_PIN, LOW);
  digitalWrite(DTA_PIN, LOW);
  digitalWrite(LATCH_PIN, LOW);

  // OE is active low; here we are disabling until startup is done.
  digitalWrite(OUTPUT_ENABLE_PIN, HIGH);

  // Clear everything during startup
  flushStopState();

  MIDI.setHandleControlChange(handleControlChange);

  MIDI.begin(MIDI_CHANNEL_OMNI);

  // Setup done; enable outputs.
  digitalWrite(OUTPUT_ENABLE_PIN, LOW);
}

void loop() {
  // No need to check for incoming messages explicitly; they are handled by callbacks.
  MIDI.read();

  if (pendingStopChanges && sinceLastStopStateFlush > FLUSH_STOP_STATE_ITVL_MS) {
    flushStopState();
  }
}
