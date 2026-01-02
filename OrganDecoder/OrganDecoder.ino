#include <MIDI.h>

constexpr int FLUSH_STOP_STATE_ITVL_MS = 2;

constexpr int CLK_PIN = 17;
constexpr int DTA_PIN = 18;
constexpr int LATCH_PIN = 19;
constexpr int OUTPUT_ENABLE_PIN = 33;

static const unsigned int B[] = {0x55555555, 0x33333333, 0x0F0F0F0F, 0x00FF00FF};
static const unsigned int S[] = {1, 2, 4, 8};

// This table is used to invert a sequence of 4 bits.
static const unsigned char NIBBLE_REVERSE_LOOKUP[16] = {
  0x0, 0x8, 0x4, 0xc, 0x2, 0xa, 0x6, 0xe, 0x1, 0x9, 0x5, 0xd, 0x3, 0xb, 0x7, 0xf,
};

MIDI_CREATE_INSTANCE(HardwareSerial, Serial1, MIDI);

elapsedMillis sinceLastStopStateFlush = 0;
bool pendingStopChanges = false;

// Each int stores 16 bits of stop state. Order: swell, great, choir, pedal.
unsigned short stopState[4] = {0, 0, 0, 0};

unsigned long interleaveBits(unsigned long a, unsigned long b) {
  unsigned long x = a;
  unsigned long y = b;

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
  }
  pendingStopChanges = true;
}

void flushStopState() {
  // We need to interleave stop state, because from L to R, we actually have
  // - ch, gt, ch, gt, ... (16x) then sw, ped, sw, ped, ...
  // - 117, 117, 116, 116, 115, 115, ..., 102, 102, 117, 117, 116, 116, ..., 104, 104 (let's pretend 102, 102)

  // Interleaving choir and great stop states
  unsigned long leftHalfStopState = interleaveBits(stopState[1], stopState[2]);
  // Interleaving swell and pedal stop states
  unsigned long rightHalfStopState = interleaveBits(stopState[3], stopState[0]);

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
