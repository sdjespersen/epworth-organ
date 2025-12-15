// Eventually this will be the decoder. For now it's a rough draft...

constexpr int CLK_PIN = 17;
constexpr int DTA_PIN = 18;
constexpr int LATCH_PIN = 19;
constexpr int OUTPUT_ENABLE_PIN = 33;

constexpr int BLINK_ITVL_MS = 500;

int counter = 0;

elapsedMillis timeSinceLastBlink = 0;

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
  shiftOut(DTA_PIN, CLK_PIN, MSBFIRST, 0);
  shiftOut(DTA_PIN, CLK_PIN, MSBFIRST, 0);
  digitalWrite(LATCH_PIN, HIGH);
  digitalWrite(LATCH_PIN, LOW);

  // Setup done; enable outputs.
  digitalWrite(OUTPUT_ENABLE_PIN, LOW);

  Serial.begin(9600);
}

void loop() {
  if (timeSinceLastBlink > BLINK_ITVL_MS) {
    timeSinceLastBlink = 0;
    int r16 = counter % 16;
    // We wired up the outputs weird. Oopsie. Remedied easily enough by mapping:
    // 0 -> 0
    // 1 -> 1
    // 2 -> 2
    // 3 -> 3
    // 4 -> 7
    // 5 -> 6
    // 6 -> 5
    // 7 -> 4
    int r8 = counter % 8;
    if (r8 > 3) {
      r16 = r16 + 11 - 2 * r8;
    }
    shiftOut(DTA_PIN, CLK_PIN, MSBFIRST, (1 << r16) >> 8);
    shiftOut(DTA_PIN, CLK_PIN, MSBFIRST, 1 << r16);
    digitalWrite(LATCH_PIN, HIGH);
    digitalWrite(LATCH_PIN, LOW);
    counter++;
  }
}
