/* Send MIDI over serial. */

#include <MIDI.h>

constexpr int MESSAGE_SEND_ITVL_MS = 500;

elapsedMillis sinceLastMessage = 0;
int counter = 0;

MIDI_CREATE_INSTANCE(HardwareSerial, Serial1, MIDI);

void setup() {
  MIDI.begin(MIDI_CHANNEL_OMNI);

  // This is the diagnostic serial, not the TX/RX.
  Serial.begin(9600);
}

void loop() {
  // Cycle through MIDI CC messages
  // CC 102-117 (inclusive), on for .5 sec, off for .5, move on to the next one
  // On: 127. Off: 0. 64 total distinct CCs.
  if (sinceLastMessage > MESSAGE_SEND_ITVL_MS) {
    uint8_t controlNumber = 102 + ((counter / 2) % 16);
    uint8_t controlValue = (counter % 2 == 0) ? 127 : 0;
    uint8_t channel = 1 + (counter / 32); // yes, integer division

    Serial.print("Sending CC {number: ");
    Serial.print(controlNumber);
    Serial.print(", value: ");
    Serial.print(controlValue);
    Serial.print(", channel: ");
    Serial.print(channel);
    Serial.println("}");

    MIDI.sendControlChange(controlNumber, controlValue, channel);

    counter = (counter + 1) % 128;
    sinceLastMessage = 0;
  }
}
