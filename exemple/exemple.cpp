#include <Arduino.h>
#include <FrequencyMeter.h>

FrequencyMeter freqMeter;
float frequency = 0, old_frequency = 0;

void setup() {
    Serial.begin(921600);
    freqMeter.initFrequencyMeter();
}

void loop() {
    freqMeter.oscillatorTestLoop();
    frequency = freqMeter.getFrequency();
    if (frequency && frequency != old_frequency) {
        old_frequency = frequency;
        Serial.print("Frequency: ");
        Serial.print(frequency);
        Serial.println(" Hz");
    }
}