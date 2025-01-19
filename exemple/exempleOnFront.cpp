#include <Arduino.h>
#include <FrequencyMeter.h>

FrequencyMeter freqMeter(50000, 20000, GPIO_NUM_34, GPIO_NUM_35, GPIO_NUM_32, PCNT_UNIT_0, PCNT_CHANNEL_0);
float frequency = 0, old_frequency = 0;

void setup() {
    Serial.begin(921600);
    freqMeter.initFrequencyMeter();
    freqMeter.initOscillator(GPIO_NUM_33);

    printf("Sample Time : %u us\n", freqMeter.getSampleTime());
    printf("Frequency Factor : %f\n", freqMeter.getFrequencyFactor());

    freqMeter.setOscFrequence(1234);
    Serial.println("\nFrequency Meter Initialized");
    Serial.println("Enter the frequency you want and press enter");
}

void loop() {
    freqMeter.oscillatorTestLoop();//Tapez sur le moniteur la frequence voulue puis appuyez sur entrée

    frequency = freqMeter.getFrequencyOnFront();
    if (frequency && frequency != old_frequency) {
        old_frequency = frequency;
        Serial.print("Frequency: ");
        Serial.print(frequency);
        Serial.println(" Hz");
    }
}