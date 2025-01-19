#include <Arduino.h>
#include <FrequencyMeter.h>

FrequencyMeter freqMeter(31250, 20000, GPIO_NUM_34, GPIO_NUM_35, GPIO_NUM_32, PCNT_UNIT_0, PCNT_CHANNEL_0);
FrequencyMeter freqMeter2(31250, 20000, GPIO_NUM_25, GPIO_NUM_26, GPIO_NUM_27, PCNT_UNIT_1, PCNT_CHANNEL_1);

float frequency = 0, old_frequency = 0;
float frequency2 = 0, old_frequency2 = 0;

void setup() {
    Serial.begin(921600);
    freqMeter.initFrequencyMeter();
    freqMeter2.initFrequencyMeter();

    freqMeter.initOscillator(GPIO_NUM_33);
    freqMeter2.initOscillator(GPIO_NUM_14);

    printf("Sample Time : %u us\n", freqMeter.getSampleTime());
    printf("Frequency Factor : %f\n", freqMeter.getFrequencyFactor());

    printf("Sample Time 2: %u us\n", freqMeter2.getSampleTime());
    printf("Frequency Factor 2: %f\n", freqMeter2.getFrequencyFactor());

    freqMeter.setOscFrequence(1234);
    freqMeter2.setOscFrequence(10000);
    
    Serial.println("\nFrequency Meter Initialized");
    Serial.println("Enter the frequency you want and press enter");
    delay(1000);
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

    frequency2 = freqMeter2.getFrequencyOnFront();
    if (frequency2 && frequency2 != old_frequency2) {
        old_frequency2 = frequency2;
        Serial.print("Frequency2: ");
        Serial.print(frequency2);
        Serial.println(" Hz");
    }
}