#include <Arduino.h>
#include "mySignal.h"

/*
The functions getFrequency() and getFrequencyOnFront() may not be suitable for the signal you want to read.
In this case, you can redefine them in your derived class.
Create a derived class of FrequencyMeter and implement your own getFrequency().

Here is an example of redefining getFrequency() for a signal where you expect to receive a frame of 6 frequencies at any time.
The frame lasts 210ms and each frequency lasts 35ms.
The 6 frequencies are sent one after the other.

To read this frame, it is immediately clear why getFrequency() and getFrequencyOnFront() are not suitable.
Indeed, getFrequency() and getFrequencyOnFront() read with a sample time that could start reading in the middle 
of one frequency and end in the middle of the next. This distorts the reading.

To read this frame, we will start the PCNT counter each time we see an edge.
The sample time will be 31250us. At the end of each reading, we wait for the first frequency to finish 
before starting the reading of the next one. So we wait 35ms - 31250us = 3750us.
*/

MySignal emission;//, GPIO_NUM_34, GPIO_NUM_35, GPIO_NUM_32, PCNT_UNIT_0, PCNT_CHANNEL_0);

void setup() {
    Serial.begin(921600);
    emission.init();

    emission.frame(0x12, (0x3456));
    Serial.printf("\nFrequency factor : %f\n", emission.getFrequencyFactor());
    Serial.println("Frequency Meter Initialized");
    delay(1000);
}

float frequency = 0, old_frequency = 0;
void loop() {
    frequency = emission.Frequency();
    if (frequency && frequency != old_frequency) {
        old_frequency = frequency;
        // Serial.print("Frequency: ");
        // Serial.print(frequency);
        // Serial.printf(" Hz, ton : 0x%X\n", emission.FDC(frequency));
        uint8_t ton = emission.FDC(frequency);
        if(ton != 0xF){
            Serial.printf("Frequency: %f Hz, TON : 0x%X\n", frequency, ton);
        }
    }
    
    emission.emissionLoop();
    
    while(Serial.available()){
        char c = Serial.read();
        if(c == 'a'){
            emission.frame(0x21, (0xA400));
        }
    }
}