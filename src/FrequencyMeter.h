#ifndef FREQUENCYMETER_H
#define FREQUENCYMETER_H
#include <Arduino.h>
#include "stdio.h"                                                        // Library STDIO
#include "driver/ledc.h"                                                  // Library ESP32 LEDC
#include "driver/pcnt.h"                                                  // Library ESP32 PCNT
#include "soc/pcnt_struct.h"

/*
The project:

A high accuracy frequency meter using ESP32,
measuring up to 40 MHz. Very stable. Optionally, you can test the frequency meter with internal oscillator.
Caution = input signal to Freq Meter - only 3.3 Volts! If you want 5 Volts, use level converter.

Definitions:

GPIO_34 = Freq Meter Input
GPIO_33 = Oscillator output - to test Frequency Meter
To test freq Meter with internal oscillator, make connection between GIPO_34 and GPIO_33 (optional).
Change frequency, inputting value at Arduino console (1 Hz to 40 MHz)

The frequency meter is divided into 5 parts:
1. Pulse counter;
2. Counting time control;
3. Printing the result;
4. Space for other functions.
5. Signal programmed Oscillator

1. The pulse counter uses the ESP32 pcnt.
Pcnt has the following parameters:
a. input port;
b. input channel;
c. control port;
d. count on Pulse raising;
e. count on Pulse falling;
f. counting only with Control - high level;
g. maximum count limit.

2. Counting time control uses the high resolution esp-timer:
The esp-timer has the following parameter:
a. time control;

5. Frequency Oscillator for tests, uses ledc peripheral:
The ledc has the following parameters:
a. output port;
B. lcd channel;
ç. frequency;
d. resolution of ledc;
e. duty cycle at 50%;

Operation:

The high level of control port enable the counter to count the pulses that arrive at the input port.
Pulses are counted both as the pulse rising and falling, to improve the counting average.
The sampling time is defined by the high resolution esp-timer, and it is defined in 1 second, in the sample-time variable.
If the count is greater than 20000 pulses during the counting time, overflow occurs and for with each overflow that occurs
the multPulses variable is incremented. Then pulse counter is cleared and proceed to counting.
Unfortunately the Pulse Counter has only 16 bits that may be used.

When the sampling time ends, a routine is called and the value in the pulse counter is read and saved.
A flag is set on to indicating that the pulse reading has ended.

In the loop, when verifying if the flag is on indicates that the pulse reading has finished, the value is calculated by multiplying
the number of overflow by 20000 and adding to the number of remaining pulses and dividing by 2, because it counted 2 times.

As the pulses are counted on the way up and down, the count is reel the frequency.
In the frequency value, commas are inserted and printed on the serial monitor.
The registers are reset and the input control port is set to a high level again and the pulse count starts.

It also has a signal oscillator that generates pulses, and can be used for testing.
This oscillator can be configured to generate frequencies up to 40 MHz.
We use the LEDC peripheral of ESP32 to generate frequency that can be used as a test.
The base frequency value is 12543 Hz, but it can be typed to another value on the serial monitor.
The default duty cycle was set to 50%, and the resolution is properly calculated.
The output port of this generator is currently defined as GPIO 33.

Internally using GPIO matrix, the input pulse was directed to the ESP32 native LED,
so the LED will flash at the input frequency.*/

#define PCNT_COUNT_UNIT       PCNT_UNIT_0                                 // Set Pulse Counter Unit - 0 
#define PCNT_COUNT_CHANNEL    PCNT_CHANNEL_0                              // Set Pulse Counter channel - 0 

#define PCNT_INPUT_SIG_IO     GPIO_NUM_34                                 // Set Pulse Counter input - Freq Meter Input GPIO 34
#define LEDC_HS_CH0_GPIO      GPIO_NUM_33                                 // Saida do LEDC - gerador de pulsos - GPIO_33
#define PCNT_INPUT_CTRL_IO    GPIO_NUM_35                                 // Set Pulse Counter Control GPIO pin - HIGH = count up, LOW = count down  
#define OUTPUT_CONTROL_GPIO   GPIO_NUM_32                                 // Timer output control port - GPIO_32

// #define IN_BOARD_LED          GPIO_NUM_2                                  // ESP32 native LED - GPIO 2

void set_gpio_if_needed(gpio_num_t gpio_num, uint32_t level);

typedef float reel;

class FrequencyMeter
{
public:
    //Choisissez des valeurs de sample_time_us en faisant en sorte que le calcul factor = ((1 000 000)/sample_time_us)/2  donne un resultat rond
    FrequencyMeter(uint32_t sample_time_us = 1000000, uint32_t count_overflow = 20000);
    ~FrequencyMeter();

    void initFrequencyMeter ();
    void setOscFrequence (uint32_t freq);// Oscillator frequency (may be 1 Hz to 40 MHz)

    void oscillatorTestLoop();
    reel getFrequency();

    static FrequencyMeter* instance;  // Pointer to the current instance
    static void IRAM_ATTR pcntOverflowHandler(void *arg);
    static void readPCNT(void *p);

    double getFrequencyFactor() {
        return frequency_factor;
    }
    
private:
    bool            flag          ;                                        // Flag to enable print frequency reading
    uint32_t        overflow      ;                                       // Max Pulse Counter value
    int16_t         pulses        ;                                      // Pulse Counter value
    uint32_t        mult_pulses    ;                                     // Quantidade de overflows do contador PCNT
    uint32_t        sample_time   ;                                    // sample time in us to count pulses                                  
    uint32_t        m_duty         ;                                   // Duty value
    uint32_t        resolution    ;                                  // Resolution value
    reel            frequency;                                      // frequency value
    reel            frequency_factor     ;                         // frequency factor = (1e6 / sample_time) / 2.0
                                                                
    esp_timer_create_args_t create_args;                         // Create an esp_timer instance
    esp_timer_handle_t timer_handle;                            // Create an single timer

    portMUX_TYPE timer_mux         ;                           // portMUX_TYPE to do synchronism

    ledc_timer_config_t ledc_timer = {};                    // LEDC timer config instance
    ledc_channel_config_t ledc_channel = {};               // LEDC Channel config instance

    void initPCNT(void) ;
};



#endif // FREQUENCYMETER_H