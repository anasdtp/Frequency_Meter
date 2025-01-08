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

Example:

Freq Meter Input = GPIO_34
Oscillator Output = GPIO_33 - to test Frequency Meter
Control signal = GPIO_35 - to enable/disable counting, input
Control signal = GPIO_32 - to enable/disable counting, output
The GPIO_35 and GPIO_32 must be connected to each other.

To test freq Meter with internal oscillator, make connection between GIPO_34 and GPIO_33 (optional).
Change frequency, inputting value at Arduino console (1 Hz to 40 MHz) with the fonction OscillatorTestLoop.

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
The default duty cycle was set to 50%, and the resolution is properly calculated.
*/                           

void set_gpio_if_needed(gpio_num_t gpio_num, uint32_t level);

typedef float reel;

/**
 * @brief Constructs a FrequencyMeter object with specified parameters.
 * 
 * @param sample_time_us The sample time in microseconds. Default is 1,000,000 us (1 second).
 * @param count_overflow The count overflow value. Default is 20,000.
 * @param GPIO_FREQUENCE_INPUT The GPIO pin for frequency input. Default is pin GPIO_NUM_34.
 * @param GPIO_COUNT_CONTROL_INPUT The GPIO pin for count control input. Default is GPIO_NUM_35. Must be connected to GPIO_COUNT_CONTROL_OUTPUT.
 * @param GPIO_COUNT_CONTROL_OUTPUT The GPIO pin for count control output. Default is GPIO_NUM_32. Must be connected to GPIO_COUNT_CONTROL_INPUT.
 * @param PCNT_UNIT The PCNT unit number. Default is PCNT_UNIT_0. There are 8 PCNT units available on ESP32.
 * @param PCNT_CHANNEL The PCNT channel number. Default is PCNT_CHANNEL_0. There are 2 channels available on each PCNT unit.
 */
class FrequencyMeter
{
public:
    //Choisissez des valeurs de sample_time_us en faisant en sorte que le calcul factor = ((1 000 000)/sample_time_us)/2  donne un resultat rond
    /**
     * @brief Constructs a FrequencyMeter object with specified parameters.
     * 
     * @param sample_time_us The sample time in microseconds. Default is 1,000,000 us (1 second).
     * @param count_overflow The count overflow value. Default is 20,000.
     * @param GPIO_FREQUENCE_INPUT The GPIO pin for frequency input. Default is pin GPIO_NUM_34.
     * @param GPIO_COUNT_CONTROL_INPUT The GPIO pin for count control input. Default is GPIO_NUM_35. Must be connected to GPIO_COUNT_CONTROL_OUTPUT.
     * @param GPIO_COUNT_CONTROL_OUTPUT The GPIO pin for count control output. Default is GPIO_NUM_32. Must be connected to GPIO_COUNT_CONTROL_INPUT.
     * @param PCNT_UNIT The PCNT unit number. Default is PCNT_UNIT_0. There are 8 PCNT units available on ESP32.
     * @param PCNT_CHANNEL The PCNT channel number. Default is PCNT_CHANNEL_0. There are 2 channels available on each PCNT unit.
     */
    FrequencyMeter(uint32_t sample_time_us = 1000000, uint32_t count_overflow = 20000, gpio_num_t GPIO_FREQUENCE_INPUT = GPIO_NUM_34, gpio_num_t GPIO_COUNT_CONTROL_INPUT = GPIO_NUM_35, gpio_num_t GPIO_COUNT_CONTROL_OUTPUT = GPIO_NUM_32, pcnt_unit_t PCNT_UNIT = PCNT_UNIT_0, pcnt_channel_t PCNT_CHANNEL = PCNT_CHANNEL_0);
    ~FrequencyMeter();

    /**
     * @brief Initializes the PCNT (Pulse Counter) unit with specified parameters.
     * 
     * This function configures and initializes the PCNT (pulse counter) unit to count pulses from a specified GPIO pin.
     * It allows you to configure the counter to count on rising edges, falling edges, or both, depending on the application's needs.
     * The function also sets up the counter's overflow limit and initial state.
     * 
     * @param gpio_input The GPIO pin number for the input signal to count pulses from.
     * @param gpio_ctrl The GPIO pin number for the control signal. May be unused in some configurations.
     * @param unit The PCNT unit to use (e.g., `PCNT_UNIT_0`, `PCNT_UNIT_1`).
     * @param channel The channel of the counter to use for the specified PCNT unit (e.g., `PCNT_CHANNEL_0`).
     * @param overflow_limit The overflow limit for the counter. If the counter reaches this value, it triggers an overflow event.
     * @param pos_edge_mode The counting mode for rising edges. Default is `PCNT_COUNT_INC` to increment on rising edges.
     * @param neg_edge_mode The counting mode for falling edges. Default is `PCNT_COUNT_INC`. Can be `PCNT_COUNT_DIS` to disable counting on falling edges.
     * 
     * @return void
     * 
     * @note This function configures the counter to start immediately counting after initialization with the provided parameters.
     */
    void initPCNT(gpio_num_t gpio_input, gpio_num_t gpio_ctrl, pcnt_unit_t unit, pcnt_channel_t channel, int16_t overflow_limit, pcnt_count_mode_t pos_edge_mode = PCNT_COUNT_INC, pcnt_count_mode_t neg_edge_mode = PCNT_COUNT_INC);

    /**
     * @brief Initializes everything that need to be initialazed to use the frequency meter
     * 
     * @param count_on_falling_edges If it's true, the pcnt counter will increase on falling edges. If not, disable counting on falling edges.
     */
    void initFrequencyMeter (bool count_on_falling_edges = true);

    /** @brief Gets the frequency value.
     * 
     * This function returns the frequency value in Hz.
     * 
     * @return The frequency value in Hz. Return 0 if the frequency is not available yet.
     */
    reel getFrequency();

    void changeSampleTime(uint32_t sample_time_us);
    

    /**
     * @brief Initializes the LEDC oscillator for generating a signal on the specified GPIO pin.
     * 
     * @param output_pin The GPIO pin number for the oscillator. Default is GPIO_NUM_33.
     * @param timer_num The LEDC timer to use (`LEDC_TIMER_0` to `LEDC_TIMER_3`) (e.g., `LEDC_TIMER_0`, `LEDC_TIMER_1`).
     * @param channel The LEDC channel to use (`LEDC_CHANNEL_0` to `LEDC_CHANNEL_7`) (e.g., `LEDC_CHANNEL_0`, `LEDC_CHANNEL_1`).
     */
    void initOscillator(gpio_num_t output_pin = GPIO_NUM_33, ledc_timer_t timer_num = LEDC_TIMER_0, ledc_channel_t channel = LEDC_CHANNEL_0);

    /** @brief Sets the frequency of the oscillator.
     * 
     * This function sets the frequency of the oscillator.
     * 
     * @param freq The frequency value in Hz (may be 1 Hz to 40 MHz).
     */
    void setOscFrequence (uint32_t freq);

    void oscillatorTestLoop();


    double getFrequencyFactor() {
        return frequency_factor;
    }
    
private:
    pcnt_config_t pcnt_config = { };                               // PCNT unit instance

    esp_timer_create_args_t create_args;                         // Create an esp_timer instance
    esp_timer_handle_t timer_handle;                            // Create an single timer

    portMUX_TYPE timer_mux         ;                           // portMUX_TYPE to do synchronism

    ledc_timer_config_t ledc_timer = {};                    // LEDC timer config instance
    ledc_channel_config_t ledc_channel = {};               // LEDC Channel config instance

    reel                frequency;                                      // frequency value
    reel                frequency_factor     ;                         // frequency factor = (1e6 / sample_time) / 2.0

    gpio_num_t          gpio_pcnt_input;                        // Pulse Counter input GPIO pin - Frequence input
    gpio_num_t          gpio_pcnt_ctrl;                        // Pulse Counter control GPIO pin - To enable/disable counting, input
    gpio_num_t          gpio_output_control;                  // Output control GPIO pin - To enable/disable counting, output connected to gpio_pcnt_ctrl
    gpio_num_t          gpio_ledc_output;                    // LEDC output GPIO pin - Oscillator output
    pcnt_unit_t         pcnt_unit;                            // PCNT unit instance
    pcnt_channel_t      pcnt_channel;                         // PCNT channel instance

    uint32_t            overflow      ;                                       // Max Pulse Counter value
    uint32_t            mult_pulses    ;                                     // Quantidade de overflows do contador PCNT
    uint32_t            sample_time   ;                                    // sample time in us to count pulses                                  
    uint32_t            m_duty         ;                                   // Duty value
    uint32_t            resolution    ;                                  // Resolution value
    int16_t             pulses        ;                                      // Pulse Counter value
    
    bool                flag          ;                                        // Flag to enable print frequency reading

    static FrequencyMeter* instance;  // Pointer to the current instance
    static void IRAM_ATTR pcntOverflowHandler(void *arg);
    static void readPCNT(void *p);
};



#endif // FREQUENCYMETER_H