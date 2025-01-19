#ifndef MY_SIGNAL_H
#define MY_SIGNAL_H
#include <Arduino.h>
#include <FrequencyMeter.h>

#ifndef SIZE_FIFO
    #define SIZE_FIFO 64
#endif

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

// A frame consists of 6 frequencies
// Each frequency lasts 35 ms
// A frame therefore lasts 210 ms

#define TIME_OF_A_TONE 35000lu // in us

#define TONE_0 2653
#define TONE_1 1505
#define TONE_2 1603
#define TONE_3 1707
#define TONE_4 1818
#define TONE_5 1936
#define TONE_6 2062
#define TONE_7 2196
#define TONE_8 2340
#define TONE_9 2491
#define TONE_A 3214
#define TONE_B 1245
#define TONE_C 3009
#define TONE_D 1327
#define TONE_E 2825
#define TONE_F 0

typedef struct {
    uint32_t frequency;
    unsigned long duration;
} EmissionList;

/**
 * @class MySignal
 * @brief A class that extends FrequencyMeter to handle emission functionalities according to the
 * @brief A class that extends FrequencyMeter to handle emission functionalities according to the module X voies of Assystem.
 *
 * @param sample_time_us The sample time in microseconds for frequency measurement. Default is 35000 microseconds.
 *
 */
class MySignal : public FrequencyMeter
{
public:
    MySignal(gpio_num_t GPIO_FREQUENCE_INPUT = GPIO_NUM_34, 
             gpio_num_t GPIO_COUNT_CONTROL_INPUT = GPIO_NUM_35, gpio_num_t GPIO_COUNT_CONTROL_OUTPUT = GPIO_NUM_32, 
             pcnt_unit_t PCNT_UNIT = PCNT_UNIT_0, pcnt_channel_t PCNT_CHANNEL = PCNT_CHANNEL_0);
    ~MySignal(){}

    /**
     * @brief Initializes the FrequencyMeter and the MySignal object.
     * 
     * This function initializes the FrequencyMeter and the MySignal object.
     * 
     * @param count_on_falling_edges If it's true, the pcnt counter will increase on falling edges. If not, disable counting on falling edges.
     * @param oscillator_output_pin The GPIO pin for the oscillator output. Default is GPIO_NUM_33.
     */
    void init(bool count_on_falling_edges = true, gpio_num_t oscillator_output_pin = GPIO_NUM_33);
    
    /**
     * @brief Get the Frequency value. Designed especially for the Module X Voies.
     * 
     * This function returns the frequency value as a floating-point number.
     * 
     * @return reel The frequency value.
     */
    reel Frequency();

    uint32_t DFC(uint8_t TON); //Digital to frequency converter ; TON sur 4 bits
    uint8_t FDC(uint32_t freq);//Frequency to digital converter ; TON sur 4 bits
    uint8_t FDC(reel freq);

    void frame(uint8_t adresse, uint8_t commande1, uint8_t commande0);
    void frame(uint8_t adresse, uint16_t commande);

    //Frequence en Hz et duree en us
    void addEmission(uint32_t frequence, unsigned long duree_us);
    void emissionLoop();


private:
    EmissionList emissionList[SIZE_FIFO];
    unsigned char EMISSION_write = 0;
};


#endif // MY_SIGNAL_H