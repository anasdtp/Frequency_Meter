#include "FrequencyMeter.h"

void set_gpio_if_needed(gpio_num_t gpio_num, uint32_t level) {
  if (gpio_get_level(gpio_num) != level) {
    gpio_set_level(gpio_num, level);
  }
}

/*0 : 2653.  1 : 1505.  2 : 1603.  3 : 1707.  4 : 1818.  5 : 1936.  6 : 2062.  7 : 2196.  8 : 2340.  
  9 : 2491.  A : 3214.  B : 1245.  C : 3009.  D : 1327.  E : 2825.  F : 0   */

//----------------------------------------------------------------------------

FrequencyMeter* FrequencyMeter::instance = nullptr;  // Initialize the static instance pointer

FrequencyMeter::FrequencyMeter(uint32_t sample_time_us, uint32_t count_overflow)
{
  instance = this;                                                            // Set the instance pointer to the current object
  this->flag = true;                                                         // Flag to enable print frequency reading
  this->overflow = count_overflow;                                          // Max Pulse Counter - initial : 20000
  this->pulses = 0;                                                        // Pulse Counter value
  this->mult_pulses = 0;                                                   // Quantidade de overflows do contador PCNT
  this->sample_time = sample_time_us;                                    // sample time in us to count pulses, can be changed - initial : 1000000
  this->m_duty = 0;                                                      // Duty value
  this->resolution = 0;                                                // Resolution value
  this->frequency = 0;                                                // frequency value
  this->frequency_factor = ((1./sample_time) * 1e6) / 2.0;                                    

  this->timer_mux = portMUX_INITIALIZER_UNLOCKED;                   // portMUX_TYPE to do synchronism

  this->ledc_timer.speed_mode = LEDC_HIGH_SPEED_MODE;            // Set high speed mode
  this->ledc_timer.timer_num = LEDC_TIMER_0; 

  this->ledc_channel.channel    = LEDC_CHANNEL_0;             // Set HS Channel - 0
  this->ledc_channel.gpio_num   = LEDC_HS_CH0_GPIO;          // LEDC Oscillator output GPIO 33
  this->ledc_channel.intr_type  = LEDC_INTR_DISABLE;        // LEDC Fade interrupt disable
  this->ledc_channel.speed_mode = LEDC_HIGH_SPEED_MODE;    // Set LEDC high speed mode
  this->ledc_channel.timer_sel  = LEDC_TIMER_0;           // Set timer source of channel - 0
  
}

FrequencyMeter::~FrequencyMeter()
{
  instance = nullptr;  // Set the instance pointer to null
}

void FrequencyMeter::changeSampleTime(uint32_t sample_time_us)
{
  this->sample_time = sample_time_us;  // Change the sample time
  this->frequency_factor = ((1./sample_time_us) * 1e6) / 2.0;  // Change the frequency factor
}

void FrequencyMeter::setOscFrequence (uint32_t freq)
{
  static uint32_t last_osc_freq = 0;
  static reel res_factor = 1./(2. * log(2.)), log_80000000 = log(80000000);  // Factor to calculate resolution

  if (freq == last_osc_freq) {return;}
  else if(freq == 0){//If freq is 0, stop the LEDC channel
    last_osc_freq = freq;  // Mise à jour de la dernière fréquence
    ledc_stop(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, 0);  // Stop the LEDC channel
    return;
  }

  this->resolution = (log_80000000 - log(freq)) * res_factor;  // Calculer uniquement si freq change
  if (this->resolution < 1){ this->resolution = 1;}
  this->m_duty = (1 << this->resolution) / 2;  // Utilise un décalage pour éviter `pow`, équivalent à `pow(2, resolution)`

  last_osc_freq = freq;  // Mise à jour de la dernière fréquence
  
  this->ledc_timer.duty_resolution =  ledc_timer_bit_t(this->resolution);             // Set resolution
  this->ledc_timer.freq_hz    = freq;                                       // Set Oscillator frequency
  ledc_timer_config(&this->ledc_timer);                                         // Set LEDC Timer config

  this->ledc_channel.duty       = this->m_duty;                                        // Set Duty Cycle 50%
  ledc_channel_config(&this->ledc_channel);                                     // Config LEDC channel
}

void FrequencyMeter::initFrequencyMeter()
{
  this->setOscFrequence(2653);  // Initialize oscillator frequency
  this->initPCNT();      // Initialize PCNT unit

  gpio_pad_select_gpio(OUTPUT_CONTROL_GPIO);                 // Set GPIO pad
  gpio_set_direction(OUTPUT_CONTROL_GPIO, GPIO_MODE_OUTPUT); // Set GPIO 32 as output
  create_args.callback = FrequencyMeter::readPCNT;              // Set esp-timer argument
  esp_timer_create(&create_args, &timer_handle); // Create esp-timer instance

  // gpio_set_direction(IN_BOARD_LED, GPIO_MODE_OUTPUT); // Set LED inboard as output

  gpio_matrix_in(PCNT_INPUT_SIG_IO, SIG_IN_FUNC226_IDX, false);    // Set GPIO matrin IN - Freq Meter input
  // gpio_matrix_out(IN_BOARD_LED, SIG_IN_FUNC226_IDX, false, false); // Set GPIO matrix OUT - to inboard LED
}

void FrequencyMeter::initPCNT()
{
  pcnt_config_t pcnt_config = { };                                        // PCNT unit instance

  pcnt_config.pulse_gpio_num = PCNT_INPUT_SIG_IO;                         // Pulse input GPIO 34 - Freq Meter Input
  pcnt_config.ctrl_gpio_num = PCNT_INPUT_CTRL_IO;                         // Control signal input GPIO 35
  pcnt_config.unit = PCNT_COUNT_UNIT;                                     // Unidade de contagem PCNT - 0
  pcnt_config.channel = PCNT_COUNT_CHANNEL;                               // PCNT unit number - 0
  pcnt_config.counter_h_lim = this->overflow;                             // Maximum counter value - 20000
  pcnt_config.pos_mode = PCNT_COUNT_INC;                                  // PCNT positive edge count mode - inc
  pcnt_config.neg_mode = PCNT_COUNT_INC;                                  // PCNT negative edge count mode - inc
  pcnt_config.lctrl_mode = PCNT_MODE_DISABLE;                             // PCNT low control mode - disable
  pcnt_config.hctrl_mode = PCNT_MODE_KEEP;                                // PCNT high control mode - won't change counter mode
  pcnt_unit_config(&pcnt_config);                                         // Initialize PCNT unit

  pcnt_counter_pause(PCNT_COUNT_UNIT);                                    // Pause PCNT unit
  pcnt_counter_clear(PCNT_COUNT_UNIT);                                    // Clear PCNT unit

  pcnt_event_enable(PCNT_COUNT_UNIT, PCNT_EVT_H_LIM);                     // Enable event to watch - max count
  pcnt_isr_register(FrequencyMeter::pcntOverflowHandler, NULL, 0, NULL);    // Setup Register ISR handler
  pcnt_intr_enable(PCNT_COUNT_UNIT);                                      // Enable interrupts for PCNT unit

  pcnt_counter_resume(PCNT_COUNT_UNIT);                                   // Resume PCNT unit - starts count
}


void FrequencyMeter::readPCNT(void *p)
{
  if(instance != nullptr) {
    set_gpio_if_needed(OUTPUT_CONTROL_GPIO, 0);                                        // Stop counter - output control LOW
    pcnt_get_counter_value(PCNT_COUNT_UNIT, &instance->pulses);                       // Read Pulse Counter value
    instance->flag = true;                                                           // Change flag to enable print
  }
}

void IRAM_ATTR FrequencyMeter::pcntOverflowHandler(void *arg)
{
  if(instance != nullptr) {
    portENTER_CRITICAL_ISR(&instance->timer_mux);                                      // disabling the interrupts
    instance->mult_pulses++;                                                          // increment Overflow counter
    PCNT.int_clr.val = BIT(PCNT_COUNT_UNIT);                                        // Clear Pulse Counter interrupt bit
    portEXIT_CRITICAL_ISR(&instance->timer_mux);                                    // enabling the interrupts
  }
}

reel FrequencyMeter::getFrequency(){
  if(!this->flag){return 0.;}
  this->flag = false;                                                       // change flag to disable print

  pcnt_counter_clear(PCNT_COUNT_UNIT);                                // Clear Pulse Counter
  esp_timer_start_once(timer_handle, sample_time);                    // Initialize High resolution timer 
  set_gpio_if_needed(OUTPUT_CONTROL_GPIO, 1);                         // Set enable PCNT count

  // frequency = ((pulses + (mult_pulses * overflow)) / 2.0)* 1e6/sample_time  ; // Calculation of frequency
  // Calcul optimisé sans opération flottante intermédiaire :
  this->frequency = (reel)(this->pulses + (this->mult_pulses * this->overflow)) * this->frequency_factor;
  // printf("Frequency : %f Hz \n", frequency);               // Print frequency 

  this->mult_pulses = 0;                                                     // Clear overflow counter

  return this->frequency;
}

void FrequencyMeter::oscillatorTestLoop(){
  static String inputString = "";                                               // clear temporary string
  static uint32_t osc_freq = 0;                                                          // Clear oscillator frequency

  while (Serial.available())
  {
    char inChar = (char)Serial.read();                                   // Reads a byte on the console
    Serial.print(inChar);                                                // Print the byte
    inputString += inChar;                                               // Add char to string
    if (inChar == '\n')                                                  // If new line (enter)
    {
      osc_freq = inputString.toInt();                                    // Converts String into integer value
      inputString = "";                                                  // Clear string
    }
                                                    
  }
  if (osc_freq != 0)                                                     // If some value inputted to oscillator frequency
  {
    this->setOscFrequence(osc_freq);                                                    // reconfigure ledc function - oscillator 
    Serial.printf("Setting frequence to %d", osc_freq);
    osc_freq = 0;
  }
}
