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

FrequencyMeter::FrequencyMeter(uint32_t sample_time_us, uint32_t count_overflow, gpio_num_t GPIO_FREQUENCE_INPUT, gpio_num_t GPIO_COUNT_CONTROL_INPUT, gpio_num_t GPIO_COUNT_CONTROL_OUTPUT, pcnt_unit_t PCNT_UNIT, pcnt_channel_t PCNT_CHANNEL)
{
  instance = this;                                                            // Set the instance pointer to the current object

  this->gpio_pcnt_input = GPIO_FREQUENCE_INPUT;                          // Pulse Counter input GPIO pin - Frequence input
  this->gpio_output_control = GPIO_COUNT_CONTROL_OUTPUT;                // Output control GPIO pin - To enable/disable counting, output connected to gpio_pcnt_ctrl
  this->gpio_pcnt_ctrl = GPIO_COUNT_CONTROL_INPUT;                     // Control signal input GPIO pin - To enable/disable counting, input
  this->gpio_ledc_output = GPIO_NUM_NC;                               // LEDC output GPIO pin - Oscillator output
  this->pcnt_unit = PCNT_UNIT;                                       // PCNT unit instance
  this->pcnt_channel = PCNT_CHANNEL;                                // PCNT channel instance

  
  this->flag = true;                                                         // Flag to enable print frequency reading
  this->overflow = count_overflow;                                          // Max Pulse Counter - initial : 20000
  this->pulses = 0;                                                        // Pulse Counter value
  this->mult_pulses = 0;                                                   // Quantidade de overflows do contador PCNT
  this->sample_time = sample_time_us;                                    // sample time in us to count pulses, can be changed - initial : 1000000
  this->m_duty = 0;                                                      // Duty value
  this->resolution = 0;                                                // Resolution value
  this->frequency = 0;                                                // frequency value
  this->frequency_factor = ((1./sample_time) * 1e6) / 2.0;           // divided by 2 if the counter increase on falling edge                         

  this->timer_mux = portMUX_INITIALIZER_UNLOCKED;                   // portMUX_TYPE to do synchronism
}

FrequencyMeter::~FrequencyMeter()
{
  instance = nullptr;  // Set the instance pointer to null
}


void FrequencyMeter::initPCNT(gpio_num_t gpio_input, gpio_num_t gpio_ctrl, pcnt_unit_t unit, pcnt_channel_t channel, int16_t overflow_limit, pcnt_count_mode_t pos_edge_mode, pcnt_count_mode_t neg_edge_mode)
{
  pcnt_config.pulse_gpio_num = gpio_input;   // GPIO for the frequency input signal
  pcnt_config.ctrl_gpio_num = gpio_ctrl;     // GPIO for the control signal
  pcnt_config.unit = unit;                    // The PCNT unit to use
  pcnt_config.channel = channel;              // The PCNT channel to use

  // Configure the counter limits and counting modes
  pcnt_config.counter_h_lim = overflow_limit; // Set the overflow limit for the counter
  pcnt_config.pos_mode = pos_edge_mode;       // Counting mode for rising edges
  pcnt_config.neg_mode = neg_edge_mode;       // Counting mode for falling edges
  pcnt_config.lctrl_mode = PCNT_MODE_DISABLE; // Control signal low mode (disabled)
  pcnt_config.hctrl_mode = PCNT_MODE_KEEP;    // Control signal high mode (kept as is)

  // Initialize the PCNT unit with the provided configuration
  pcnt_unit_config(&pcnt_config);

  // Pause, efface et reprend l'unité pour une initialisation propre
  pcnt_counter_pause(unit);                                          // Met en pause le compteur
  pcnt_counter_clear(unit);                                          // Réinitialise le compteur
  pcnt_event_enable(unit, PCNT_EVT_H_LIM);                           // Active l'événement de limite haute
  pcnt_isr_register(FrequencyMeter::pcntOverflowHandler, NULL, 0, NULL);  // Enregistre l'ISR pour gérer le débordement
  pcnt_intr_enable(unit);                                            // Active les interruptions pour l'unité PCNT
  pcnt_counter_resume(unit);                                         // Reprend le comptage des impulsions


  this->frequency_factor = ((1./this->sample_time) * 1e6);// Change the frequency factor
  if((pcnt_config.pos_mode == PCNT_COUNT_INC) && (pcnt_config.neg_mode == PCNT_COUNT_INC))// If count on both edges
  {  
    this->frequency_factor /= 2.0; //Then divide by 2.0 to get the correct frequency
  }
}

void FrequencyMeter::initFrequencyMeter(bool count_on_falling_edges)
{
  pcnt_count_mode_t pos_edge_mode = PCNT_COUNT_INC; 
  pcnt_count_mode_t neg_edge_mode = PCNT_COUNT_INC;
  if(!count_on_falling_edges) { // If count on falling edges
    neg_edge_mode = PCNT_COUNT_DIS; // Disable counting on falling edges
  }
  this->initPCNT(this->gpio_pcnt_input, this->gpio_pcnt_ctrl, this->pcnt_unit, this->pcnt_channel, this->overflow, pos_edge_mode, neg_edge_mode);

  gpio_pad_select_gpio(this->gpio_output_control);                 // Set GPIO pad
  gpio_set_direction(this->gpio_output_control, GPIO_MODE_OUTPUT); // Set GPIO 32 as output
  create_args.callback = FrequencyMeter::readPCNT;              // Set esp-timer argument
  esp_timer_create(&create_args, &timer_handle); // Create esp-timer instance

  // gpio_set_direction(IN_BOARD_LED, GPIO_MODE_OUTPUT); // Set LED inboard as output

  gpio_matrix_in(this->gpio_pcnt_input, SIG_IN_FUNC226_IDX, false);    // Set GPIO matrin IN - Freq Meter input
  // gpio_matrix_out(IN_BOARD_LED, SIG_IN_FUNC226_IDX, false, false); // Set GPIO matrix OUT - to inboard LED
}

void FrequencyMeter::readPCNT(void *p)
{
  if(instance != nullptr) {
    set_gpio_if_needed(instance->gpio_output_control, 0);                                        // Stop counter - output control LOW
    pcnt_get_counter_value(instance->pcnt_unit, &instance->pulses);                       // Read Pulse Counter value
    instance->flag = true;                                                           // Change flag to enable print
  }
}

void IRAM_ATTR FrequencyMeter::pcntOverflowHandler(void *arg)
{
  if(instance != nullptr) {
    portENTER_CRITICAL_ISR(&instance->timer_mux);                                      // disabling the interrupts
    instance->mult_pulses++;                                                          // increment Overflow counter
    PCNT.int_clr.val = BIT(instance->pcnt_unit);                                        // Clear Pulse Counter interrupt bit
    portEXIT_CRITICAL_ISR(&instance->timer_mux);                                    // enabling the interrupts
  }
}

reel FrequencyMeter::getFrequency(){
  if(!this->flag){return 0.;}
  this->flag = false;                                                       // change flag to disable print

  pcnt_counter_clear(instance->pcnt_unit);                                // Clear Pulse Counter
  esp_timer_start_once(timer_handle, sample_time);                    // Initialize High resolution timer 
  set_gpio_if_needed(instance->gpio_output_control, 1);                         // Set enable PCNT count

  // frequency = ((pulses + (mult_pulses * overflow)) / 2.0)* 1e6/sample_time  ; // Calculation of frequency
  // Calcul optimisé sans opération flottante intermédiaire :
  this->frequency = (reel)(this->pulses + (this->mult_pulses * this->overflow)) * this->frequency_factor;
  // printf("Frequency : %f Hz \n", frequency);               // Print frequency 

  this->mult_pulses = 0;                                                     // Clear overflow counter

  return this->frequency;
}

void FrequencyMeter::changeSampleTime(uint32_t sample_time_us)
{
  this->sample_time = sample_time_us;  // Change the sample time
  this->frequency_factor = ((1./this->sample_time) * 1e6);// Change the frequency factor

  if((pcnt_config.pos_mode == PCNT_COUNT_INC) && (pcnt_config.neg_mode == PCNT_COUNT_INC))// If count on both edges
  {  
    this->frequency_factor /= 2.0; //Then divide by 2.0 to get the correct frequency
  }
  // else if((pcnt_config.pos_mode == PCNT_COUNT_INC) && (pcnt_config.neg_mode == PCNT_COUNT_DIS))
  // {
  //   this->frequency_factor = this->frequency_factor;
  // }

  //If you want to use the mode PCNT_COUNT_DEC, this calculation need to be changed.
}

void FrequencyMeter::initOscillator(gpio_num_t output_pin, ledc_timer_t timer_num, ledc_channel_t channel)
{
  this->gpio_ledc_output = output_pin;                     // Set the GPIO pin for the oscillator output
  if(this->gpio_ledc_output){
    // Configure the LEDC timer
    this->ledc_timer.speed_mode = LEDC_HIGH_SPEED_MODE; // Set speed mode (high or low speed)
    this->ledc_timer.timer_num = timer_num;  // Set the timer to use

    // Configure the LEDC channel
    this->ledc_channel.channel = channel;       // Set the channel to use
    this->ledc_channel.gpio_num = this->gpio_ledc_output;   // Set the GPIO pin for the output signal
    this->ledc_channel.intr_type = LEDC_INTR_DISABLE;   // Set the interrupt type (disabled by default)
    this->ledc_channel.speed_mode = LEDC_HIGH_SPEED_MODE; // Set the speed mode for the channel
    this->ledc_channel.timer_sel = timer_num;   // Link the channel to the specified timer
  }
}

void FrequencyMeter::setOscFrequence (uint32_t freq)
{
  static uint32_t last_osc_freq = 0;
  static reel res_factor = 1./(2. * log(2.)), log_80000000 = log(80000000);  // Factor to calculate resolution

  if ((freq == last_osc_freq) || (this->gpio_ledc_output == GPIO_NUM_NC)) {return;}
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
