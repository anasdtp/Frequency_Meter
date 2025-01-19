#include "mySignal.h"

MySignal::MySignal(gpio_num_t GPIO_FREQUENCY_INPUT, gpio_num_t GPIO_COUNT_CONTROL_INPUT, gpio_num_t GPIO_COUNT_CONTROL_OUTPUT, pcnt_unit_t PCNT_UNIT, pcnt_channel_t PCNT_CHANNEL)
  : FrequencyMeter(0, 20000, GPIO_FREQUENCY_INPUT, GPIO_COUNT_CONTROL_INPUT, GPIO_COUNT_CONTROL_OUTPUT, PCNT_UNIT, PCNT_CHANNEL) {
}

void MySignal::init(bool count_on_falling_edges, gpio_num_t oscillator_output_pin){
  this->initFrequencyMeter(count_on_falling_edges);
  this->initOscillator(oscillator_output_pin);

  this->setOscFrequence(0);
  this->getFlag(false);
  this->setSampleTime(31250);
}

reel MySignal::Frequency(){
  static reel frequency = 0.f;
  static unsigned long start_time, wait_time;
  static uint8_t state = 0;
  static bool old_flag = false;

  switch (state)
  {
  case 0:{
        if(old_flag != digitalRead(this->gpio_pcnt_input)){
          this->startCounting(this->pcnt_unit, this->timer_handle, this->getSampleTime(), this->gpio_output_control);
          state = 1;
        }
  }break;
  case 1:{
        if(this->getFlag(false)){
          start_time = micros();
          wait_time = (TIME_OF_A_TONE) - (unsigned long)(this->getSampleTime()); // One measurement per frequency (one freq lasts 35ms) in the 210ms frame
          // Optimized calculation without intermediate floating-point operation:
          frequency = (reel)(this->pulses + (this->mult_pulses * this->overflow)) * this->getFrequencyFactor();
          this->mult_pulses = 0; // Clear overflow counter
          
          state = 2;
        }
  }break;
  case 2:{
        if ((micros() - start_time) > wait_time) {
          state = 0;
          old_flag = digitalRead(this->gpio_pcnt_input);
        }
  }break;
  default:
    break;
  }
  return frequency;
}

/*Digital to frequency converter; TONE on 4 bits*/
uint32_t MySignal::DFC(uint8_t TONE){
  switch (TONE)
  {
  case 0:
    return TONE_0;
    break;

  case 1:
    return TONE_1;
    break;

  case 2:
    return TONE_2;
    break;

  case 3:
    return TONE_3;
    break;

  case 4:
    return TONE_4;
    break;

  case 5:
    return TONE_5;
    break;

  case 6:
    return TONE_6;
    break;

  case 7:
    return TONE_7;
    break;

  case 8:
    return TONE_8;
    break;
    
  case 9:
    return TONE_9;
    break;

  case 0x0A:
    return TONE_A;
    break;

  case 0x0B:
    return TONE_B;
    break;

  case 0x0C:
    return TONE_C;
    break;

  case 0x0D:
    return TONE_D;
    break;

  case 0x0E:
    return TONE_E;
    break;

  case 0x0F:
    return TONE_F;
    break;
  
  default:
    break;
  }

  return TONE_F;
}

/*Frequency to digital converter*/
uint8_t MySignal::FDC(uint32_t freq) {
  static const uint32_t frequencies[16] = {TONE_0, TONE_1, TONE_2, TONE_3, TONE_4, TONE_5, TONE_6, TONE_7, TONE_8, TONE_9, TONE_A, TONE_B, TONE_C, TONE_D, TONE_E, TONE_F};
  static const uint32_t marginOfError = 16; // Frequency margin of error in Hz
  uint8_t result = 0xF; // Default value in case of unrecognized frequency

  for (uint8_t i = 0; i < 16; i++) {
      if ((freq >= (frequencies[i] - marginOfError)) && (freq <= (frequencies[i] + marginOfError))) {
          result = i;
          break;
      }
  }
  return result;
}

uint8_t MySignal::FDC(reel freq) {
  static const reel frequencies[16] = {TONE_0, TONE_1, TONE_2, TONE_3, TONE_4, TONE_5, TONE_6, TONE_7, TONE_8, TONE_9, TONE_A, TONE_B, TONE_C, TONE_D, TONE_E, TONE_F};
  static const reel marginOfError = 16.0f; // Frequency margin of error in Hz
  uint8_t result = 0xF; // Default value in case of unrecognized frequency

  for (uint8_t i = 0; i < 16; i++) {
    if (abs(freq - frequencies[i]) <= marginOfError) {
        result = i;
        break;
    }
  }
  return result;
}

void MySignal::frame(uint8_t address, uint8_t command1, uint8_t command0){ // Example: 0x21 0x88 0x88
  uint8_t addr1  = (address  >> 4)&0x0F, addr0 = address    &0x0F,
          cmd3  = (command1>> 4)&0x0F, cmd2 = command1  &0x0F,
          cmd1  = (command0>> 4)&0x0F, cmd0 = command0  &0x0F;
  // No two similar successive frequencies:
  if(addr0 == addr1)  {addr0 = 0x0E;}
  if(cmd3 == addr0)  {cmd3 = 0x0E;}
  if(cmd2 == cmd3)  {cmd2 = 0x0E;}
  if(cmd1 == cmd2)  {cmd1 = 0x0E;}
  if(cmd0 == cmd1)  {cmd0 = 0x0E;} // Example: 0x21 0x8E 0x8E

  addEmission(DFC(addr1), TIME_OF_A_TONE); // Most significant addr
  addEmission(DFC(addr0), TIME_OF_A_TONE); // Least significant addr

  addEmission(DFC(cmd3), TIME_OF_A_TONE); // Most significant cmd
  addEmission(DFC(cmd2), TIME_OF_A_TONE); // Second most significant cmd
  addEmission(DFC(cmd1), TIME_OF_A_TONE); // Second least significant cmd
  addEmission(DFC(cmd0), TIME_OF_A_TONE); // Least significant cmd // Example: At tam: F218E8EF
  addEmission(0u, 2000lu);
}

void MySignal::frame(uint8_t address, uint16_t command){
  frame(address, ((command>>8)&0xFF), (command&0xFF));
}

void MySignal::addEmission(uint32_t frequency, unsigned long duration_us){
  this->emissionList[this->EMISSION_write].frequency = frequency;
  this->emissionList[this->EMISSION_write].duration = duration_us;
  this->EMISSION_write = (this->EMISSION_write + 1) % SIZE_FIFO;
}

void MySignal::emissionLoop(){
  static int state = 0;
  static signed char FIFO_read=0, FIFO_occupation=0, FIFO_max_occupation=0;
  static unsigned long start_time, duration;

  switch (state)
  {
  case 0:{
    FIFO_occupation=this->EMISSION_write-FIFO_read;
    if(FIFO_occupation<0){FIFO_occupation=FIFO_occupation+SIZE_FIFO;}
    if(FIFO_max_occupation<FIFO_occupation){FIFO_max_occupation=FIFO_occupation;}

    if(!FIFO_occupation){/*Then no frequency*/return;}

    this->setOscFrequence(this->emissionList[FIFO_read].frequency);
    start_time = micros();
    duration = this->emissionList[FIFO_read].duration;
    state = duration?1:0;
    
    FIFO_read=(FIFO_read+1)%SIZE_FIFO;
  }
    break;

  case 1:{
    if((micros() - start_time) > duration){
      state = 0;
    }
  }
    break;
  default:
    break;
  }
}
