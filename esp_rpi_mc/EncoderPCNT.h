#ifndef ENCODER_PCNT_H
#define ENCODER_PCNT_H

#include <Arduino.h>
#include "driver/pcnt.h"
#include "soc/pcnt_struct.h"


#define H_LIM 5000
#define L_LIM -5000

class EncoderPCNT {
public:
  EncoderPCNT(gpio_num_t pinA, gpio_num_t pinB,pcnt_unit_t unit);
  ~EncoderPCNT();

  void begin();
  int32_t get_count();
  void resetEncoder();
  pcnt_unit_t _pcnt_unit;
  int32_t _totalCount;

private:
  gpio_num_t _pinA;
  gpio_num_t _pinB; 
};

#endif