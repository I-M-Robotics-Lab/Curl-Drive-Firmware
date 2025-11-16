#pragma once
#include <adc.h>
#include <stm32g4xx_hal.h>
#include <math.h>

namespace temp {

  void     init();
  uint16_t raw();

  float    celcius();
  float    fahrenheit();
  float    kelvin();

}
