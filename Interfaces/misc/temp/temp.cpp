#include "temp.hpp"

namespace temp {

  static constexpr uint16_t kFS     = 4095;
  static constexpr float    kRfixed = 3300.0f;
  static constexpr float    kR0     = 10000.0f;
  static constexpr float    kBeta   = 3380.0f;
  static constexpr float    kT0K    = 25.0f + 273.15f;


  static volatile uint16_t t_sample = 0;

  void init() {

    HAL_ADCEx_Calibration_Start(&hadc3, ADC_SINGLE_ENDED);
    HAL_ADC_Start_DMA(&hadc3, (uint32_t*)&t_sample, 1);

  }

  uint16_t raw() { return t_sample; }


  static inline float code_to_rntc(uint16_t code) {
    const float c  = static_cast<float>(code);
    const float fs = static_cast<float>(kFS);

    if (c <= 0.5f)      return 99999.9f;
    if (c >= fs - 0.5f) return 0.0f;

    return kRfixed * ((fs - c) / c);
  }

  static inline float r_to_kelvin(float R) {
    const float invT = (1.0f / kT0K) + (1.0f / kBeta) * logf(R / kR0);
    return 1.0f / invT;
  }

  float kelvin() {
    const float Rntc = code_to_rntc(t_sample);
    return r_to_kelvin(Rntc);
  }

  float celcius() {
    return kelvin() - 273.15f;
  }

  float fahrenheit() {
    return celcius() * (9.0f/5.0f) + 32.0f;
  }
}
