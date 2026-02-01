#include <misc/pwr/pwr.hpp>

extern volatile uint16_t g;

namespace pwr {

	static constexpr uint16_t kFS = 4095;
	static constexpr float kVref = 3.3f;
	static constexpr float kRhigh = 10000.0f;
	static constexpr float kRlow = 1000.0f;
	static constexpr float offSet = 0.0405f;

	static volatile uint16_t v_sample = 0;

	void init() {
		HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
		HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&v_sample, 1);
	}

	uint16_t raw() { return v_sample; }

	float volts() {
		const float vin = (static_cast<float>(v_sample) / kFS) * kVref;
		const float scale = (kRhigh + kRlow) / kRlow;
		return vin * scale + offSet;
	}

}
