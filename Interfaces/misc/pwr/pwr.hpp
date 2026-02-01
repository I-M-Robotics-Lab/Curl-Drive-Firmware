#pragma once
#include <adc.h>
#include <stm32g4xx_hal.h>

namespace pwr {

	void init();
	uint16_t raw();
	float volts();

}
