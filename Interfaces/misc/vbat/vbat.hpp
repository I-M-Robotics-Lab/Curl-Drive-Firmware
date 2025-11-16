#pragma once
#include <adc.h>
#include <stm32g4xx_hal.h>

namespace vbat {

	void init();
	uint16_t raw();
	float volts();

}
