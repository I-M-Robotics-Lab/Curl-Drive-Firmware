#include "trig.hpp"
#include <cstdint>
#include "stm32g4xx_hal.h"

namespace trig {

SC get_sin_cos(uint16_t theta14) noexcept {

	CORDIC->WDATA = static_cast<int32_t>(theta14) << 18;

	return {
		static_cast<float>(static_cast<int32_t>(CORDIC->RDATA)) * 4.65661287e-10f,
	    static_cast<float>(static_cast<int32_t>(CORDIC->RDATA)) * 4.65661287e-10f
	};
}

}

