#include "led.hpp"

extern TIM_HandleTypeDef htim7;

namespace led {

void setLed(bool on) {
    HAL_GPIO_WritePin(LED_IND_GPIO_Port, LED_IND_Pin,
                      on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void startToggle() {

    __HAL_TIM_SET_COUNTER(&htim7, 0u);
    __HAL_TIM_CLEAR_FLAG(&htim7, TIM_FLAG_UPDATE);
    __HAL_TIM_ENABLE_IT(&htim7, TIM_IT_UPDATE);
    HAL_TIM_Base_Start_IT(&htim7);
}

void startToggle(float hz) {

	if (hz > 0.0f) {
		uint16_t arr = static_cast<uint16_t>((10000.0f / hz) - 1.0f);
		__HAL_TIM_SET_AUTORELOAD(&htim7, arr);
	}

    __HAL_TIM_SET_COUNTER(&htim7, 0u);
    __HAL_TIM_CLEAR_FLAG(&htim7, TIM_FLAG_UPDATE);
    __HAL_TIM_ENABLE_IT(&htim7, TIM_IT_UPDATE);
    HAL_TIM_Base_Start_IT(&htim7);
}

void stopToggle() {
    HAL_TIM_Base_Stop_IT(&htim7);
    __HAL_TIM_DISABLE_IT(&htim7, TIM_IT_UPDATE);
    __HAL_TIM_CLEAR_FLAG(&htim7, TIM_FLAG_UPDATE);
    HAL_GPIO_WritePin(LED_IND_GPIO_Port, LED_IND_Pin, GPIO_PIN_RESET);
}

}
