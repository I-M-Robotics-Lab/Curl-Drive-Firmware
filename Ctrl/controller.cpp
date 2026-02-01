// controller.cpp
#include "controller.hpp"
#include "common_inc.hpp"
#include "stm32g4xx_hal_flash.h"
#include "stm32g4xx_hal_flash_ex.h"
#include <cstring>

void Controller::configure(const Config& c) { cfg_ = c; }

bool Controller::config_write() {
    cfg_.valid_SOF = 0xA55A5AA5u;
    cfg_.valid_EOF = 0x5AA5A55Au;

    FLASH_EraseInitTypeDef erase{};
    erase.TypeErase = FLASH_TYPEERASE_PAGES;
    erase.Banks     = FLASH_BANK_2;
    erase.Page      = 159u;
    erase.NbPages   = 1;

    uint32_t page_error = 0;
    HAL_StatusTypeDef st;

    HAL_FLASH_Unlock();
    __disable_irq();
    st = HAL_FLASHEx_Erase(&erase, &page_error);
    if (st != HAL_OK) { __enable_irq(); HAL_FLASH_Lock(); return false; }

    const uint8_t* src = reinterpret_cast<const uint8_t*>(&cfg_);
    const size_t   len = sizeof(cfg_);
    uintptr_t      dst = 0x0804F800u;

    for (size_t i = 0; i < len; i += 8, dst += 8) {
        uint64_t word = 0xFFFFFFFFFFFFFFFFull;
        const size_t rem = (i + 8 <= len) ? 8 : (len - i);
        std::memcpy(&word, src + i, rem);
        st = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, dst, word);
        if (st != HAL_OK) { __enable_irq(); HAL_FLASH_Lock(); return false; }
        if (*reinterpret_cast<const uint64_t*>(dst) != word) { __enable_irq(); HAL_FLASH_Lock(); return false; }
    }

    __enable_irq();
    HAL_FLASH_Lock();
    return true;
}

bool Controller::config_read() {
    const Config* p = reinterpret_cast<const Config*>(0x0804F800u);
    if (p->valid_SOF != 0xA55A5AA5u) return false;
    if (p->valid_EOF != 0x5AA5A55Au) return false;
    std::memcpy(&cfg_, p, sizeof(Config));
    return true;
}

void Controller::set_current_loop_freq(uint32_t hz) {
    const uint32_t tim_clk = 144000000u;
    const uint32_t den     = (hz > 0u) ? hz : 1u;
    uint32_t arr_u32       = tim_clk / (2u * den);
    if (arr_u32 < 2u) arr_u32 = 2u;
    const uint16_t arr     = static_cast<uint16_t>(arr_u32 - 1u);
    const uint16_t ccr4    = static_cast<uint16_t>((arr + 1u) / 2u);

    __HAL_TIM_DISABLE(&htim1);
    __HAL_TIM_SET_AUTORELOAD(&htim1, arr);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, ccr4);
    __HAL_TIM_ENABLE(&htim1);
}

void Controller::disarm() {
    if (drv.setCoast()) {
        foc::status.TarId  = 0.0f;
        foc::status.TarIq  = 0.0f;
        foc::status.phaseA = 0.0f;
        foc::status.phaseB = 0.0f;
        foc::status.phaseC = 0.0f;
        status_.armed = false;
    }
}

void Controller::arm() {
    foc::status.TarId  = 0.0f;
    foc::status.TarIq  = 0.0f;
    foc::status.phaseA = 0.0f;
    foc::status.phaseB = 0.0f;
    foc::status.phaseC = 0.0f;

    Driver8323s::Status s{};
    if (drv.readStatus(s)) {
        const bool all_clear = ((static_cast<uint16_t>(s.raw_status1) | static_cast<uint16_t>(s.raw_status2)) == 0u);
        if (all_clear && drv.setRun()) { status_.armed = true; return; }
    }
    status_.armed = true;
}

void Controller::calibrate() {

}

void Controller::init() {
	(void)config_read();

	set_current_loop_freq(cfg_.current_loop_freq);
    __HAL_TIM_MOE_DISABLE(&htim1);

    HAL_NVIC_DisableIRQ(ADC1_2_IRQn);
    HAL_NVIC_DisableIRQ(TIM1_UP_TIM16_IRQn);
    __HAL_ADC_DISABLE_IT(&hadc2, ADC_IT_JEOS | ADC_IT_JEOC);
    __HAL_TIM_DISABLE_IT(&htim1, TIM_IT_UPDATE);
    __HAL_ADC_CLEAR_FLAG(&hadc2, ADC_FLAG_JEOC | ADC_FLAG_JEOS | ADC_FLAG_OVR | ADC_FLAG_AWD1);

    drv.clearFaults();
    HAL_Delay(1);
    drv.calBegin();

    HAL_TIM_OC_Start(&htim1, TIM_CHANNEL_4);
    HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);
    HAL_Delay(1);
    HAL_ADCEx_InjectedStart(&hadc2);
    __HAL_TIM_ENABLE(&htim1);

    {
        constexpr uint16_t N = 256;
        uint32_t accA = 0, accB = 0, accC = 0;
        uint16_t samples = 0;

        for (uint16_t i = 0; i < N; ++i) {
            while (__HAL_ADC_GET_FLAG(&hadc2, ADC_FLAG_JEOS) == RESET)
            __HAL_ADC_CLEAR_FLAG(&hadc2, ADC_FLAG_JEOS);
            const uint16_t r1 = HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1);
            const uint16_t r2 = HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_2);
            const uint16_t r3 = HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_3);
            accA += r1; accB += r2; accC += r3;
            ++samples;
        }

        if (samples == 0) {
            cfg_.offset_a = 2048u; cfg_.offset_b = 2048u; cfg_.offset_c = 2048u;
        } else {
        	usb::println(accA, " ", accB, " ", accC);
            cfg_.offset_a = static_cast<uint16_t>(accA / samples);
            cfg_.offset_b = static_cast<uint16_t>(accB / samples);
            cfg_.offset_c = static_cast<uint16_t>(accC / samples);
        }
    }

    drv.calEnd();
    HAL_Delay(1);
    __HAL_ADC_CLEAR_FLAG(&hadc2, ADC_FLAG_JEOC | ADC_FLAG_JEOS | ADC_FLAG_OVR | ADC_FLAG_AWD1);
    __HAL_ADC_ENABLE_IT(&hadc2, ADC_IT_JEOS);
    __HAL_TIM_CLEAR_IT(&htim1, TIM_IT_UPDATE);
    __HAL_TIM_ENABLE_IT(&htim1, TIM_IT_UPDATE);
    HAL_NVIC_EnableIRQ(ADC1_2_IRQn);
    HAL_NVIC_EnableIRQ(TIM1_UP_TIM16_IRQn);

    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

    __HAL_TIM_MOE_ENABLE(&htim1);
}







void Controller::execute() {
    if (!status_.armed) return;

    if (status_.isCalibrated) {
        switch (status_.mode) {
            case Mode::Run:
                break;
            case Mode::Calibrating:
                break;
        }
    } else {
        switch (status_.mode) {
            case Mode::Run:
                break;
            case Mode::Calibrating:
                break;
        }
    }
}









void Controller::write_pwm() {
    const uint16_t arr  = __HAL_TIM_GET_AUTORELOAD(&htim1);
    float vbat = vbat::volts();
    foc::status.vbat = vbat;
    const float v_max = std::min((vbat > 0.0f) ? vbat : 1.0f, controller.config().max_voltage);
    const float mid     = 0.5f * arr;
    const float span    = float(arr) - 2.0f;
    const float inv_vmax = 1.0f / v_max;

    auto to_ccr = [&](float v)->uint16_t {
    	float u = v * inv_vmax;
        if (u >  1.0f) u =  1.0f;
        if (u < -1.0f) u = -1.0f;
        float c = mid + 0.5f * span * u;
        if (c < 1.0f)         c = 1.0f;
        if (c > float(arr-1)) c = float(arr-1);
        return uint16_t(c);
    };

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, to_ccr(foc::status.phaseA));
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, to_ccr(foc::status.phaseB));
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, to_ccr(foc::status.phaseC));
}

uint16_t Controller::mech_to_elec(uint16_t mech_theta14) const {
    uint16_t elec = static_cast<uint16_t>(mech_theta14 & 0x3FFF);
    elec = static_cast<uint16_t>((static_cast<uint32_t>(elec) * static_cast<uint32_t>(cfg_.pole_pairs)) & 0x3FFF);
    if (cfg_.elec_s < 0) elec = static_cast<uint16_t>((0u - elec) & 0x3FFF);
    elec = static_cast<uint16_t>((elec + (cfg_.elec_offset & 0x3FFF)) & 0x3FFF);
    return elec;
}


void Controller::process_encoder() {

    (void)encoder.updateAngle();

    const auto& ad = encoder.angleData();
    const uint16_t elec = mech_to_elec(ad.rectifiedAngle);
    status_.mechAng = ad.rectifiedAngle;
    status_.elecAng = ad.rectifiedAngle;

    foc::status.prevTheta = foc::status.currTheta;
    foc::status.prevT     = foc::status.currT;
    uint32_t currT = (uint32_t)ad.tstamp;

    if (status_.isCalibrated) {

    	uint32_t dt = currT - (uint32_t)foc::status.prevT;
    	int16_t dTheta = (int16_t)((((int)elec - (int)foc::status.prevTheta + 8192) & 0x3FFF) - 8192);
    	int aheadTheta = 0;

    	if (dt != 0u && dTheta != -8192) {
    		aheadTheta = ( dTheta * ( (8000000 / (int)cfg_.current_loop_freq) - 80 ) ) / (int)dt;
    	}

        foc::status.currTheta = static_cast<uint16_t>(elec & 0x3FFF);
        foc::status.currT     = currT;

    } else {
        foc::status.currTheta = static_cast<uint16_t>((uint16_t)(foc::status.currTheta + (uint16_t)status_.dtheta_per_isr) & 0x3FFFu);
        foc::status.currT = foc::status.prevT + status_.ticks_per_isr;
    }
}


extern "C" void TIM1_UP_TIM16_IRQHandler(void) {
    if (__HAL_TIM_GET_FLAG(&htim1, TIM_FLAG_UPDATE) != RESET) {
        if (__HAL_TIM_GET_IT_SOURCE(&htim1, TIM_IT_UPDATE) != RESET) {
            __HAL_TIM_CLEAR_IT(&htim1, TIM_IT_UPDATE);

            controller.process_encoder();


/*
            foc::status.prevTheta = foc::status.currTheta;
            foc::status.prevT     = foc::status.currT;

            constexpr uint16_t MASK  = 0x3FFFu;    // 14-bit wrap
            constexpr uint16_t DELTA = 16u;        // counts per ISR (~64/16384 elec rev per tick)
            constexpr uint32_t TICKS_PER_ISR = 200u; // 50 µs at 125 ns/tick (20 kHz loop)

            foc::status.currTheta = static_cast<uint16_t>((static_cast<uint16_t>(foc::status.currTheta) + DELTA) & MASK);
            foc::status.currT    += TICKS_PER_ISR;
*/
        }
    }
    HAL_TIM_IRQHandler(&htim1);
}

extern "C" void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef* hadc) {
    if (hadc != &hadc2) return;

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);

    const uint16_t r1 = HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1);
    const uint16_t r2 = HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_2);
    const uint16_t r3 = HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_3);

    constexpr float ADC_FS_INV = 1.0f / 4095.0f;
    constexpr float VREF       = 3.3f;
    const float vscale = VREF * ADC_FS_INV;

    const int32_t c1 = int32_t(r1) - int32_t(controller.config().offset_a);
    const int32_t c2 = int32_t(r2) - int32_t(controller.config().offset_b);
    const int32_t c3 = int32_t(r3) - int32_t(controller.config().offset_c);

    const float va = float(c1) * vscale;
    const float vb = float(c2) * vscale;
    const float vc = float(c3) * vscale;

    const float denom = controller.config().adc_gain * controller.config().shunt_res;
    if (denom <= 0.0f) return;

    foc::status.cA = va / denom;
    foc::status.cB = vb / denom;
    foc::status.cC = vc / denom;

    ///
    //foc part
    ///
    foc::InDqTransform();
    foc::DqTransform();

    controller.write_pwm();
}
