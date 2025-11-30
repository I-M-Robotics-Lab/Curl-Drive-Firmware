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

void Controller::set_velocity_loop_freq(uint32_t hz) {
    const uint32_t tim_clk = 144000000u;
    const uint32_t den     = (hz > 0u) ? hz : 1u;
    uint32_t arr_u32       = (tim_clk / 144u) / den;
    if (arr_u32 < 2u) arr_u32 = 2u;
    const uint16_t arr     = static_cast<uint16_t>(arr_u32 - 1u);

    __HAL_TIM_DISABLE(&htim6);
    __HAL_TIM_SET_AUTORELOAD(&htim6, arr);
    __HAL_TIM_ENABLE(&htim6);
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
    status_.mode = Mode::Calibrating;

    usb::println("Calibration Start");

    foc::status.currTheta = 0;
    foc::status.prevTheta = 0;
    foc::status.currT = 0;
    foc::status.prevT = 0;

    cfg_.elec_offset = 0;

    foc::status.Vd = -3.14159f; //activate special phase control mode
    foc::status.Vq = -3.14159f;

    //Rough Offset Alignment
    foc::status.phaseA =  5.0f;
    foc::status.phaseB = -2.5f;
    foc::status.phaseC = -2.5f;
    HAL_Delay(500);

    uint32_t acc = 0;
    for (uint16_t i = 0; i < 1000; i++) {
    	acc += status_.curr_mechAng;
        HAL_Delay(1);
    }
    uint16_t mech_avg = static_cast<uint16_t>(acc / 1000u);
    uint16_t elec_measured = static_cast<uint16_t>((mech_avg * cfg_.pole_pairs * cfg_.elec_s) & 0x3FFFu);
    cfg_.elec_offset = static_cast<uint16_t>((0u - elec_measured) & 0x3FFFu);

    HAL_Delay(500);

    foc::status.Vd = 3.0f;
    foc::status.Vq = 0.0f;

    HAL_Delay(500);

    foc::status.Vd = -3.14159f; //activate special phase control mode
    foc::status.Vq = -3.14159f;

    foc::status.phaseA =  5.0f;
    foc::status.phaseB = -2.5f;
    foc::status.phaseC = -2.5f;
    HAL_Delay(500);

    //Fine Offset Alignment
    acc = 0;
    for (uint16_t i = 0; i < 1000; i++) {
        acc += status_.curr_mechAng;
        HAL_Delay(1);
    }
    mech_avg = static_cast<uint16_t>(acc / 1000u);
    uint16_t elec_fine = mech_to_elec(mech_avg);
    uint16_t fine_correction = static_cast<uint16_t>((0u - elec_fine) & 0x3FFFu);
    cfg_.elec_offset = static_cast<uint16_t>((cfg_.elec_offset + fine_correction) & 0x3FFFu);

    cfg_.is_calibrated = true;
    status_.isCalibrated = true;

    foc::status.Vd = 0.0f;
    foc::status.Vq = 0.0f;
    foc::status.phaseA = 0.0f;
    foc::status.phaseB = 0.0f;
    foc::status.phaseC = 0.0f;
    foc::status.currTheta = 0;
    foc::status.prevTheta = 0;
    foc::status.currT = 0;
    foc::status.prevT = 0;

    config_write();

    usb::println("Calibration End, Eoffset: ", cfg_.elec_offset);

    status_.mode = Mode::Idle;
}

void Controller::init() {
	bool config_valid = config_read();

    if (config_valid && cfg_.is_calibrated) {
        status_.isCalibrated = true;
    } else {
        status_.isCalibrated = false;
    }

	set_current_loop_freq(cfg_.current_loop_freq);
    __HAL_TIM_MOE_DISABLE(&htim1);

    HAL_NVIC_DisableIRQ(ADC1_2_IRQn);
    HAL_NVIC_DisableIRQ(TIM1_UP_TIM16_IRQn);
    __HAL_ADC_DISABLE_IT(&hadc2, ADC_IT_JEOS | ADC_IT_JEOC);
    __HAL_TIM_DISABLE_IT(&htim1, TIM_IT_UPDATE);
    __HAL_ADC_CLEAR_FLAG(&hadc2, ADC_FLAG_JEOC | ADC_FLAG_JEOS | ADC_FLAG_OVR | ADC_FLAG_AWD1);

    set_velocity_loop_freq(cfg_.vel_loop_freq);

    __HAL_TIM_SET_COUNTER(&htim6, 0u);
    __HAL_TIM_CLEAR_FLAG(&htim6, TIM_FLAG_UPDATE);
    __HAL_TIM_ENABLE_IT(&htim6, TIM_IT_UPDATE);
    HAL_TIM_Base_Start_IT(&htim6);

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















void Controller::write_pwm() {

    if (!status_.armed) {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
        return;
    }

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
    uint16_t mechAng = (uint16_t)ad.rectifiedAngle;
    uint32_t T = (uint32_t)ad.tstamp;
    uint16_t elec = mech_to_elec(mechAng);

    status_.prevT = status_.currT;
    status_.prev_mechAng = status_.curr_mechAng;

    foc::status.prevTheta = foc::status.currTheta;
    foc::status.prevT     = foc::status.currT;

    uint32_t dt = T - status_.prevT;
    int16_t dTheta_mech = (int16_t)((((int)mechAng - (int)status_.prev_mechAng + 8192) & 0x3FFF) - 8192);

    if (abs(dTheta_mech) > 12288) {
        status_.revolution_count += (dTheta_mech < 0) ? 1 : -1;
    }

    if (dt != 0u && dTheta_mech != -8192) {
        const float dt_sec = static_cast<float>(dt) * 1.25e-7f;
        const float dTheta_revs = static_cast<float>(dTheta_mech) / 16384.0f;
        const float revs_per_sec = dTheta_revs / dt_sec;
        status_.curr_vel_raw = revs_per_sec * 60.0f;

        const float alpha = dt_sec / (cfg_.vel_alpha + dt_sec);
        status_.curr_vel = status_.prev_vel + alpha * (status_.curr_vel_raw - status_.prev_vel);
        status_.prev_vel = status_.curr_vel;
    }


    // Elec Prediction within the encoder reading gap -- kind of useless
    /*
    uint32_t dt = currT - (uint32_t)foc::status.prevT;
    int16_t dTheta = (int16_t)((((int)elec - (int)foc::status.prevTheta + 8192) & 0x3FFF) - 8192);
    int aheadTheta = 0;

    if (dt != 0u && dTheta != -8192) {
   		aheadTheta = ( dTheta * ( (8000000 / (int)cfg_.current_loop_freq) - 80 ) ) / (int)dt;
   	}
     */

    status_.curr_mechAng = mechAng;
    status_.currT = T;
    status_.elecAng = elec;
    foc::status.currTheta = elec;
    foc::status.currT     = T;




}

void Controller::calibration_step() {
    foc::status.prevTheta = foc::status.currTheta;
    foc::status.prevT = foc::status.currT;

    foc::status.currTheta = (foc::status.currTheta + cfg_.dtheta_per_isr) & 0x3FFFu;
    foc::status.currT = foc::status.prevT + cfg_.ticks_per_isr;

    (void)encoder.updateAngle();
    const auto& ad = encoder.angleData();
    status_.curr_mechAng = ad.rectifiedAngle;
}


extern "C" void TIM1_UP_TIM16_IRQHandler(void) {
    if (__HAL_TIM_GET_FLAG(&htim1, TIM_FLAG_UPDATE) != RESET) {
        if (__HAL_TIM_GET_IT_SOURCE(&htim1, TIM_IT_UPDATE) != RESET) {
            __HAL_TIM_CLEAR_IT(&htim1, TIM_IT_UPDATE);

            if (controller.status().mode == Controller::Mode::Calibrating) {
            	controller.calibration_step();
            } else {
            	controller.process_encoder();
            }
        }
    }
    HAL_TIM_IRQHandler(&htim1);
}

// 20kHz current loop
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

    if (controller.status().mode == Controller::Mode::Calibrating) {
        constexpr float DIRECT_PHASE_SIGNAL = -3.14159f;
        if (foc::status.Vd != DIRECT_PHASE_SIGNAL || foc::status.Vq != DIRECT_PHASE_SIGNAL) {
            foc::InDqTransform();
            foc::DqTransform();
        }
    } else if (controller.status().mode == Controller::Mode::Idle) {
        foc::status.Vd = 0.0f;
        foc::status.Vq = 0.0f;
        foc::status.TarId = 0.0f;
        foc::status.TarIq = 0.0f;
        foc::status.i_int_d = 0.0f;
        foc::status.i_int_q = 0.0f;
        controller.status().TarVel = 0.0f;
        controller.status().TarPos = 0.0f;
        controller.status().vel_int = 0.0f;
        controller.status().pos_int = 0.0f;
        foc::DqTransform();
    } else { // Torque, Velocity, Position Mode
        if (controller.status().torque_mode == Controller::TorqueMode::Current) {
            controller.current_loop();
        } else if (controller.status().torque_mode == Controller::TorqueMode::Voltage) {
        	foc::DqTransform();
        }
    }

    controller.write_pwm();
}

// 20kHz Current Loop
void Controller::current_loop() {

	// Current Loop Implementation Later

    foc::InDqTransform();
    foc::DqTransform();
}

// 1kHz Vel and Pos loop
void Controller::velocity_loop() {

    if (status_.mode == Mode::Torque) {
        return;
    }

    const float vel_error = status_.TarVel - status_.curr_vel;
    const float dt = 1.0f / cfg_.vel_loop_freq;

    status_.vel_int += vel_error * dt;

    const float vel_p_term = cfg_.v_kp * vel_error;
    const float vel_i_term = cfg_.v_ki * status_.vel_int;
    const float vel_output = vel_p_term + vel_i_term;

    if (status_.torque_mode == Controller::TorqueMode::Voltage) {
        float vq = vel_output;
        if (vq > cfg_.max_voltage) vq = cfg_.max_voltage;
        if (vq < -cfg_.max_voltage) vq = -cfg_.max_voltage;

        foc::status.Vd = 0.0f;
        foc::status.Vq = -vq;
    } else {
        float iq = vel_output;
        if (iq > cfg_.max_current) iq = cfg_.max_current;
        if (iq < -cfg_.max_current) iq = -cfg_.max_current;

        foc::status.TarId = 0.0f;
        foc::status.TarIq = -iq;
    }
}

void Controller::position_loop() {
    // Position PID here
}

extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* h) {
    if (h->Instance == TIM6) {
        if (controller.status().mode == Controller::Mode::Position) {
            controller.position_loop();
        }

        if (controller.status().mode == Controller::Mode::Velocity ||
            controller.status().mode == Controller::Mode::Position) {
            controller.velocity_loop();
        }
    }

    if (h->Instance == TIM7) {
        HAL_GPIO_TogglePin(LED_IND_GPIO_Port, LED_IND_Pin);
    }
}
