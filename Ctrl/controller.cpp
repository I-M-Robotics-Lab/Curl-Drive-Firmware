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
    erase.Page      = 158u;
    erase.NbPages   = 2;

    uint32_t page_error = 0;
    HAL_StatusTypeDef st;

    HAL_FLASH_Unlock();
    __disable_irq();
    st = HAL_FLASHEx_Erase(&erase, &page_error);
    if (st != HAL_OK) { __enable_irq(); HAL_FLASH_Lock(); return false; }

    const uint8_t* src = reinterpret_cast<const uint8_t*>(&cfg_);
    const size_t   len = sizeof(cfg_);
    uintptr_t      dst = 0x0804F000u;

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
    const uint32_t tim_clk = 72000000u;
    const uint32_t den     = (hz > 0u) ? hz : 1u;
    uint32_t arr_u32       = (tim_clk / 72u) / den;
    if (arr_u32 < 2u) arr_u32 = 2u;
    const uint16_t arr     = static_cast<uint16_t>(arr_u32 - 1u);

    __HAL_TIM_DISABLE(&htim6);
    __HAL_TIM_SET_AUTORELOAD(&htim6, arr);
    __HAL_TIM_ENABLE(&htim6);
}

void Controller::set_zero() {

	status_.mode = Mode::Idle;
	HAL_Delay(100);

    status_.pos_zero_offset = (static_cast<float>(status_.revolution_count) * 360.0f) +
                              (static_cast<float>(status_.curr_mechAng) / 16384.0f) * 360.0f;

    status_.curr_pos = 0.0f;
    status_.prev_pos = 0.0f;
    status_.curr_pos_raw = 0.0f;
    status_.curr_vel = 0.0f;
    status_.prev_vel = 0.0f;
    status_.curr_vel_raw = 0.0f;
    status_.pos_int = 0.0f;
    status_.vel_int = 0.0f;
    status_.TarPos = 0.0f;
    status_.TarVel = 0.0f;
    status_.revolution_count = 0;

}

void Controller::disarm() {
    //drv.onhighz();

    if (drv.setCoast()) {
        foc::status.TarId  = 0.0f;
        foc::status.TarIq  = 0.0f;
        foc::status.Vq	   = 0.0f;
		foc::status.Vd 	   = 0.0f;
        foc::status.phaseA = 0.0f;
        foc::status.phaseB = 0.0f;
        foc::status.phaseC = 0.0f;
        foc::status.i_int_d = 0.0f;
        foc::status.i_int_q = 0.0f;
        controller.status().TarVel = 0.0f;
        controller.status().TarPos = 0.0f;
        controller.status().vel_int = 0.0f;
        controller.status().pos_int = 0.0f;

        status_.mode = Mode::Idle;
        status_.armed = false;
    }
}

void Controller::arm() {
    foc::status.TarId  = 0.0f;
    foc::status.TarIq  = 0.0f;
    foc::status.Vq	   = 0.0f;
	foc::status.Vd 	   = 0.0f;
    foc::status.phaseA = 0.0f;
    foc::status.phaseB = 0.0f;
    foc::status.phaseC = 0.0f;

    Driver8323s::Status s{};
    if (drv.readStatus(s)) {
        const bool all_clear = ((static_cast<uint16_t>(s.raw_status1) | static_cast<uint16_t>(s.raw_status2)) == 0u);

        if (all_clear) {
            //drv.offhighz();
            if (drv.setRun()) {
                status_.armed = true;
                return;
            }
        }
    }
    status_.armed = false;
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
    HAL_Delay(200);

    uint32_t acc = 0;
    for (uint16_t i = 0; i < 1000; i++) {
        acc += status_.curr_mechAng;
        HAL_Delay(1);
    }
    uint16_t mech_avg = static_cast<uint16_t>(acc / 1000u);
    uint16_t elec_measured = static_cast<uint16_t>((mech_avg * cfg_.pole_pairs * cfg_.elec_s) & 0x3FFFu);
    cfg_.elec_offset = static_cast<uint16_t>((0u - elec_measured) & 0x3FFFu);

    HAL_Delay(200);

    process_encoder();
    foc::status.Vd = 0.0f;
    foc::status.Vq = 2.0f;

    HAL_Delay(500);

    foc::status.Vd = -3.14159f; //activate special phase control mode
    foc::status.Vq = -3.14159f;

    foc::status.phaseA =  5.0f;
    foc::status.phaseB = -2.5f;
    foc::status.phaseC = -2.5f;
    HAL_Delay(200);

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

    //LUT creation
    HAL_Delay(200);

    for (int i = 0; i < 512; i++) {
        cfg_.enc_err_lut[i] = 0;
        lut_cal_count[i] = 0;
    }

    process_encoder();
    foc::status.Vd = 0.0f;
    foc::status.Vq = 2.0f;

    lut_cal_on = true;

    // Time for 2 full mechanical revolutions
    HAL_Delay((2u * 16384u * cfg_.pole_pairs * 1000u) / (cfg_.dtheta_per_isr * cfg_.current_loop_freq));

    lut_cal_on = false;

    for (int i = 0; i < 512; i++) {
        if (lut_cal_count[i] > 0) {
            int32_t avg = cfg_.enc_err_lut[i] / (int32_t)lut_cal_count[i];
            if (avg > 16)  avg = 16;
            if (avg < -16) avg = -16;
            cfg_.enc_err_lut[i] = avg;
        } else if (i > 0) {
            cfg_.enc_err_lut[i] = cfg_.enc_err_lut[i - 1];
        }
    }

    HAL_Delay(200);

    foc::status.Vd = 0.0f;
    foc::status.Vq = 0.0f;
    foc::status.phaseA = 0.0f;
    foc::status.phaseB = 0.0f;
    foc::status.phaseC = 0.0f;
    foc::status.currTheta = 0;
    foc::status.prevTheta = 0;
    foc::status.currT = 0;
    foc::status.prevT = 0;
    process_encoder();

    HAL_Delay(100);

    cfg_.is_calibrated = true;
    status_.isCalibrated = true;

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

    HAL_NVIC_SetPriority(TIM3_IRQn, 1, 0);
    HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 2, 0);
    HAL_NVIC_SetPriority(TIM7_DAC_IRQn, 3, 0);

	set_current_loop_freq(cfg_.current_loop_freq);
    __HAL_TIM_MOE_DISABLE(&htim1);

    HAL_NVIC_DisableIRQ(ADC1_2_IRQn);
    __HAL_ADC_DISABLE_IT(&hadc2, ADC_IT_JEOS | ADC_IT_JEOC);
    __HAL_ADC_CLEAR_FLAG(&hadc2, ADC_FLAG_JEOC | ADC_FLAG_JEOS | ADC_FLAG_OVR | ADC_FLAG_AWD1);

    set_velocity_loop_freq(cfg_.vel_loop_freq);
    __HAL_TIM_SET_COUNTER(&htim6, 0u);
    __HAL_TIM_CLEAR_FLAG(&htim6, TIM_FLAG_UPDATE);
    __HAL_TIM_ENABLE_IT(&htim6, TIM_IT_UPDATE);
    HAL_TIM_Base_Start_IT(&htim6);

    drv.clearFaults();
    HAL_Delay(1);
    drv.calBegin();

    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
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
    HAL_NVIC_EnableIRQ(ADC1_2_IRQn);

    HAL_TIM_Base_Start_IT(&htim3);

    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

    __HAL_TIM_MOE_ENABLE(&htim1);
}















void Controller::write_pwm() {

    if (!status_.armed) {
        TIM1->CCR3 = 0;
        TIM1->CCR2 = 0;
        TIM1->CCR1 = 0;
        return;
    }

    const uint16_t arr = TIM1->ARR;
    const float vbus = pwr::volts();
    foc::status.vbus = vbus;

    const float v_max = (vbus > controller.config().max_voltage) ? controller.config().max_voltage :
                        ((vbus > 0.0f) ? vbus : 1.0f);

    const float mid = 0.5f * arr;
    const float half_span = 0.5f * (float(arr) - 2.0f);
    const float inv_vmax = 1.0f / v_max;
    const float arr_limit = float(arr - 1);

    // Phase A
    float u = foc::status.phaseA * inv_vmax;
    u = (u > 1.0f) ? 1.0f : ((u < -1.0f) ? -1.0f : u);
    float c = mid + half_span * u;
    c = (c < 1.0f) ? 1.0f : ((c > arr_limit) ? arr_limit : c);
    TIM1->CCR3 = uint16_t(c);

    // Phase B
    u = foc::status.phaseB * inv_vmax;
    u = (u > 1.0f) ? 1.0f : ((u < -1.0f) ? -1.0f : u);
    c = mid + half_span * u;
    c = (c < 1.0f) ? 1.0f : ((c > arr_limit) ? arr_limit : c);
    TIM1->CCR2 = uint16_t(c);

    // Phase C
    u = foc::status.phaseC * inv_vmax;
    u = (u > 1.0f) ? 1.0f : ((u < -1.0f) ? -1.0f : u);
    c = mid + half_span * u;
    c = (c < 1.0f) ? 1.0f : ((c > arr_limit) ? arr_limit : c);
    TIM1->CCR1 = uint16_t(c);
}

uint16_t Controller::mech_to_elec(uint16_t mech_theta14) const {
    uint16_t elec = static_cast<uint16_t>(mech_theta14 & 0x3FFF);
    elec = static_cast<uint16_t>((static_cast<uint32_t>(elec) * static_cast<uint32_t>(cfg_.pole_pairs)) & 0x3FFF);
    if (cfg_.elec_s < 0) elec = static_cast<uint16_t>((0u - elec) & 0x3FFF);
    elec = static_cast<uint16_t>((elec + (cfg_.elec_offset & 0x3FFF)) & 0x3FFF);
    return elec;
}

uint16_t Controller::apply_encoder_correction(uint16_t raw) const {
    if (!status_.isCalibrated) return raw;

    constexpr uint16_t LUT_SHIFT = 5;
    constexpr uint16_t LUT_MASK = Config::kEncLutSize - 1;
    constexpr uint16_t FRAC_MASK = (1u << LUT_SHIFT) - 1;

    const uint16_t idx = raw >> LUT_SHIFT;
    const uint16_t frac = raw & FRAC_MASK;
    const uint16_t idx_next = (idx + 1) & LUT_MASK;

    const int32_t e0 = cfg_.enc_err_lut[idx];
    const int32_t e1 = cfg_.enc_err_lut[idx_next];

    const int32_t err = e0 + (((e1 - e0) * static_cast<int32_t>(frac)) >> LUT_SHIFT);

    return static_cast<uint16_t>((static_cast<int32_t>(raw) + err) & 0x3FFF);
}

void Controller::process_encoder() {

    (void)encoder.updateAngle();

    const auto& ad = encoder.angleData();
    uint16_t rawAng = ad.rawAngle;
    uint16_t mechAng = apply_encoder_correction(rawAng);
    //uint16_t mechAng = rawAng;
    uint32_t T = (uint32_t)ad.tstamp;
    uint16_t elec = mech_to_elec(mechAng);

    status_.prev_T = status_.curr_T;
    status_.prev_mechAng = status_.curr_mechAng;

    foc::status.prevTheta = foc::status.currTheta;
    foc::status.prevT     = foc::status.currT;

    uint32_t dt = T - status_.prev_T;
    int16_t dTheta_mech = (int16_t)((((int)mechAng - (int)status_.prev_mechAng + 8192) & 0x3FFF) - 8192);

    if (status_.prev_mechAng > 12288 && mechAng < 4096) {
        status_.revolution_count++;
    } else if (status_.prev_mechAng < 4096 && mechAng > 12288) {
        status_.revolution_count--;
    }

    if (dt != 0u && dTheta_mech != -8192) {
        const float dt_sec = static_cast<float>(dt) * 1.25e-7f;
        const float dTheta_revs = static_cast<float>(dTheta_mech) / 16384.0f;
        const float revs_per_sec = dTheta_revs / dt_sec;
        status_.curr_vel_raw = revs_per_sec * 60.0f;

        status_.curr_vel = status_.prev_vel + cfg_.vel_alpha * (status_.curr_vel_raw - status_.prev_vel);
        status_.prev_vel = status_.curr_vel;
    }

    const float position_raw = (static_cast<float>(status_.revolution_count) * 360.0f +
                               (static_cast<float>(mechAng) / 16384.0f) * 360.0f) - status_.pos_zero_offset;

    status_.curr_pos = status_.prev_pos + cfg_.pos_alpha * (position_raw - status_.prev_pos);
    status_.prev_pos = status_.curr_pos;
    status_.curr_pos_raw = position_raw;

    status_.curr_mechAng = mechAng;
    status_.raw_mechAng = rawAng;
    status_.curr_T = T;
    status_.elecAng = elec;
    foc::status.currTheta = elec;
    foc::status.currT     = T;
}

void Controller::calibration_step() {
    foc::status.prevT = foc::status.currT;
    foc::status.currT = foc::status.prevT + cfg_.ticks_per_isr;

    foc::status.currTheta = (foc::status.currTheta + cfg_.dtheta_per_isr) & 0x3FFFu;

    (void)encoder.updateAngle();
    uint16_t raw = encoder.angleData().rawAngle;
    status_.curr_mechAng = raw;

    if (lut_cal_on) {
        int32_t cal_elec = (int32_t)foc::status.currTheta - cfg_.elec_offset;
        if (cfg_.elec_s < 0) cal_elec = -cal_elec;
        uint16_t pred_mech = (uint16_t)((cal_elec & 0x3FFF) / cfg_.pole_pairs);

        int16_t diff = (int16_t)(pred_mech - (raw % (16384 / cfg_.pole_pairs)));
        if (diff > 4096)  diff -= 8192;
        if (diff < -4096) diff += 8192;

        uint16_t idx = raw >> 5;
        cfg_.enc_err_lut[idx] += diff;
        lut_cal_count[idx]++;
    }
}


// 20kHz current loop
extern "C" void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef* hadc) {
    if (hadc != &hadc2) return;

    GPIOA->BSRR = GPIO_PIN_15;

    const uint16_t r1 = ADC2->JDR1;
    const uint16_t r2 = ADC2->JDR2;
    const uint16_t r3 = ADC2->JDR3;

    constexpr float VSCALE = 3.3f / 4095.0f;

    const int32_t c1 = int32_t(r1) - int32_t(controller.config().offset_a);
    const int32_t c2 = int32_t(r2) - int32_t(controller.config().offset_b);
    const int32_t c3 = int32_t(r3) - int32_t(controller.config().offset_c);

    const float va = float(c1) * VSCALE;
    const float vb = float(c2) * VSCALE;
    const float vc = float(c3) * VSCALE;

    const float inv_denom = 1.0f / (controller.config().adc_gain * controller.config().shunt_res);

    foc::status.cA = va * inv_denom;
    foc::status.cB = vb * inv_denom;
    foc::status.cC = vc * inv_denom;

    // FOC part
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
    } else {
        if (controller.config().torque_mode == Controller::TorqueMode::Current) {
            controller.current_loop();
        } else if (controller.config().torque_mode == Controller::TorqueMode::Voltage) {
            foc::InDqTransform();
            foc::DqTransform();
        }
    }

    controller.write_pwm();

    GPIOA->BSRR = (uint32_t)GPIO_PIN_15 << 16U;

}

// 20kHz Current Loop
void Controller::current_loop() {
    foc::InDqTransform();

    const float d_error = foc::status.TarId - foc::status.Id;
    const float q_error = foc::status.TarIq - foc::status.Iq;
    const float dt = 1.0f / static_cast<float>(cfg_.current_loop_freq);

    foc::status.i_int_d += d_error * cfg_.i_ki * dt;
    foc::status.i_int_q += q_error * cfg_.i_ki * dt;

    if (foc::status.i_int_d > cfg_.max_voltage) foc::status.i_int_d = cfg_.max_voltage;
    if (foc::status.i_int_d < -cfg_.max_voltage) foc::status.i_int_d = -cfg_.max_voltage;

    if (foc::status.i_int_q > cfg_.max_voltage) foc::status.i_int_q = cfg_.max_voltage;
    if (foc::status.i_int_q < -cfg_.max_voltage) foc::status.i_int_q = -cfg_.max_voltage;

    float vd_out = (d_error * cfg_.i_kp) + foc::status.i_int_d;
    float vq_out = (q_error * cfg_.i_kp) + foc::status.i_int_q;

    if (vd_out > cfg_.max_voltage) vd_out = cfg_.max_voltage;
    if (vd_out < -cfg_.max_voltage) vd_out = -cfg_.max_voltage;

    if (vq_out > cfg_.max_voltage) vq_out = cfg_.max_voltage;
    if (vq_out < -cfg_.max_voltage) vq_out = -cfg_.max_voltage;

    foc::status.Vd = vd_out;
    foc::status.Vq = vq_out;

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

    if (cfg_.torque_mode == Controller::TorqueMode::Voltage) {
        float vq = vel_output;
        if (vq > cfg_.max_voltage) vq = cfg_.max_voltage;
        if (vq < -cfg_.max_voltage) vq = -cfg_.max_voltage;

        foc::status.Vd = 0.0f;
        foc::status.Vq = vq;
    } else {
        float iq = vel_output;
        if (iq > cfg_.max_current) iq = cfg_.max_current;
        if (iq < -cfg_.max_current) iq = -cfg_.max_current;

        foc::status.TarId = 0.0f;
        foc::status.TarIq = iq;
    }
}

void Controller::position_loop() {
    const float pos_error = status_.TarPos - status_.curr_pos;
    const float dt = 1.0f / cfg_.vel_loop_freq;

    status_.pos_int += pos_error * dt;

    const float pos_p_term = cfg_.p_kp * pos_error;
    const float pos_i_term = cfg_.p_ki * status_.pos_int;
    const float pos_d_term = cfg_.p_kd * status_.curr_vel;

    const float pos_output = pos_p_term + pos_i_term - pos_d_term;

    float tar_vel = pos_output;
    if (tar_vel > cfg_.max_vel) tar_vel = cfg_.max_vel;
    if (tar_vel < -cfg_.max_vel) tar_vel = -cfg_.max_vel;

    status_.TarVel = tar_vel;
}

void Controller::set_torque(float d_val, float q_val) {
    status_.mode = Mode::Torque;

    if (cfg_.torque_mode == TorqueMode::Voltage) {
        if (d_val > cfg_.max_voltage) d_val = cfg_.max_voltage;
        if (d_val < -cfg_.max_voltage) d_val = -cfg_.max_voltage;
        if (q_val > cfg_.max_voltage) q_val = cfg_.max_voltage;
        if (q_val < -cfg_.max_voltage) q_val = -cfg_.max_voltage;

        foc::status.Vd = d_val;
        foc::status.Vq = q_val;
    } else {
        if (d_val > cfg_.max_current) d_val = cfg_.max_current;
        if (d_val < -cfg_.max_current) d_val = -cfg_.max_current;
        if (q_val > cfg_.max_current) q_val = cfg_.max_current;
        if (q_val < -cfg_.max_current) q_val = -cfg_.max_current;

        foc::status.TarId = d_val;
        foc::status.TarIq = q_val;
    }
}

void Controller::set_velocity(float vel) {
    status_.mode = Mode::Velocity;

    if (vel > cfg_.max_vel) vel = cfg_.max_vel;
    if (vel < -cfg_.max_vel) vel = -cfg_.max_vel;

    status_.TarVel = vel;
}

void Controller::set_position(float pos) {
    status_.mode = Mode::Position;
    status_.TarPos = pos;
}

extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* h) {
    if (h->Instance == TIM3) {
    	GPIOC->BSRR = GPIO_PIN_10;

        if (controller.status().mode == Controller::Mode::Calibrating) {
            controller.calibration_step();
        } else {
            controller.process_encoder();
        }

        GPIOC->BSRR = (uint32_t)GPIO_PIN_10 << 16U;
    }

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
