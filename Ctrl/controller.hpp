// controller.hpp
#pragma once
#include <cstdint>
#include "tim.h"
#include "adc.h"

constexpr float TWO_PI = 6.28318530718f;

class Controller {
public:
	enum class Mode : uint8_t {
	    Idle,
	    Torque,
	    Velocity,
	    Position,
	    Calibrating
	};

    enum class TorqueMode : uint8_t {
        Voltage,
        Current
    };

    struct Status {
        Mode mode{Mode::Idle};

        bool isCalibrated{false};
        bool armed{false};

        uint32_t prev_T{0};
        uint32_t curr_T{0};
        uint16_t elecAng{0};
        uint16_t raw_mechAng{0};
        uint16_t curr_mechAng{0};
        uint16_t prev_mechAng{0};

        int32_t revolution_count{0};

        float pos_zero_offset = 0.0f;

        float curr_vel_raw{0.0f};  // rpm
        float curr_vel{0.0f};      // rpm filtered
        float prev_vel{0.0f};

        float curr_pos{0.0f}; // degrees
        float curr_pos_raw{0.0f};
        float prev_pos{0.0f};

        float vel_int{0.0f};
        float pos_int{0.0f};

        float TarVel{0.0f}; //rpm
        float TarPos{0.0f};
    };

    struct Config {
        uint32_t valid_SOF{0xA55A5AA5u};

        bool     is_calibrated{false};
        TorqueMode torque_mode{TorqueMode::Current};

        uint32_t current_loop_freq{20000}; //Hz
        uint32_t vel_loop_freq{1000}; //Hz
        uint16_t pole_pairs{14}; // pair
        int8_t   elec_s{-1}; // 1, -1
        uint16_t elec_offset{0}; // 14-bit
        float    shunt_res{0.02f}; // ohms
        float    adc_gain{40.0f};

        uint16_t dtheta_per_isr{5}; //openloop
        uint16_t ticks_per_isr{320};

        float max_voltage{16.0f}; // Volts
        float max_current{0.3f}; // Amps
        float max_vel{500.0f}; // rpm

        float    i_kp{150.0f};
        float    i_ki{1.0f};
        float    v_kp{0.13f};
        float    v_ki{0.5f};
        float    p_kp{1.5f};
        float    p_ki{0.0f};
        float 	 p_kd{0.0f};

        float 	 vel_alpha{0.01f};
        float    pos_alpha{0.05f};

        uint16_t offset_a{2048};
        uint16_t offset_b{2048};
        uint16_t offset_c{2048};

        static constexpr uint16_t kEncLutSize = 512;
        int32_t  enc_err_lut[kEncLutSize] = {0};

        uint32_t valid_EOF{0x5AA5A55Au};
    };


    Controller() = default;

    void configure(const Config& c);
    void set_current_loop_freq(uint32_t hz);
    void set_velocity_loop_freq(uint32_t hz);
    void init();

    void set_zero();

    void arm();
    void disarm();

    void calibrate();

    void execute();

    void current_loop();
    void position_loop();
    void velocity_loop();

    void set_torque(float d_val, float q_val);
    void set_velocity(float vel);
    void set_position(float pos);

    void write_pwm();
    void process_encoder();
    void calibration_step();
    uint16_t mech_to_elec(uint16_t mech_theta14) const;

    bool config_write();
    bool config_read();

    bool isArmed() const { return status_.armed; }
    const Config& config() const { return cfg_; }
    Config&       config()       { return cfg_; }
    const Status& status() const { return status_; }
    Status&       status()       { return status_; }

private:
    Config cfg_{};
    Status status_{};

    uint16_t lut_cal_count[512];
    bool     lut_cal_on{false};

    uint16_t apply_encoder_correction(uint16_t raw) const;
};

