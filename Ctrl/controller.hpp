// controller.hpp
#pragma once
#include <cstdint>
#include "tim.h"
#include "adc.h"

class Controller {
public:
    enum class Mode : uint8_t {
        Calibrating,
        Run
    };

    struct Status {
        Mode mode{Mode::Run};
        bool isCalibrated{false};
        bool armed{false};

        uint16_t elecAng{0};
        uint16_t mechAng{0};

        uint16_t dtheta_per_isr{5};
        uint16_t ticks_per_isr{320};

    };

    struct Config {
        uint32_t valid_SOF{0xA55A5AA5u};

        uint32_t current_loop_freq{25000}; //Hz
        uint16_t pole_pairs{14}; // pair
        int8_t   elec_s{-1}; // 1, -1
        uint16_t elec_offset{5067}; // 14-bit
        float    shunt_res{0.003f}; // ohms
        float    adc_gain{40.0f};

        float max_voltage{24.0f}; // Volts
        float max_current{5.0f}; // Amps

        float    i_kp{0.0f};
        float    i_ki{0.0f};
        float    v_kp{0.0f};
        float    v_ki{0.0f};
        float    p_kp{0.0f};
        float    p_ki{0.0f};

        uint16_t offset_a{2048};
        uint16_t offset_b{2048};
        uint16_t offset_c{2048};

        static constexpr uint16_t kEncLutCapacity = 512;
        int16_t  enc_err_lut[kEncLutCapacity] = {0};

        uint32_t valid_EOF{0x5AA5A55Au};
    };


    Controller() = default;

    void configure(const Config& c);
    void set_current_loop_freq(uint32_t hz);
    void init();

    void arm();
    void disarm();

    void calibrate();

    void execute();

    void write_pwm();
    void process_encoder();
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
};

