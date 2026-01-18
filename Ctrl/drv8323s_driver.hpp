#pragma once
#include <cstdint>
#include "spi.h"
#include "gpio.h"

extern void delay_us_1();

class Driver8323s {
public:
    enum class OcpMode : uint8_t { Latch = 0, Retry = 1, ReportOnly = 2, Disabled = 3 };

    struct Config {
        uint16_t idrivep_hs_idx{4};            // IDRIVEP_HS mA: 10, 30, 60, 80, 120, 140, 170, 190, 260, 330, 370, 440, 570, 680, 820, 1000
        uint16_t idriven_hs_idx{4};            // IDRIVEN_HS mA: 20, 60, 120, 160, 240, 280, 340, 380, 520, 660, 740, 880, 1140, 1360, 1640, 2000
        uint16_t idrivep_ls_idx{4};            // IDRIVEP_LS mA: 10, 30, 60, 80, 120, 140, 170, 190, 260, 330, 370, 440, 570, 680, 820, 1000
        uint16_t idriven_ls_idx{4};            // IDRIVEN_LS mA: 20, 60, 120, 160, 240, 280, 340, 380, 520, 660, 740, 880, 1140, 1360, 1640, 2000
        uint16_t tdrive_idx{1};                // TDRIVE ns: 500, 1000, 2000, 4000
        uint16_t dead_time_idx{1};             // DEADTIME ns: 50, 100, 200, 400
        OcpMode  ocp_mode{OcpMode::Retry};     // OCP mode: Latch, Retry, ReportOnly, Disabled
        uint16_t ocp_deg_idx{1};               // OCP deglitch µs: 2, 4, 6, 8
        uint16_t vds_lvl_idx{9};               // VDS level mV: 60, 130, 200, 260, 310, 450, 530, 600, 680, 750, 940, 1130, 1300, 1500, 1700, 1880
        uint16_t csa_gain_idx{3};              // CSA gain V/V: 5, 10, 20, 40
        bool     vref_div_2{true};             // VREF_DIV_2: true=Vref/2, false=Vref
        bool     ls_ref_sn{false};             // LS_REF_SN: true=LS ref (SN), false=default wiring
        bool     disable_sen_oc{false};        // DISABLE_SEN_OC: true=disable shunt OC comp, false=enable
        uint16_t sen_lvl_idx{3};               // SEN_LVL mV: 250, 500, 750, 1000
    };

    struct Status {
        bool fault{false}, vds_ocp{false}, gdf{false}, uvlo{false}, oTsd{false};
        bool otw{false}, cp_uv{false}, sa_oc{false}, sb_oc{false}, sc_oc{false};
        uint16_t raw_status1{0}, raw_status2{0};
        uint16_t raw_ctrl2{0}, raw_ctrl3{0}, raw_ctrl4{0}, raw_ctrl5{0}, raw_ctrl6{0};
    };

    Driver8323s() = default;
    explicit Driver8323s(const Config& cfg) : cfg_(cfg) {}

    int init();

    bool enable();
    bool disable();

    void calBegin() { HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET); }
    void calEnd()   { HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET); }
    bool calVerifyCleared();

    bool faultAsserted() const { return HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3) == GPIO_PIN_RESET; }
    void clearFaults();
    bool readStatus(Status& out);

    bool setCsaGain(uint16_t gain_idx);
    bool setVdsLevel(uint16_t vds_lvl_idx);
    bool setOcpMode(OcpMode m);
    bool setGateDriveHS(uint16_t idrivep_idx, uint16_t idriven_idx);
    bool setGateDriveLS(uint16_t idrivep_idx, uint16_t idriven_idx, uint16_t tdrive_idx);
    bool setPwmMode3x();
    bool setDeadtime(uint16_t dead_time_idx);
    bool setCoast();
    bool setBrake();
    bool setRun();
    bool onhighz();
    bool offhighz();

    int  mode() const { return mode_; }
    const Config& config() const { return cfg_; }
    void setConfig(const Config& c) { cfg_ = c; }

    bool writeReg(uint8_t addr, uint16_t value11);
    bool writeReg_nocheck(uint8_t addr, uint16_t value11);
    bool readReg(uint8_t addr, uint16_t& value11);
    bool write_then_verify(uint8_t reg, uint16_t newval, uint16_t mask);


private:
    Config  cfg_{};
    int mode_{0};

    static uint16_t packWrite_(uint8_t addr, uint16_t data11) { return (uint16_t(addr) << 11) | (data11 & 0x07FF); }
    static uint16_t packReadCmd_(uint8_t addr) { return uint16_t(0x8000u | (uint16_t(addr) << 11)); }
};


