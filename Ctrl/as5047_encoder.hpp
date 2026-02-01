#pragma once
#include <cstdint>
#include <cstddef>
#include "spi.h"
#include "gpio.h"
#include "tim.h"

extern uint16_t buildCommandAS5047();
extern uint8_t  calcEvenParity(uint16_t rx);
extern uint16_t align14(uint16_t rx);
extern void     delay_us_1();

class EncoderAS5047P {
public:
    struct AngleData {
        uint16_t rawAngle{0};
        float degrees{0};
        float radians{0};
        uint32_t tstamp{0};
    };

    static constexpr uint32_t RESOLUTION = 1u << 14;

    explicit EncoderAS5047P(const volatile uint32_t* tick_source = nullptr)
        : tick_{tick_source} {}

    bool init();
    uint16_t updateAngle();
    uint16_t readDiaagc();
    const AngleData& angleData() const { return data_; }

    void setTimestampSource(const volatile uint32_t* tick_source) { tick_ = tick_source; }

private:
    const volatile uint32_t* tick_{nullptr};
    AngleData                data_{};
};
