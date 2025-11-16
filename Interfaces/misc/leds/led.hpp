#pragma once
#include <cstdint>
#include <gpio.h>
#include <tim.h>

namespace led {

    void setLed(bool on);
    void startToggle();
    void startToggle(float hz);
    void stopToggle();

};
