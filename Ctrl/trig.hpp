#pragma once
#include <cstdint>

namespace trig {
    struct SC { float sin; float cos; };
    SC get_sin_cos(uint16_t theta14) noexcept;
}
