// foc.hpp (add/replace pieces)
#pragma once
#include "trig.hpp"
#include "common_inc.hpp"
#include <cstdint>

namespace foc
{

struct FOCstatus
{
    uint16_t currTheta{0};
    uint16_t prevTheta{0};
    uint32_t    currT{0};
    uint32_t    prevT{0};

    float cA{0.0f};
    float cB{0.0f};
    float cC{0.0f};

    float phaseA{0.0f};
    float phaseB{0.0f};
    float phaseC{0.0f};

    float Iq{0.0f};
    float Id{0.0f};
    float prevIq{0.0f};
    float prevId{0.0f};
    float cur_alpha{0.2f};

    float Vq{0.0f};
    float Vd{0.0f};
    float vbat{0.0f};

    float TarId{0.0f};
    float TarIq{0.0f};

    float i_int_d{0.0f};
    float i_int_q{0.0f};

};

extern FOCstatus status;

inline constexpr float SQRT_2_OVER_3 = 0.81649658092772603f;
inline constexpr float SQRT_3_OVER_2 = 1.22474487139158905f;
inline constexpr float SQRT3          = 1.73205080756887729f;
inline constexpr float INV_SQRT3      = 0.57735026918962584f;
inline constexpr float SQRT_1_OVER_2 = 0.70710678118654752f;
inline constexpr float SQRT_1_OVER_6 = 0.40824829046386302f;

struct AB { float a; float b; };
struct DQ { float d; float q; };
struct ABC { float a; float b; float c; };

AB  ClarkeTransform() noexcept;
ABC InClarkeTransform(float alpha, float beta) noexcept;
DQ  ParkTransform(float alpha, float beta) noexcept;
AB  InParkTransform(float d, float q) noexcept;

void InDqTransform() noexcept;
void DqTransform() noexcept;
void svpwm(float a, float b, float c) noexcept;
void filter_currents() noexcept;

}
