// foc.cpp (add/modify)
#include "foc.hpp"
#include <algorithm>
#include <cstdint>

namespace foc
{

FOCstatus status{};

static inline uint16_t tim1_arr() {
    return __HAL_TIM_GET_AUTORELOAD(&htim1);
}

static inline float clampf(float x, float lo, float hi) {
    return x < lo ? lo : (x > hi ? hi : x);
}

AB ClarkeTransform() noexcept
{
    const float a = status.cA;
    const float b = status.cB;
    const float c = status.cC;
    const float k = SQRT_2_OVER_3;
    AB out;
    out.a = k * (a - 0.5f * (b + c));
    out.b = k * (0.5f * SQRT3 * (b - c));
    return out;
}


ABC InClarkeTransform(float alpha, float beta) noexcept
{
    ABC out;
    out.a =  SQRT_2_OVER_3 * alpha;
    out.b = -SQRT_1_OVER_6 * alpha + SQRT_1_OVER_2 * beta;
    out.c = -SQRT_1_OVER_6 * alpha - SQRT_1_OVER_2 * beta;
    return out;
}

DQ ParkTransform(float alpha, float beta) noexcept
{
    trig::SC sc = trig::get_sin_cos(status.currTheta);

    DQ out;
    out.d =  alpha * sc.cos + beta * sc.sin;
    out.q = -alpha * sc.sin + beta * sc.cos;
    return out;
}

AB InParkTransform(float d, float q) noexcept
{
    trig::SC sc = trig::get_sin_cos(status.currTheta);

    AB out;
    out.a = d * sc.cos - q * sc.sin;
    out.b = d * sc.sin + q * sc.cos;
    return out;
}

void svpwm(float a, float b, float c) noexcept
{
    float vmin = (a < b ? (a < c ? a : c) : (b < c ? b : c));
    float vmax = (a > b ? (a > c ? a : c) : (b > c ? b : c));
    float voff = -0.5f * (vmax + vmin);
    status.phaseA = a + voff;
    status.phaseB = b + voff;
    status.phaseC = c + voff;
}

void InDqTransform() noexcept
{
    AB ab = ClarkeTransform();
    DQ dq = ParkTransform(ab.a, ab.b);

    // filter
    status.Id = status.prevId + status.cur_alpha * (dq.d - status.prevId);
    status.Iq = status.prevIq + status.cur_alpha * (dq.q - status.prevIq);
    status.prevId = status.Id;
    status.prevIq = status.Iq;

}

void DqTransform() noexcept
{
    AB ab = InParkTransform(status.Vd, status.Vq);
    ABC abc = InClarkeTransform(ab.a, ab.b);
    svpwm(abc.a, abc.b, abc.c);
}


}
