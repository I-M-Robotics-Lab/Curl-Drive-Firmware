#ifndef COMMON_INC
#define COMMON_INC

#ifdef __cplusplus
extern "C" {
#endif
/*---------------------------- C Scope ---------------------------*/
#include "stdint-gcc.h"

void Main();

#ifdef __cplusplus
}

/*---------------------------- C++ Scope ---------------------------*/
#include <cstdio>
#include <charconv>
#include <misc/leds/led.hpp>
#include <misc/pwr/pwr.hpp>
#include <misc/temp/temp.hpp>
#include <usbs/usb.hpp>
#include <as5047_encoder.hpp>
#include <drv8323s_driver.hpp>
#include <controller.hpp>
#include <foc.hpp>

extern Controller controller;
extern EncoderAS5047P encoder;
extern Driver8323s    drv;

void cli_poll();

#endif
#endif
