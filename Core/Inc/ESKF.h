//
// Created by unimage on 2026/6/27.
//

#ifndef FLIGHTCTRL_ESKF_H
#define FLIGHTCTRL_ESKF_H

#ifdef __cplusplus
extern "C" {
#endif
#include "gpio.h"
#include "stm32h7xx_hal_gpio.h"

void eskf(float* meas,float* imu);

#ifdef __cplusplus
}
#endif

#endif //FLIGHTCTRL_ESKF_H
