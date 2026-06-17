#ifndef __TMC2160_H
#define __TMC2160_H

#include "stm32f4xx_hal.h"

/* PAN Driver */
void TMC2160_Init_PAN(void);

/* TILT Driver */
void TMC2160_Init_TILT(void);

/* Used by homing functions */
void TMC2160_PAN_HomingMode(void);
void TMC2160_PAN_RunMode(void);

void TMC2160_TILT_HomingMode(void);
void TMC2160_TILT_RunMode(void);

#endif
