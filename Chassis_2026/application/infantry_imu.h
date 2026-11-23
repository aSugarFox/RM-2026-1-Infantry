#ifndef INFANTRY_IMU_H
#define INFANTRY_IMU_H

#include "struct_typedef.h"

uint8_t Infantry_IMU_Init(void);
void Infantry_IMU_Poll(void);
uint8_t Infantry_IMU_IsReady(void);
uint8_t Infantry_IMU_GetError(void);
/* Horizontal heading, zeroed with muzzle aligned to chassis front at boot. */
float Infantry_IMU_GetYawDeg(void);
float Infantry_IMU_GetPitchDeg(void);
float Infantry_IMU_GetRollDeg(void);
/* Horizontal yaw rate in RPM, compensated for board pitch/roll. */
float Infantry_IMU_GetGyroZRpm(void);

#endif
