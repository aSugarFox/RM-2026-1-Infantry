#ifndef INFANTRY_GIMBAL_H
#define INFANTRY_GIMBAL_H

#include "remote_control.h"
#include "struct_typedef.h"

typedef enum
{
    INFANTRY_SHOOT_OFF = 0,
    INFANTRY_SHOOT_FIRE,
    INFANTRY_SHOOT_REVERSE,
    INFANTRY_SHOOT_FRICTION_ONLY
} infantry_shoot_mode_e;

void Infantry_Gimbal_Init(void);
void Infantry_Gimbal_Update(const RC_ctrl_t *rc, uint8_t enable, uint8_t auto_aim_enable);
uint8_t Infantry_Vision_CanFire(void);
float Infantry_Gimbal_GetPitchTarget(void);
/* 0: wait disabled, 1: wait mode, 2: enable, 3: running, 4: inward recovery. */
uint8_t Infantry_Gimbal_GetPitchPhase(void);
/* Diagnostic target is expressed in the world frame in both modes. */
float Infantry_Gimbal_GetYawTarget(void);
uint8_t Infantry_Gimbal_IsDialFollowing(void);
float Infantry_Gimbal_GetYawSpeedSet(void);
/* Values actually used by the current update; zero while disabled/stale. */
float Infantry_Gimbal_GetChassisYawRate(void);
float Infantry_Gimbal_GetYawFeedforward(void);
void Infantry_Shooter_Update(infantry_shoot_mode_e mode, uint8_t enable);
uint8_t Infantry_Gimbal_IsReady(void);
uint8_t Infantry_Shooter_IsReady(void);
void Infantry_PitchCalibration_Update(const RC_ctrl_t *rc, uint8_t enable);
uint8_t Infantry_PitchCalibration_IsReady(void);
float Infantry_PitchCalibration_GetTarget(void);
void Infantry_YawTest_Update(const RC_ctrl_t *rc, uint8_t enable);
float Infantry_YawTest_GetSpeedSet(void);
int16_t Infantry_YawTest_GetCommand(void);

#endif
