#ifndef DM4310_H
#define DM4310_H

#include "struct_typedef.h"

typedef struct
{
    uint8_t motor_id;
    uint8_t error;
    float position_rad;
    float velocity_rad_s;
    float torque_nm;
    uint8_t mos_temperature;
    uint8_t rotor_temperature;
    uint32_t rx_tick;
} dm4310_measure_t;

typedef struct
{
    uint32_t rx_frames;
    uint32_t tx_failures;
    uint16_t last_rx_id;
    uint16_t feedback_id;
    uint8_t control_mode;
} dm4310_debug_t;

uint8_t DM4310_HandleFeedback(uint32_t std_id, const uint8_t *data, uint8_t len);
void DM4310_QueryControlMode(void);
const volatile dm4310_debug_t *DM4310_GetDebug(void);
uint8_t DM4310_Enable(void);
void DM4310_Disable(void);
void DM4310_ClearError(void);
void DM4310_RequestPositionMode(void);
uint8_t DM4310_PositionModeReady(void);
uint8_t DM4310_SetPositionSpeed(float position_rad, float max_speed_rad_s);
const dm4310_measure_t *DM4310_GetMeasure(void);
uint8_t DM4310_IsOnline(void);
uint8_t DM4310_IsHealthy(void);

#endif
