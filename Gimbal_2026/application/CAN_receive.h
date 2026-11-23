#ifndef CAN_RECEIVE_H
#define CAN_RECEIVE_H

#include "struct_typedef.h"

typedef struct
{
    uint16_t ecd;
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperate;
    int16_t last_ecd;
    volatile uint32_t rx_tick; /* Published by the CAN RX interrupt. */
} motor_measure_t;

typedef struct
{
    uint32_t tx_queued; /* HAL accepted the frame; this does not prove a bus ACK. */
    uint32_t tx_failed;
    int16_t command;
} can_yaw_debug_t;

const can_yaw_debug_t *CAN_yaw_get_debug(void);

typedef struct
{
    uint32_t rx_frames;
    uint32_t tx_failed;
    uint16_t seen_ids; /* Bits 0..7: CAN1 feedback 0x201..0x208 seen since boot. */
    uint16_t last_rx_id;
} can_shooter_debug_t;

const volatile can_shooter_debug_t *CAN_shooter_get_debug(void);
/* Online bits: bit 0 trigger, bit 1 left friction, bit 2 right friction. */
uint8_t CAN_shooter_online_mask(void);

void CAN_cmd_gimbal(int16_t yaw, int16_t pitch, int16_t shoot, int16_t rev);
void CAN_cmd_fric_wheel(int16_t fric_left, int16_t fric_right, int16_t rev1, int16_t rev2);
void CAN_cmd_shoot(int16_t fric_left, int16_t fric_right, int16_t trigger);
void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
void CAN_cmd_chassis_reset_ID(void);

const motor_measure_t *get_yaw_gimbal_motor_measure_point(void);
const motor_measure_t *get_pitch_gimbal_motor_measure_point(void);
const motor_measure_t *get_trigger_motor_measure_point(void);
const motor_measure_t *get_fric_wheel_left_motor_measure_point(void);
const motor_measure_t *get_fric_wheel_right_motor_measure_point(void);
const motor_measure_t *get_chassis_motor_measure_point(uint8_t i);

uint8_t CAN_yaw_motor_online(void);
uint8_t CAN_shooter_motors_online(void);

#endif
