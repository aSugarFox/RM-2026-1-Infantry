#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include "remote_control.h"
#include "struct_typedef.h"

typedef enum
{
    SYSTEM_DISABLE = 0,
    SYSTEM_MANUAL,
    SYSTEM_SMALL_GYRO
} system_mode_e;

typedef enum
{
    LAUNCHER_DISABLE = 0,
    LAUNCHER_ENABLE,
    LAUNCHER_REVERSE,
    LAUNCHER_REVERSE_HOLD
} launcher_mode_e;

typedef struct
{
    system_mode_e system_mode;
    launcher_mode_e launcher_mode;
    uint8_t prev_sright;
    uint8_t gimbal_ready;
    uint8_t shoot_ready;
    uint8_t estop;
    uint32_t reverse_until_tick;
} state_machine_t;

void state_machine_init(state_machine_t *sm);
void state_machine_update(state_machine_t *sm, const RC_ctrl_t *rc_data);
void state_machine_execute(state_machine_t *sm);
uint8_t state_machine_gimbal_ready(const state_machine_t *sm);
uint8_t state_machine_shoot_ready(const state_machine_t *sm);
uint8_t state_machine_estop(const state_machine_t *sm);

#endif
