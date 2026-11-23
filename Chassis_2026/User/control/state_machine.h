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

typedef struct
{
    system_mode_e system_mode;
    uint8_t imu_ready;
    uint8_t enabled;
    uint8_t small_gyro_active;
    uint8_t fault;
} state_machine_t;

void state_machine_init(state_machine_t *sm);
void state_machine_update(state_machine_t *sm, const RC_ctrl_t *rc_data);
void state_machine_execute(state_machine_t *sm);
uint8_t state_machine_is_enabled(const state_machine_t *sm);
uint8_t state_machine_small_gyro_active(const state_machine_t *sm);

#endif
