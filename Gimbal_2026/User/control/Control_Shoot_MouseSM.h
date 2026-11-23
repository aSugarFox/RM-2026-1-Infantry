#ifndef CONTROL_SHOOT_MOUSE_SM_H
#define CONTROL_SHOOT_MOUSE_SM_H

#include "Control_Shoot.h"

typedef enum
{
    MOUSE_SHOOT_SM_DISABLED = 0,/* 禁用状态，摩擦轮未启动或未准备好，无法射击 */
    MOUSE_SHOOT_SM_READY,/* 准备就绪状态，摩擦轮已准备好，等待射击输入 */
    MOUSE_SHOOT_SM_SINGLE,/* 单发射击状态 */
    MOUSE_SHOOT_SM_CONTINUOUS,/* 连续射击状态 */
    MOUSE_SHOOT_SM_JAM_BACKOFF/* 卡弹回退状态 */
} mouse_shoot_sm_mode_t;

typedef struct
{
    mouse_shoot_sm_mode_t mode;
    uint8_t fric_ready;
    uint8_t jammed;

    uint8_t left_pressed;
    uint8_t left_pressed_last;
    uint16_t left_hold_ticks;

    uint16_t fric_ready_ticks;
    uint16_t jam_ticks;

    uint16_t heat_block_ticks;
    uint16_t heat_limit;
    uint16_t heat_now;
    uint16_t heat_effective;
    uint16_t heat_reserved;
    uint16_t heat_last_now;
    uint16_t heat_cooling_value;
    uint16_t heat_predicted;
    uint8_t heat_blocked;
    uint8_t heat_data_valid;
    uint32_t heat_sync_tick;

    int32_t total_ecd;
    int16_t last_ecd;

    fp32 current_angle_deg;
    fp32 target_angle_deg;
    fp32 speed_set_rpm;

    uint32_t single_shot_count;
    uint32_t continuous_shot_count;
} mouse_shoot_sm_t;

void Mouse_Shoot_SM_Init(void);
void Mouse_Shoot_SM_Reset(void);
void Mouse_Shoot_SM_Update(uint8_t single_shot_trigger);
void Mouse_Shoot_SM_ForceNoFeed(void); /* 强制停拨弹（2006停止），不关闭摩擦轮 */
void Mouse_Shoot_SM_SetHeatFromUart1(uint16_t heat_limit, uint16_t heat, uint8_t valid);
const mouse_shoot_sm_t *Mouse_Shoot_SM_Get(void);

#endif

