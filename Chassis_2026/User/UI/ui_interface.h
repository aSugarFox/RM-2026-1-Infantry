//
// Created by bismarckkk on 2024/2/17.
//

#ifndef UI_INTERFACE_H
#define UI_INTERFACE_H

#include <stdio.h>
#include "ui_types.h"

/*
 * 阵营选择（只需要改这一个宏）：
 * UI_CFG_SIDE 0  -> 红方
 * UI_CFG_SIDE 1  -> 蓝方
 */
#define UI_CFG_SIDE 0u //red
// #define UI_CFG_SIDE  1u //blue




/*
 * 默认机器人ID（常见官方映射：红3=3，蓝3=103）。
 * 如果你的裁判系统ID映射不同，可按需修改这两个值。
 */
#ifndef UI_CFG_RED_SELF_ID
#define UI_CFG_RED_SELF_ID  3u
#endif

#ifndef UI_CFG_BLUE_SELF_ID
#define UI_CFG_BLUE_SELF_ID 103u
#endif

#if (UI_CFG_SIDE)
#define UI_CFG_SELF_ID ((uint16_t)UI_CFG_BLUE_SELF_ID)
#elif (!UI_CFG_SIDE)
#define UI_CFG_SELF_ID ((uint16_t)UI_CFG_RED_SELF_ID)
#else
#error "UI_CFG_TEAM_COLOR must be UI_CFG_TEAM_RED or UI_CFG_TEAM_BLUE"
#endif

extern uint16_t ui_self_id;

void print_message(const uint8_t* message, int length);
void ui_send_message(const uint8_t *message, uint16_t length);
void ui_set_self_id(uint16_t self_id);

// User Code Begin
#define SEND_MESSAGE(message, length) \
    ui_send_message((const uint8_t *)(message), (uint16_t)(length))
// User Code End

void ui_proc_1_frame(ui_1_frame_t *msg);
void ui_proc_2_frame(ui_2_frame_t *msg);
void ui_proc_5_frame(ui_5_frame_t *msg);
void ui_proc_7_frame(ui_7_frame_t *msg);
void ui_proc_string_frame(ui_string_frame_t *msg);

#endif //UI_INTERFACE_H
