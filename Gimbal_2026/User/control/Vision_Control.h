/**
  ******************************************************************************
  * @attention
  * @file       Vision_Control.c/h
  * @brief      Vision receive interfaces (legacy XY + target offset)
  ******************************************************************************
  */

#ifndef VISION_CONTROL_H
#define VISION_CONTROL_H

#include "struct_typedef.h"
#include "main.h"
#include "CAN_receive.h"
#include "pid.h"
#include "bsp_can.h"
#include "remote_control.h"
#include "usart.h"
#include "bsp_usart.h"
#include "string.h"
#include <stdio.h>
#include "bsp_rc.h"

/* Legacy frame: X/Y */
typedef struct
{
    uint16_t x;
    uint16_t y;
    uint32_t frame_count;
    uint8_t data_ready;
    uint32_t last_update_time;
} vision_coordinate_t;

/* New frame: [H1][H2][valid][x_offset_L][x_offset_H][y_offset_L][y_offset_H][status_flags][tail] */
typedef struct
{
    uint8_t valid;
    int16_t x_offset;
    int16_t y_offset;
    uint8_t status_flags;
    uint8_t bullet_enable;
    uint8_t control_far_active;
    uint8_t fallback_active;
    uint32_t frame_count;
    uint8_t data_ready;
    uint32_t last_update_time;
} vision_target_offset_t;

typedef vision_target_offset_t vision_angle_error_t;

//֡ͷ֡β
#define VISION_HEADER_H             0x01u
#define VISION_HEADER_L             0x10u
#define VISION_TAIL                 0x11u

/* Legacy: H1 + H2 + XH + XL + YH + YL + T */
#define VISION_FRAME_LEN            7u

/* New: H1 + H2 + VALID + X_L + X_H + Y_L + Y_H + STATUS_FLAGS + T */
#define VISION_ANGLE_FRAME_LEN      9u

#define VISION_LEGACY_PAYLOAD_LEN   (VISION_FRAME_LEN - 3u)
#define VISION_ANGLE_PAYLOAD_LEN    (VISION_ANGLE_FRAME_LEN - 3u)

#define RX_BUF_LEN                  16u

#define VISION_STATUS_FIRE_ENABLED_BIT        0u
#define VISION_STATUS_CONTROL_FAR_ACTIVE_BIT  1u
#define VISION_STATUS_FALLBACK_ACTIVE_BIT     2u

/* Legacy interfaces (kept) */
void Vision_USB_Process(void);
const vision_coordinate_t *get_vision_data_point(void);
uint8_t Vision_GetXY(uint16_t *x, uint16_t *y);

/* New target-offset interfaces */
void Vision_USB_Process_TargetOffset(void);
void Vision_ReceiveBytes(const uint8_t *bytes, uint16_t length);
const vision_target_offset_t *get_vision_target_offset_data_point(void);
uint8_t Vision_GetTargetOffset(int16_t *x_offset,
                               int16_t *y_offset,
                               uint8_t *valid,
                               uint8_t *bullet_enable,
                               uint8_t *control_far_active,
                               uint8_t *fallback_active);

/* Compatibility wrappers */
void Vision_USB_Process_AngleError(void);
const vision_angle_error_t *get_vision_angle_error_data_point(void);
uint8_t Vision_GetAngleError(int16_t *x_offset,
                             int16_t *y_offset,
                             uint8_t *valid,
                             uint8_t *bullet_enable,
                             uint8_t *control_far_active,
                             uint8_t *fallback_active);

#endif
