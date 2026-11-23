/*
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     2026-4-12       zxy             视觉相关接收数据处理模块，负责从USB接收视觉数据并解析成结构体形式供上层调用。
  *  还有视觉控制云台的功能，没用上，写的不好，自瞄震荡还比较严重，由于最后没有能调试的接口都是盲调的
  * 建议以后C板CAN通信这样省下的串口用于调试不然都不好调试，尤其是视觉相关的
  ****************************(C) COPYRIGHT 2024****************************        
 */
#include "Vision_Control.h"
#include "usbd_cdc_if.h"

/*--------------------------------------------------------------------------------------------
视觉相关接收数据处理模块，负责从USB接收视觉数据并解析成结构体形式供上层调用。
还有视觉控制云台的功能，没用上，写的不好，自瞄震荡还比较严重，由于最后没有能调试的接口都是盲调的
建议以后C板CAN通信这样省下的串口用于调试不然都不好调试，尤其是视觉相关的
----------------------------------------------------------------------------------------------*/
/* USB buffers from usbd_cdc_if.c */
extern uint8_t usb_rx_buffer[];
extern uint16_t usb_rx_length;
extern uint8_t usb_rx_complete_flag;

/* Legacy data (XY) */
static vision_coordinate_t vision_data;
static uint8_t vision_buf[RX_BUF_LEN];

/* New data (target offsets + valid + bullet enable) */
static vision_target_offset_t vision_target_offset_data;
static vision_target_offset_t vision_target_offset_pending;
static volatile uint8_t vision_pending_ready;
static uint8_t vision_target_offset_buf[RX_BUF_LEN];

static int16_t vision_read_le_i16(uint8_t low, uint8_t high)
{
    return (int16_t)(((uint16_t)high << 8) | (uint16_t)low);
}

static void process_vision_data(const uint8_t *buffer)
{
    vision_data.x = (uint16_t)(((uint16_t)buffer[0] << 8) | (uint16_t)buffer[1]);
    vision_data.y = (uint16_t)(((uint16_t)buffer[2] << 8) | (uint16_t)buffer[3]);
    vision_data.frame_count++;
    vision_data.data_ready = 1u;
    vision_data.last_update_time = HAL_GetTick();
}

static void process_vision_target_offset_data(const uint8_t *buffer, uint8_t payload_len)
{
    /* New payload order:
     * [valid][x_offset_L][x_offset_H][y_offset_L][y_offset_H][status_flags]
     */
    if (payload_len != VISION_ANGLE_PAYLOAD_LEN)
    {
        return;
    }

    vision_target_offset_pending.valid = (uint8_t)((buffer[0] != 0u) ? 1u : 0u);
    vision_target_offset_pending.x_offset = vision_read_le_i16(buffer[1], buffer[2]);
    vision_target_offset_pending.y_offset = vision_read_le_i16(buffer[3], buffer[4]);
    vision_target_offset_pending.status_flags = buffer[5];
    vision_target_offset_pending.bullet_enable =
        (uint8_t)(((vision_target_offset_pending.status_flags >> VISION_STATUS_FIRE_ENABLED_BIT) & 0x01u) != 0u);
    vision_target_offset_pending.control_far_active =
        (uint8_t)(((vision_target_offset_pending.status_flags >> VISION_STATUS_CONTROL_FAR_ACTIVE_BIT) & 0x01u) != 0u);
    vision_target_offset_pending.fallback_active =
        (uint8_t)(((vision_target_offset_pending.status_flags >> VISION_STATUS_FALLBACK_ACTIVE_BIT) & 0x01u) != 0u);
    vision_target_offset_pending.frame_count++;
    vision_target_offset_pending.data_ready = 1u;
    vision_target_offset_pending.last_update_time = HAL_GetTick();
    vision_pending_ready = 1u;
}

static void vision_usb_echo_raw_data(void)
{
    if (usb_rx_length == 0u)
    {
        return;
    }

    /* Echo back the exact USB payload received from vision side. */
    USB_Transmit(usb_rx_buffer, usb_rx_length);
}

static void parse_usb_data(void)
{
    static uint8_t state = 0u;
    static uint8_t num = 0u;
    uint16_t i;

    for (i = 0u; i < usb_rx_length; i++)
    {
        uint8_t data = usb_rx_buffer[i];

        if (state == 0u)
        {
            if (data == VISION_HEADER_H)
            {
                state = 1u;
            }
        }
        else if (state == 1u)
        {
            if (data == VISION_HEADER_L)
            {
                state = 2u;
                num = 0u;
            }
            else if (data == VISION_HEADER_H)
            {
                state = 1u;
            }
            else
            {
                state = 0u;
            }
        }
        else
        {
            if (num < VISION_LEGACY_PAYLOAD_LEN)
            {
                vision_buf[num++] = data;
            }
            else
            {
                if (data == VISION_TAIL)
                {
                    process_vision_data(vision_buf);
                }

                if (data == VISION_HEADER_H)
                {
                    state = 1u;
                }
                else
                {
                    state = 0u;
                }
                num = 0u;
            }
        }
    }
}

/* Feed every USB packet in the callback, so split frames cannot be overwritten.
   Fixed length: 0x11 is also a valid byte inside an angle, not just the tail. */
void Vision_ReceiveBytes(const uint8_t *bytes, uint16_t length)
{
    static uint8_t state;
    static uint8_t num;
    uint16_t i;
    for (i = 0u; i < length; i++)
    {
        uint8_t data = bytes[i];
        if (state == 0u)
        {
            if (data == VISION_HEADER_H) state = 1u;
        }
        else if (state == 1u)
        {
            if (data == VISION_HEADER_L) { state = 2u; num = 0u; }
            else state = (uint8_t)(data == VISION_HEADER_H);
        }
        else if (num < VISION_ANGLE_PAYLOAD_LEN)
        {
            vision_target_offset_buf[num++] = data;
        }
        else
        {
            if (data == VISION_TAIL)
                process_vision_target_offset_data(vision_target_offset_buf, num);
            state = (uint8_t)(data == VISION_HEADER_H);
            num = 0u;
        }
    }
}

/* Legacy processing API */
void Vision_USB_Process(void)
{
    if (usb_rx_complete_flag != 0u)
    {
        vision_usb_echo_raw_data();
        parse_usb_data();
        usb_rx_complete_flag = 0u;
        usb_rx_length = 0u;
    }
}

/* New processing API */
void Vision_USB_Process_TargetOffset(void)
{
    uint32_t irq_mask = __get_PRIMASK();
    __disable_irq();
    if (vision_pending_ready != 0u)
    {
        vision_target_offset_data = vision_target_offset_pending;
        vision_pending_ready = 0u;
    }
    __set_PRIMASK(irq_mask);
    /* No blocking USB echo in the motor control loop. */
}

const vision_coordinate_t *get_vision_data_point(void)
{
    return &vision_data;
}

const vision_target_offset_t *get_vision_target_offset_data_point(void)
{
    return &vision_target_offset_data;
}

uint8_t Vision_GetXY(uint16_t *x, uint16_t *y)
{
    if ((x == NULL) || (y == NULL))
    {
        return 0u;
    }

    if (vision_data.data_ready != 0u)
    {
        *x = vision_data.x;
        *y = vision_data.y;
        vision_data.data_ready = 0u;
        return 1u;
    }

    return 0u;
}

uint8_t Vision_GetTargetOffset(int16_t *x_offset,
                               int16_t *y_offset,
                               uint8_t *valid,
                               uint8_t *bullet_enable,
                               uint8_t *control_far_active,
                               uint8_t *fallback_active)
{
    if ((x_offset == NULL) || (y_offset == NULL) || (valid == NULL) || (bullet_enable == NULL) ||
        (control_far_active == NULL) || (fallback_active == NULL))
    {
        return 0u;
    }

    if (vision_target_offset_data.data_ready != 0u)
    {
        *x_offset = vision_target_offset_data.x_offset;
        *y_offset = vision_target_offset_data.y_offset;
        *valid = vision_target_offset_data.valid;
        *bullet_enable = vision_target_offset_data.bullet_enable;
        *control_far_active = vision_target_offset_data.control_far_active;
        *fallback_active = vision_target_offset_data.fallback_active;
        vision_target_offset_data.data_ready = 0u;
        return 1u;
    }

    return 0u;
}

void Vision_USB_Process_AngleError(void)
{
    Vision_USB_Process_TargetOffset();
}

const vision_angle_error_t *get_vision_angle_error_data_point(void)
{
    return &vision_target_offset_data;
}

uint8_t Vision_GetAngleError(int16_t *x_offset,
                             int16_t *y_offset,
                             uint8_t *valid,
                             uint8_t *bullet_enable,
                             uint8_t *control_far_active,
                             uint8_t *fallback_active)
{
    return Vision_GetTargetOffset(x_offset, y_offset, valid, bullet_enable, control_far_active, fallback_active);
}
