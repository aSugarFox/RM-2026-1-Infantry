#ifndef __USART6_REMOTE_CONTROL_H
#define __USART6_REMOTE_CONTROL_H

#include "remote_control.h"
#include "main.h"
#include "detect_task.h"
#include "infantry_config.h"

#define USART1_RX_BUF_NUM         100U
#define USART1_FRAME_LENGTH       20U

#define USART1_FRAME_HEADER_WIT       0xA5u
#define USART1_FRAME_HEADER_RC        0xA6u
#define USART1_FRAME_HEADER_KEYBOARD  0xA7u

#define RC_CHANNAL_ERROR_VALUE    700
extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx;

typedef enum
{
    USART1_INPUT_NONE = 0,
    USART1_INPUT_RC,
    USART1_INPUT_KEYBOARD
} usart1_input_mode_e;

typedef struct
{
    uint8_t flags;
    uint8_t fault;
    uint8_t sequence;
    float chassis_gyro_speed_rpm;
    uint16_t shooter_17mm_heat;
    uint16_t shooter_17mm_heat_limit;
    uint16_t shooter_barrel_cooling_value;
    int16_t remote_ch2; /* Status bytes 14..15: echo left-stick horizontal. */
    int16_t remote_ch3; /* Status bytes 16..17: echo left-stick vertical. */
    uint8_t remote_left_switch; /* Status byte 18. */
} usart1_tx_status_t;

typedef struct
{
    uint8_t flags;
    uint8_t fault;
    uint8_t sequence;
    float yaw_angle_deg;
    float yaw_speed_rpm;
    uint32_t update_tick;
} usart1_gimbal_status_t;

void usart1_remote_control_init(void);
void USART1_IRQHandler(void);
const RC_ctrl_t *get_usart1_remote_control_point(void);
uint8_t usart1_RC_data_is_error(void);
void usart1_slove_RC_lost(void);
void usart1_slove_data_error(void);
uint16_t get_usart1_keyboard_value(void);
usart1_input_mode_e get_usart1_input_mode(void);
uint32_t get_usart1_input_mode_update_tick(void);
uint32_t get_usart1_remote_update_tick(void);
uint8_t usart1_remote_is_online(void);
uint32_t get_usart1_keyboard_update_tick(void);
float get_usart1_gimbal_yaw_angle_deg(void);
float get_usart1_gimbal_gyro_comp_speed_rpm(void);
uint8_t get_usart1_gimbal_gyro_enable(void);
uint32_t get_usart1_gimbal_gyro_update_tick(void);
const usart1_gimbal_status_t *get_usart1_gimbal_status(void);
uint8_t get_usart1_gimbal_imu_ready(void);
uint8_t get_usart1_gimbal_ready(void);
void data_to_usart1(const usart1_tx_status_t *status);
void Data_chuli(void);
uint8_t usart1_tx_is_busy(void);

#endif /* __USART6_REMOTE_CONTROL_H */
