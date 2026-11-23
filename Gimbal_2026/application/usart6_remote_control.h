/**
  ****************************(C) COPYRIGHT 2024****************************
  * @file       usart1_remote_control.h
  * @brief      串口1接收转发的遥控器数据处理
  * @note       该模块用于接收通过其他串口转发的SBUS数据，并进行解码
  ****************************(C) COPYRIGHT 2024****************************
  */
 /*
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     2026-4-12       zxy             串口收发任务
  *   这里有串口一和底盘通信，发送云台陀螺仪Yaw角，接收底盘旋转角速度
  *   串口六和图传通信，接收图传数据，例如鼠标键盘
  *   板子上丝印是错的串口一实际是串口六，串口二实际是串口一具体看C板说明手册
  ****************************(C) COPYRIGHT 2024****************************        
  */

#ifndef __USART6_REMOTE_CONTROL_H
#define __USART6_REMOTE_CONTROL_H

#include "remote_control.h"
#include "main.h"
#include "detect_task.h"
#include "infantry_config.h"

#define USART1_RX_BUF_NUM 36
#define USART1_FRAME_LENGTH 20


/* USART1 20-byte frame headers */
#define USART1_FRAME_HEADER_WIT       0xA5u
#define USART1_FRAME_HEADER_RC        0xA6u
#define USART1_FRAME_HEADER_KEYBOARD  0xA7u
#define RC_CHANNAL_ERROR_VALUE 700
extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx;

/*
 * 底盘->云台 状态帧（0xA5）接收结果：
 * [1]      flags                       IMU/底盘/小陀螺状态位
 * [2]      fault                       底盘故障位
 * [3]      sequence                    发送序号
 * [4..7]   chassis_gyro_speed_rpm      底盘绕Z轴速度
 * [8..9]   shooter_17mm_heat           17mm当前热量
 * [10..11] shooter_17mm_heat_limit     17mm热量上限
 * [12..13] shooter_barrel_cooling_value 17mm枪管冷却值/秒
 */
typedef struct
{
    uint8_t flags;
    uint8_t fault;
    uint8_t sequence;
    float chassis_gyro_speed_rpm;
    uint16_t shooter_17mm_heat;
    uint16_t shooter_17mm_heat_limit;
    uint16_t shooter_barrel_cooling_value;
    int16_t remote_ch2; /* Status bytes 14..15: chassis-received left stick. */
    int16_t remote_ch3; /* Status bytes 16..17. */
    uint8_t remote_left_switch; /* Status byte 18. */
} usart1_chassis_status_t;

typedef struct
{
    uint8_t flags;
    uint8_t fault;
    uint8_t sequence;
    float yaw_angle_deg;
    float yaw_speed_rpm;
    uint16_t yaw_encoder; /* A7 bytes 12..13, little-endian raw GM6020 Ecd. */
} usart1_gimbal_tx_status_t;

typedef struct
{
    float relative_yaw_deg;
    int16_t target_rpm[4];
    int16_t actual_rpm[4];
    uint32_t update_tick;
} usart1_chassis_motion_t;
const volatile usart1_chassis_motion_t *get_usart1_chassis_motion(void);

/* Independent diagnostic sample; never used as a control-link heartbeat. */
typedef struct
{
    float yaw_angle_deg;
    float pitch_angle_deg;
    float roll_angle_deg;
    float yaw_rate_rpm;
    uint32_t source_tick;
    uint32_t update_tick;
    uint8_t ready;
    uint8_t error;
    uint8_t sequence;
} usart1_chassis_imu_t;
const volatile usart1_chassis_imu_t *get_usart1_chassis_imu(void);

void usart1_remote_control_init(void);
void USART1_IRQHandler(void);
const RC_ctrl_t *get_usart1_remote_control_point(void);
uint8_t usart1_RC_data_is_error(void);
void usart1_slove_RC_lost(void);
void usart1_slove_data_error(void);
float get_rad_from_chassis(uint8_t* data);

/* USART1 底盘状态帧解析接口 */
uint8_t usart1_parse_chassis_status_frame(const uint8_t *frame, uint16_t len);
uint16_t usart1_parse_chassis_status_stream(const uint8_t *data, uint16_t len);
void usart1_reset_chassis_status_stream_parser(void);
float get_usart1_chassis_gyro_speed_rpm(void);
uint8_t get_usart1_small_gyro_enable(void);
uint8_t get_usart1_chassis_imu_ready(void);
uint8_t get_usart1_chassis_enabled(void);
uint16_t get_usart1_shooter_17mm_heat(void);
uint16_t get_usart1_shooter_17mm_heat_limit(void);
uint16_t get_usart1_shooter_barrel_cooling_value(void);
uint32_t get_usart1_chassis_status_update_tick(void);
const volatile usart1_chassis_status_t *get_usart1_chassis_status(void);
void usart1_send_gimbal_status(const usart1_gimbal_tx_status_t *status);

#define TUCHUAN_RX_CHUNK_SIZE  1u

#define TUCHUAN_KEY_W      (1u << 0)
#define TUCHUAN_KEY_S      (1u << 1)
#define TUCHUAN_KEY_A      (1u << 2)
#define TUCHUAN_KEY_D      (1u << 3)
#define TUCHUAN_KEY_SHIFT  (1u << 4)
#define TUCHUAN_KEY_CTRL   (1u << 5)
#define TUCHUAN_KEY_Q      (1u << 6)
#define TUCHUAN_KEY_E      (1u << 7)
#define TUCHUAN_KEY_R      (1u << 8)
#define TUCHUAN_KEY_F      (1u << 9)
#define TUCHUAN_KEY_G      (1u << 10)
#define TUCHUAN_KEY_Z      (1u << 11)
#define TUCHUAN_KEY_X      (1u << 12)
#define TUCHUAN_KEY_C      (1u << 13)
#define TUCHUAN_KEY_V      (1u << 14)
#define TUCHUAN_KEY_B      (1u << 15)

typedef struct
{
  uint16_t channel_0;// 1000-2000 对应遥控器摇杆的五个档位
  uint16_t channel_1;// 1000-2000 对应遥控器摇杆的五个档位
  uint16_t channel_2;
  uint16_t channel_3;
  uint16_t dial;
  uint8_t mode_switch;
  uint8_t pause_button;
  uint8_t custom_button_l;
  uint8_t custom_button_r;
  uint8_t trigger_down;
  int16_t mouse_x;   //鼠标x
  int16_t mouse_y;   //鼠标y
  int16_t mouse_z;//
  uint8_t left_button_down;//鼠标左键，0表示未按下，1表示按下
  uint8_t right_button_down;//鼠标右键，0表示未按下，1表示按下
  uint8_t middle_button_down;//鼠标中键，0表示未按下，1表示按下
  uint16_t keyboard_value;//键盘值，每一位代表一个键，1表示按下，0表示未按下，具体按键定义见 TUCHUAN_KEY_XXX 宏定义
  uint16_t reserved;
  uint32_t update_tick;
  uint8_t updated;//
} usart6_tuchuan_remote_control_t;

typedef struct
{
  uint8_t data[30];
  uint8_t data_len;
  uint32_t update_tick;
  uint8_t updated;
} usart6_tuchuan_custom_data_t;

typedef struct
{
  uint32_t rx_irq_cnt;
  uint32_t frame_ok_cnt;
  uint32_t frame_short_ok_cnt;
  uint32_t frame_fail_cnt;
  uint32_t rx_start_ok_cnt;
  uint32_t rx_start_fail_cnt;
  uint32_t uart_err_cnt;
  uint32_t uart_last_err_code;
  uint32_t hdr1_cnt;
  uint32_t hdr2_cnt;
} usart6_tuchuan_debug_stat_t;

void usart6_tuchuan_init(void);
void usart6_tuchuan_rx_callback(void);
void usart6_tuchuan_error_callback(uint32_t error_code);
const usart6_tuchuan_remote_control_t *get_usart6_tuchuan_remote_control_point(void);
const usart6_tuchuan_custom_data_t *get_usart6_tuchuan_custom_data_point(void);
uint16_t get_usart6_tuchuan_last_cmd_id(void);
uint16_t get_usart6_tuchuan_keyboard_value(void);
void clear_usart6_tuchuan_remote_control_update_flag(void);
void clear_usart6_tuchuan_custom_data_update_flag(void);
void usart6_tuchuan_debug_enable(uint8_t enable);
void usart6_tuchuan_usb_debug_flush(void);
const usart6_tuchuan_debug_stat_t *get_usart6_tuchuan_debug_stat_point(void);

/* Low-rate text output for standalone motor testing. */
void usart6_debug_printf(const char *format, ...);
void usart6_debug_poll(void);

#endif /* __USART6_REMOTE_CONTROL_H */










