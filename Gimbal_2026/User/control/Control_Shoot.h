/**
  ******************************************************************************
  * @attention
  * @file       Control_Shoot.c/h
  * @brief      控制射击，包括初始化，射击状态机，
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     1-10-2026      zxy             1. test
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ******************************************************************************
  */
#ifndef CONTROL_SHOOT_H
#define CONTROL_SHOOT_H

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
#include "Control_Gimbal.h"

#define MAX_MOTOR_CAN_CURRENT       16000.0f // M3508最大CAN电流值
/* 常量定义 */
#define FRIC_TARGET_SPEED          5000    // 摩擦轮目标速度
#define TRIGGER_TARGET_SPEED       2000    // 拨弹轮目标速度
#define TRIGGER_REVERSE_SPEED      -1500   // 拨弹轮反转速度
#define FRIC_READY_TIME             200     // 摩擦轮准备时间(ms)
#define STUCK_DETECT_TIME          500     // 卡弹检测时间(ms)
#define STUCK_SPEED_THRESHOLD      100     // 卡弹速度阈值(RPM)
#define SINGLE_SHOOT_TIME          200     // 单发持续时间(ms)

/* 状态定义 */
typedef enum {
    SHOOT_MODE_DISABLE = 0,      // 失能状态
    SHOOT_MODE_FRIC_READY,       // 摩擦轮准备
    SHOOT_MODE_READY,            // 准备就绪
    SHOOT_MODE_SINGLE_SHOOT,     // 单发
    SHOOT_MODE_CONTINUOUS,       // 连发
    SHOOT_MODE_STUCK_RECOVER,    // 卡弹恢复
    SHOOT_MODE_ERROR             // 错误状态
} shoot_mode_t;

/* 2006电机参数 */
#define MOTOR_2006_REDUCTION_RATIO     36      // 减速比 36:1
#define MOTOR_2006_ENCODER_RESOLUTION  8192    // 编码器分辨率 8192线
#define MOTOR_2006_OUTPUT_RESOLUTION   (MOTOR_2006_ENCODER_RESOLUTION * MOTOR_2006_REDUCTION_RATIO)  // 输出轴分辨率
#define MOTOR_ECD_TO_ANGLE             (360.0f / MOTOR_2006_ENCODER_RESOLUTION)  // 电机轴角度转换
#define MOTOR_OUTPUT_TO_ANGLE          (360.0f / MOTOR_2006_OUTPUT_RESOLUTION)   // 输出轴角度转换
#define HALF_ECD_RANGE                 (MOTOR_2006_ENCODER_RESOLUTION / 2)  // 半编码器范围
#define ECD_RANGE                      MOTOR_2006_ENCODER_RESOLUTION        // 编码器范围

/* 控制模式 */
typedef enum {
    MOTOR_SPEED_CONTROL = 0,     // 速度控制
    MOTOR_POSITION_CONTROL,      // 位置控制（相对角度）
    MOTOR_ABSOLUTE_POSITION,     // 绝对位置控制
    MOTOR_ROTATE_ONE_CIRCLE      // 旋转一圈
} motor_control_mode_t;

/* 电机控制结构体 */
typedef struct {
    motor_control_mode_t mode;   // 控制模式
    float target_speed;          // 目标速度（RPM）
    float target_angle;          // 目标角度（度）
    float current_angle;         // 当前角度（度）
    int32_t total_ecd;           // 总编码器值（考虑溢出）
    int32_t start_ecd;           // 开始时的编码器值
    uint8_t is_running;          // 是否正在运行
    uint8_t direction;           // 方向：0-正转，1-反转
    uint32_t move_time;          // 移动时间
} motor_2006_controller_t;


/* 按键状态 */
typedef enum {
    TRIGGER_OFF = 0,             // 未触发
    TRIGGER_ON                   // 触发
} trigger_state_t;

/* 射击控制结构体 */
typedef struct {
    shoot_mode_t mode;           // 当前模式
    shoot_mode_t last_mode;      // 上次模式
    
    /* 摩擦轮控制 */
    int16_t fric1_target;        // 摩擦轮1目标速度
    int16_t fric2_target;        // 摩擦轮2目标速度
    int16_t fric1_current;       // 摩擦轮1当前速度
    int16_t fric2_current;       // 摩擦轮2当前速度
    
    /* 拨弹轮控制 */
    int16_t trigger_target;      // 拨弹轮目标速度
    int16_t trigger_current;     // 拨弹轮当前速度
    uint32_t bullet_count;       // 子弹计数
    
    /* 状态控制 */
    trigger_state_t trigger_state;   // 扳机状态
    uint32_t shoot_timer;            // 射击计时器
    uint32_t fric_ready_timer;       // 摩擦轮准备计时
    uint32_t stuck_timer;            // 卡弹计时
    
    /* 安全参数 */
    uint8_t is_fric_ready;           // 摩擦轮是否就绪
    uint8_t is_jammed;               // 是否卡弹
    int16_t trigger_speed_limit;     // 拨弹轮速度限制
    
    /* 遥控器输入 */
    int16_t rc_s_left;               // 左拨杆
    int16_t rc_s_right;              // 右拨杆
    int16_t rc_sw;                   // 开关状态
    
} shoot_control_t;






extern gimbal_output_t gimbal_out;

/* PID参数声明 */
extern pid_type_def fric_speed_1;
extern pid_type_def fric_speed_2;
extern pid_type_def trigger_speed;

void RC_Control_Bodan(int16_t bodan_set);
void RC_Control_Fric(int fric_1_set, int fric_2_set);
void motor_rotate_one_circle_speed(void);
void motor_rotate_one_circle_position(void);

#endif
