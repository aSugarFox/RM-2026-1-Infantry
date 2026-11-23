/**
  ******************************************************************************
  * @attention
  * @file       Control_Ghassis.c/h
  * @brief      控制云台电机实现函数，包括初始化，PID计算函数，
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     11-21-2025      zxy             1. test
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ******************************************************************************
  */
 #ifndef CONTROL_GHASSIS_H
 #define CONTROL_GHASSIS_H
 
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
#include "user_lib.h"
#include <stdint.h>
#include <math.h>
#include "stm32f4xx_it.h"
#include "Control_Gimbal.h"
#include "usart6_remote_control.h"
// ------------------- 底盘参数宏定义 -------------------

// 任务控制周期，主循环中需要以该周期调用底盘任务
#define CHASSIS_CONTROL_TIME_MS     2     // 2ms
#define CHASSIS_CONTROL_TIME        0.002f
#define CHASSIS_CONTROL_FREQUENCE   500.0f

// 任务初始化等待时间 (FreeRTOS移除后，此宏不再使用，但保留以防万一)
#define CHASSIS_TASK_INIT_TIME      100

// 遥控器通道
#define CHASSIS_X_CHANNEL           3
#define CHASSIS_Y_CHANNEL           2
#define CHASSIS_WZ_CHANNEL          0
// #define CHASSIS_MODE_CHANNEL        1

// 遥控器灵敏度
#define CHASSIS_VX_RC_SEN           0.006f
#define CHASSIS_VY_RC_SEN           0.005f
#define CHASSIS_WZ_RC_SEN           0.01f
#define CHASSIS_RC_DEADLINE         10

// 加速度（一阶低通滤波系数，接近于1/Tau， Tau = 1/k）
#define CHASSIS_ACCEL_X_NUM         0.1666666667f
#define CHASSIS_ACCEL_Y_NUM         0.3333333333f

// 电机 RPM 到 m/s 的转换系数
#define M3508_MOTOR_RPM_TO_VECTOR   0.000415809748903494517209f
#define CHASSIS_MOTOR_RPM_TO_VECTOR_SEN M3508_MOTOR_RPM_TO_VECTOR

// 全向轮结构参数
#define MOTOR_DISTANCE_TO_CENTER    0.2f  // 轮子到中心的距离 D (m)

// 速度限制
#define MAX_WHEEL_SPEED             4.0f  // 单个轮子最大线速度 m/s
#define NORMAL_MAX_CHASSIS_SPEED_X  2.0f  // 底盘运动过程最大前进速度
#define NORMAL_MAX_CHASSIS_SPEED_Y  1.5f  // 底盘运动过程最大平移速度
#define MAX_MOTOR_CAN_CURRENT       16000.0f // M3508最大CAN电流值

// 按键定义 (假设在遥控器或键鼠处理中定义)
#define CHASSIS_FRONT_KEY           KEY_PRESSED_OFFSET_W
#define CHASSIS_BACK_KEY            KEY_PRESSED_OFFSET_S
#define CHASSIS_LEFT_KEY            KEY_PRESSED_OFFSET_A
#define CHASSIS_RIGHT_KEY           KEY_PRESSED_OFFSET_D


// ------------------- PID 参数 -------------------

// 底盘电机速度环PID (M3508/M3505)
#define M3505_MOTOR_SPEED_PID_KP    15000.0f
#define M3505_MOTOR_SPEED_PID_KI    10.0f
#define M3505_MOTOR_SPEED_PID_KD    0.0f
#define M3505_MOTOR_SPEED_PID_MAX_OUT MAX_MOTOR_CAN_CURRENT
#define M3505_MOTOR_SPEED_PID_MAX_IOUT 2000.0f

// 底盘旋转跟随PID
#define CHASSIS_FOLLOW_GIMBAL_PID_KP 40.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_KI 0.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_KD 0.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT 6.0f // rad/s
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT 0.2f


#define M3508_RPM_TO_VECTOR   0.00795870125333f// 0.152*3.1415926/60
// ------------------- 结构体和枚举定义 -------------------

// 底盘控制模式
typedef enum
{
    CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW,   // 底盘会跟随云台相对角度 (云台坐标系)
    CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW,  // 底盘由底盘角度控制闭环 (底盘坐标系，角度闭环)
    CHASSIS_VECTOR_NO_FOLLOW_YAW,       // 底盘由旋转速度控制 (底盘坐标系，速度控制)
    CHASSIS_VECTOR_RAW,                 // 无控制 (直接发送电流值)
} chassis_mode_e;

// 单个底盘电机数据
typedef struct
{
    const motor_measure_t *chassis_motor_measure;
    fp32 accel;
    fp32 speed;
    fp32 speed_set;
    int16_t give_current;
} chassis_motor_t;

// 底盘运动控制数据
typedef struct
{
    const RC_ctrl_t *chassis_RC;                // 遥控器指针
    // const gimbal_motor_t *chassis_yaw_motor;    // Yaw云台电机指针 (相对角度)
    // const gimbal_motor_t *chassis_pitch_motor;  // Pitch云台电机指针 (相对角度)
    // const fp32 *chassis_INS_angle;              // 陀螺仪欧拉角指针 [yaw, pitch, roll]

    chassis_mode_e chassis_mode;                // 当前底盘控制模式
    chassis_mode_e last_chassis_mode;           // 上次底盘控制模式

    chassis_motor_t motor_chassis[4];           // 4个底盘电机数据
    pid_type_def motor_speed_pid[4];            // 底盘电机速度PID
    pid_type_def chassis_angle_pid;             // 底盘跟随角度PID

    first_order_filter_type_t chassis_cmd_slow_set_vx; // V_x 一阶低通滤波
    first_order_filter_type_t chassis_cmd_slow_set_vy; // V_y 一阶低通滤波

    fp32 vx;         // 实际速度 前进 (m/s)
    fp32 vy;         // 实际速度 左右 (m/s)
    fp32 wz;         // 实际角速度 旋转 (rad/s)
    fp32 vx_set;     // 设定速度 前进 (m/s)
    fp32 vy_set;     // 设定速度 左右 (m/s)
    fp32 wz_set;     // 设定角速度 旋转 (rad/s)

    fp32 chassis_relative_angle;         // 底盘与云台的相对角度 (rad)
    fp32 chassis_relative_angle_set;     // 相对云台的目标角度 (rad)
    fp32 chassis_yaw_set;                // 底盘绝对目标Yaw角度 (rad)

    fp32 vx_max_speed; // 前进最大速度
    fp32 vx_min_speed; // 后退最小速度
    fp32 vy_max_speed; // 左最大速度
    fp32 vy_min_speed; // 右最小速度

    fp32 chassis_yaw;   // 叠加云台/INS后的Yaw角度 (rad)
    fp32 chassis_pitch; // 叠加云台/INS后的Pitch角度 (rad)
    fp32 chassis_roll;  // 陀螺仪Roll角度 (rad)

} chassis_move_t;

typedef struct
{
    int16_t chassis_motor_1;
    int16_t chassis_motor_2;
    int16_t chassis_motor_3;    
    int16_t chassis_motor_4;
} chassis_output_t;

// ------------------- 外部函数声明 -------------------

// 外部全局变量
extern chassis_move_t chassis_move;
extern chassis_output_t chassisoutput;


/**
 * @brief 底盘控制任务函数。此函数应被周期性(2ms)调用。
 * @param None
 * @retval None
 */
extern void chassis_task_loop(void);

/**
 * @brief 底盘初始化函数
 * @param chassis_move_init: "chassis_move"变量指针.
 * @retval None
 */
extern void chassis_init(chassis_move_t *chassis_move_init);

// 需要由外部实现的函数 (例如在 behaviour.c 中)
// extern void chassis_behaviour_mode_set(chassis_move_t *chassis_move_mode);
// extern void chassis_behaviour_control_set(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set, chassis_move_t *chassis_move_control);
// extern void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);

/**
 * @brief       拨弹控制函数
 * @note
 * @param[in]      bodan_set:   拨弹目标速度
 * @retval         none
 */
extern void RC_Control_Fric(int fric_1_set, int fric_2_set);

/**
 * @brief 底盘控制函数
 * @param mode: chassis_mode_e 枚举类型 什么模式
 * @retval None
 */
extern void RC_Control_Chassis(chassis_mode_e mode);
extern void Tuchuan_Keyboard_Control_Chassis(chassis_mode_e mode);

/**
 * @brief 清除底盘输出数据
 * @param None
 * @retval None
 */
extern void chassis_output_clear(void);

/**
 * @brief 发送底盘输出数据
 * @param None
 * @retval None
 */
extern void chassis_output_send(void);

#endif 


