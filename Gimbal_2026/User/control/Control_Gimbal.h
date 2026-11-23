#ifndef CONTROL_GIMBAL_H
#define CONTROL_GIMBAL_H

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

#define MOTOR_ECD_RANGE              8191.0f
#define ANGLE_360                    360.0f
#define ANGLE_180                    180.0f
#define CONTROL_SENSITIVITY          0.05f
#define VISION_CONTROL_SENSITIVITY   0.1f

/* Pitch encoder limits */
#define PITCH_ENCODER_MIN            670
#define PITCH_ENCODER_MAX            2100
#define PITCH_ENCODER_MARGIN         30
#define PITCH_SAFE_MIN               (PITCH_ENCODER_MIN + PITCH_ENCODER_MARGIN)
#define PITCH_SAFE_MAX               (PITCH_ENCODER_MAX - PITCH_ENCODER_MARGIN)

#define ENCODER_TO_ANGLE(ecd)        ((ecd) * 360.0f / 8192.0f)
#define PITCH_ANGLE_MIN              ENCODER_TO_ANGLE(PITCH_ENCODER_MIN)
#define PITCH_ANGLE_MAX              ENCODER_TO_ANGLE(PITCH_ENCODER_MAX)

//视觉的都没用上可以先不看
/* Vision params */
#define VISION_X_MID                 320     // 视觉画面 X 中心基准；相机分辨率或裁剪区域变化时改它
#define VISION_Y_MID                 370     // 视觉画面 Y 中心基准；相机安装高度或图像 ROI 变化时改它
#define VISION_X_DEADZONE            10      // 视觉 Yaw 死区；准星附近左右微抖就增大，中心附近吸附太慢就减小
#define VISION_Y_DEADZONE            5       // 视觉 Pitch 死区；上下微抖就增大，上下跟随迟钝就减小
#define VISION_TRACK_GAIN            0.5f    // 视觉总跟踪增益；整体跟踪偏慢增大，整体过冲/振荡减小
#define VISION_SMOOTHING             0.40f   // 视觉误差平滑系数；画面噪声大减小，想更跟手增大
#define VISION_MAX_ERROR             200.0f  // 视觉输入误差限幅；丢目标时跳变大减小，大偏差追不上增大
#define VISION_X_TO_YAW_ERR_GAIN     (VISION_TRACK_GAIN * 0.13f) // X 像素误差转 Yaw 角误差比例；左右跟随慢增大，摆头过大减小
#define VISION_Y_TO_PITCH_ERR_GAIN   (VISION_TRACK_GAIN * 0.08f) // Y 像素误差转 Pitch 角误差比例；上下跟随慢增大，抬头点头过冲减小
#define VISION_XY_YAW_SPEED_DAMP     0.08f   // 视觉 Yaw 速度阻尼；左右回摆增大，制动太硬或拖沓减小
#define VISION_XY_PITCH_SPEED_DAMP   0.10f   // 视觉 Pitch 速度阻尼；上下回弹增大，俯仰发闷减小
#define VISION_XY_YAW_MAX_ERR_DEG    180.0f  // 视觉 Yaw 角误差限幅；大偏差抢控制太猛减小，远距离拉不过来增大
#define VISION_XY_PITCH_MAX_ERR_DEG  30.0f   // 视觉 Pitch 角误差限幅；俯仰大步进太猛减小，抬头低头跟不上增大
#define VISION_XY_PITCH_GRAVITY_FF_BASE_CURRENT   120.0f // Pitch 重力前馈基值；抬头无力或下垂时优先增大
#define VISION_XY_PITCH_GRAVITY_FF_GAIN_CURRENT   480.0f // Pitch 重力前馈斜率；角度越大越托不住时增大，抬头过冲减小
#define VISION_XY_CMD_DELTA_LIMIT_YAW            12.0f  // 正常模式单周期 Yaw 命令变化限幅；视觉抖动减小，响应慢增大
#define VISION_XY_CMD_DELTA_LIMIT_PITCH          10.0f  // 正常模式单周期 Pitch 命令变化限幅；上下抽动减小，响应慢增大
#define VISION_XY_CMD_DELTA_LIMIT_YAW_FALLBACK   6.0f   // fallback 模式 Yaw 变化限幅；fallback 时发抖减小，太保守增大
#define VISION_XY_CMD_DELTA_LIMIT_PITCH_FALLBACK 5.0f   // fallback 模式 Pitch 变化限幅；fallback 上下抖减小，太慢增大
#define VISION_XY_CMD_DELTA_LIMIT_INVALID        8.0f   // 无效目标/切目标时的命令限幅；切换跳变大减小，恢复太慢增大
#define VISION_XY_FALLBACK_SCALE                 0.45f  // fallback 输出缩放；备用目标太激进减小，太软增大

/* Small-gyro auto-aim feedforward params */
#define VISION_SMALL_GYRO_YAW_FF_GAIN        1.0f   // 小陀螺视觉前馈比例；开小陀螺时准星跟不上底盘增大，互相打架/过补偿减小
#define VISION_SMALL_GYRO_LOCK_FF_DEADZONE_RPM 0.5f // 小陀螺前馈锁定死区；目标居中还在轻转就增大，锁定太迟钝减小

/* Auto-aim shared lock params */
#define VISION_ANGLE_STATIC_LOCK_YAW_DEG    0.12f  // Yaw 静止锁定角度阈值；居中仍抖增大，停稳后还慢慢修正减小
#define VISION_ANGLE_STATIC_LOCK_PITCH_DEG  0.08f  // Pitch 静止锁定角度阈值；上下微抖增大，锁定太松减小
#define VISION_ANGLE_STATIC_LOCK_SPEED_RPM  8      // 静止锁定速度阈值；低速抖动时增大，停稳判定太晚减小

#define CHANNEL_DEADZONE                10
#define VISION_DEADZONE_X               10
#define VISION_DEADZONE_Y               5

//图传传回的鼠标键盘数据宏定义
/* Tuchuan mouse params */
#define TUCHUAN_MOUSE_DEADZONE_X        10      // 鼠标 X 死区；手离鼠标仍左右抖动就增大，轻推不跟手就减小
#define TUCHUAN_MOUSE_DEADZONE_Y        8      // 鼠标 Y 死区；手离鼠标仍上下抖动就增大，轻推不跟手就减小
#define TUCHUAN_MOUSE_YAW_GAIN          0.0062f // 鼠标 X 到 Yaw 角度步进增益；左右转头慢增大，过于灵敏减小
#define TUCHUAN_MOUSE_PITCH_GAIN        0.20f   // 鼠标 Y 到 Pitch 角度步进增益；上下抬头慢增大，容易点头减小
#define TUCHUAN_MOUSE_YAW_SPEED_BOOST   0.008f  // 鼠标快速横甩附加加速系数；大幅甩枪不够快增大，快甩过冲减小
#define TUCHUAN_MOUSE_PITCH_SPEED_BOOST   0.008f  // 鼠标快速横甩附加加速系数；大幅甩枪不够快增大，快甩过冲减小

#define TUCHUAN_MOUSE_YAW_SPEED_SCALE_MAX 1.9f  // 鼠标横甩最大加速倍数；极速甩枪不够快增大，控不住减小
#define TUCHUAN_MOUSE_X_MAX_STEP_DEG    2.9f    // 单周期最大 Yaw 步进；大动作太冲减小，极速转向不够快增大
#define TUCHUAN_MOUSE_Y_MAX_STEP_DEG    0.70f   // 单周期最大 Pitch 步进；上下跳动大减小，抬头低头不够快增大
#define TUCHUAN_MOUSE_X_DECOUPLE_THRESHOLD 80   // X 主导解耦阈值；横移时带出 Pitch 就增大，横向纯度太高导致上下难带出则减小
#define TUCHUAN_MOUSE_Y_DECOUPLE_THRESHOLD 80   // Y 主导解耦阈值；纵移时带出 Yaw 就增大，纵向纯度太高导致左右难带出则减小
#define TUCHUAN_MOUSE_XY_DECOUPLE_RATIO 2       // XY 解耦比例；斜向容易串轴就增大，斜向操作太难做减小
#define TUCHUAN_GYRO_YAW_SPEED_DPS      90.0f  // 纯小陀螺基础自转角速度；想整体转更快增大，太晕/太猛减小
 
typedef enum {
    GIMBAL_MANUAL = 0,
    GIMBAL_POSITION_HOLD,
    GIMBAL_SAFE,
    GIMBAL_GYRO,
} gimbal_mode_t;

typedef struct {
    gimbal_mode_t mode;
    float hold_angle;
    uint32_t hold_start_time;
    float hold_kp;
    float max_hold_current;
} position_hold_t;

typedef struct {
    int16_t yaw;
    int16_t pitch;
    int16_t trigger;
    int16_t reserved;
} gimbal_output_t;

extern gimbal_output_t gimbal_out;
extern float chassis_rad;

extern pid_type_def yaw_angle_pid;
extern pid_type_def yaw_speed_pid;
extern pid_type_def picth_angle_pid;
extern pid_type_def pitch_speed_pid;

int16_t pitch_limit_protection(int16_t pitch_current,
                               const motor_measure_t *pitch_motor,
                               float pitch_angle_error);

void Control_Gimbal_Init(void);
void RC_Control_Gimbal(void);
void Vision_Control_Gimbal(int16_t x_offset,
                           int16_t y_offset,
                           uint8_t target_valid,
                           uint8_t control_far_active,
                           uint8_t fallback_active);
void Vision_Control_Gimbal_SmallGyro(int16_t x_offset,
                                     int16_t y_offset,
                                     uint8_t target_valid,
                                     uint8_t control_far_active,
                                     uint8_t fallback_active);
void Vision_Control_Gimbal_AngleError_Reset(void);
void Tuchuan_Mouse_Control_Gimbal(void);
void Tuchuan_Mouse_Control_Gimbal_Gyro(void);
void Tuchuan_Mouse_Control_Gimbal_Gyro_ChassisComp(void);
void request_tuchuan_mouse_gimbal_resync(void);

void RC_Control_Bodan(int16_t bodan_set);
void Test_Pitch_Speed_Control(int16_t speed_set);
uint8_t check_pitch_limit_status(const motor_measure_t *pitch_motor);
void set_gimbal_mode(gimbal_mode_t mode);
float get_pitch_angle(const motor_measure_t *motor);
float encoder_to_angle(int16_t encoder);
void Gimbal_ba(int16_t speed_set);
void set_gimbal_yaw_mode(gimbal_mode_t mode);
void RC_Control_Bodan_Danfa(void);
void Gimbal_Angle_Init(float init_target_angle);
void Gimbal_ecd_angle(void);
float Get_Gimbal_Angle(void);
void gimbal_output_clear(void);
void gimbal_output_send(void);
float get_tuchuan_gimbal_gyro_comp_speed(void);
uint8_t get_tuchuan_gimbal_gyro_enabled(void);
void reset_tuchuan_gimbal_gyro_link(void);

#endif
