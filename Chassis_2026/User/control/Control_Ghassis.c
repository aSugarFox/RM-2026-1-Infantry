/**
  ******************************************************************************
  * @attention
  * @file       Control_Ghassis.c/h
  * @brief      控制底盘电机实现函数，包括初始化，PID计算函数，
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     11-21-2025      zxy             1. test
  * 底盘控制由遥控或者键盘，可以选择跟随云台yaw或者原地旋转
  * 用陀螺仪校准底盘方向，便于小陀螺和跟随云台yaw模式的稳定性
  * 建议后面键盘控制时加上梯形加减速，防止急停急启晃头。
  * 还有开启关闭小陀螺时可以加个斜坡加减速，防止突然启动/停止云台响应跟不上
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ******************************************************************************
  */
 #include "Control_Ghassis.h"
#include "infantry_config.h"
// ------------------- 宏定义 -------------------

// 遥控器死区限制
#define rc_deadband_limit(input, output, dealine)        \
    {                                                    \
        if ((input) > (dealine) || (input) < -(dealine)) \
        {                                                \
            (output) = (input);                          \
        }                                                \
        else                                             \
        {                                                \
            (output) = 0;                                \
        }                                                \
    }

// 全向轮角度 (弧度)
// 假设轮子编号顺序为 0:前左, 1:前右, 2:后右, 3:后左
// 角度定义: 相对于底盘X轴 (向前)
#define OMNI_WHEEL_ANGLE_0  2.356194490192345f    // 135° (3*pi/4)
#define OMNI_WHEEL_ANGLE_1  0.7853981633974483f   // 45° (pi/4)
#define OMNI_WHEEL_ANGLE_2 -0.7853981633974483f   // -45° (-pi/4)
#define OMNI_WHEEL_ANGLE_3 -2.356194490192345f    // -135° (-3*pi/4)

// 全向轮 $\cos$ 和 $\sin$ 值
// $\cos(45^\circ) = \sin(45^\circ) = \sqrt{2}/2 \approx 0.7071$
#define SIN_45          0.7071067811865476f
#define COS_45          0.7071067811865476f

// 键盘平移速度档位（Keyboard_Control_Chassis 使用）
// CHASSIS_KEYBOARD_SPEED_MID：默认 W/A/S/D 速度，建议先调这个。每次改 200~300。
// CHASSIS_KEYBOARD_SPEED_HIGH：SHIFT 档，通常比 MID 高 500~1500。
// CHASSIS_KEYBOARD_SPEED_BOOST：低血量加速档，通常比 HIGH 再高 300~800。
// CHASSIS_KEYBOARD_SPEED_LOW：CTRL 微调档，建议每次改 100~300。
// 规律：调大更快更冲，调小更稳、更容易刹住。
#define CHASSIS_KEYBOARD_SPEED_MID          3000
#define CHASSIS_KEYBOARD_SPEED_HIGH         3500
#define CHASSIS_KEYBOARD_SPEED_BOOST        3000
#define CHASSIS_KEYBOARD_SPEED_LOW          2000

// 键盘旋转/小陀螺速度档位（Keyboard_Control_Chassis 使用）
// 注意：当前小陀螺目标转速是直接跳变的，还没有单独做斜坡加减速。
// CHASSIS_KEYBOARD_WZ_SPEED：默认 Q/E 旋转速度，也是普通小陀螺基础值。每次建议改 300~500。
// CHASSIS_SMALL_GYRO_SPEED_HIGH：SHIFT 小陀螺速度，通常比默认高 500~1500。
// CHASSIS_SMALL_GYRO_SPEED_BOOST：低血量加速小陀螺，建议每次只加 300~500。
// CHASSIS_SMALL_GYRO_SPEED_LOW：CTRL 小陀螺微调速度，适合需要更稳的时候。
// 规律：调大旋转更猛，但更容易抖和甩尾；调小更稳，但小陀螺压制感会变弱。
#define CHASSIS_KEYBOARD_WZ_SPEED           8000
#define CHASSIS_SMALL_GYRO_SPEED_HIGH       10000
#define CHASSIS_SMALL_GYRO_SPEED_BOOST      8000
#define CHASSIS_SMALL_GYRO_SPEED_LOW        5000

// 遥控器底盘速度倍率（RC_Control_Chassis 使用）
// CHASSIS_RC_VX_SET_SCALE：遥控器前后速度倍率，建议每次改 0.5f~1.0f。
// CHASSIS_RC_VY_SET_SCALE：遥控器左右速度倍率，建议每次改 0.5f~1.0f。
// CHASSIS_RC_Z_SET_SCALE：跟随云台时的旋转倍率，建议每次改 0.5f~1.0f。
// CHASSIS_RC_WZ_SET_SCALE：原地旋转倍率，建议每次改 1.0f~2.0f。
// 规律：调大摇杆更灵，调小更柔和。
#define CHASSIS_RC_VX_SET_SCALE             INFANTRY_CHASSIS_VX_SCALE
#define CHASSIS_RC_VY_SET_SCALE             INFANTRY_CHASSIS_VY_SCALE
#define CHASSIS_RC_Z_SET_SCALE              10.0f
#define CHASSIS_RC_WZ_SET_SCALE             20.0f

// 云台陀螺仪补偿参数
// CHASSIS_GYRO_FRAME_TIMEOUT_MS：云台补偿数据超时时间，常用 50~150ms。
//   调大：更不容易丢补偿，但可能吃到旧数据；调小：更安全，但补偿更容易断。
// CHASSIS_GYRO_COMP_SIGN：补偿方向，只能填 1.0f 或 -1.0f。
//   如果开补偿后抖得更厉害，就把符号翻过来。
// CHASSIS_MOTOR_GEAR_RATIO：真实传动比，只有硬件齿比变了才改。
// CHASSIS_GYRO_RPM_TO_Z_SET：由齿比和轮半径推出来的换算量，一般不直接手调。
#define CHASSIS_GYRO_FRAME_TIMEOUT_MS       100u
#define CHASSIS_GYRO_COMP_SIGN              1.0f
#define CHASSIS_MOTOR_GEAR_RATIO            48.125f
#define CHASSIS_GYRO_RPM_TO_Z_SET (CHASSIS_MOTOR_GEAR_RATIO / WHEEL_RADIUS)

// ------------------- 静态函数声明 -------------------

static void chassis_set_mode(chassis_move_t *chassis_move_mode);
static void chassis_mode_change_control_transit(chassis_move_t *chassis_move_transit);
static void chassis_feedback_update(chassis_move_t *chassis_move_update);
static void chassis_set_contorl(chassis_move_t *chassis_move_control);
static void chassis_control_loop(chassis_move_t *chassis_move_control_loop);
static void chassis_vector_to_omni_wheel_speed(const fp32 vx_set, const fp32 vy_set, const fp32 wz_set, fp32 wheel_speed[4]);
static int16_t get_gimbal_small_gyro_compensation(void);
static void chassis_small_gyro_update_from_keyboard(uint16_t key_value);
static float chassis_small_gyro_get_speed_from_key(uint16_t key_value);

// ------------------- 全局变量 -------------------

chassis_move_t chassis_move;
chassis_output_t chassisoutput;

// ------------------- 任务函数 -------------------

/**
 * @brief 底盘控制任务主循环函数。此函数应被外部以 2ms 周期调用。
 * @param None
 * @retval None
 */
void chassis_task_loop(void)
{
    // 如果电机或遥控器全部离线，则发送零电流
    // if (toe_is_error(DBUS_TOE) || (toe_is_error(CHASSIS_MOTOR1_TOE) && toe_is_error(CHASSIS_MOTOR2_TOE) && toe_is_error(CHASSIS_MOTOR3_TOE) && toe_is_error(CHASSIS_MOTOR4_TOE)))
    // {
    //     CAN_cmd_chassis(0, 0, 0, 0);
    //     return;
    // }
    
    // 设置底盘控制模式
    // chassis_behaviour_mode_set(&chassis_move); // 假设此函数在外部实现
    
    // 模式切换数据保存
    // chassis_mode_change_control_transit(&chassis_move);
    
    // 底盘数据更新
    chassis_feedback_update(&chassis_move);
    
    // 底盘控制量设置
    chassis_set_contorl(&chassis_move);
    
    // 底盘控制PID计算
    chassis_control_loop(&chassis_move);

    // 发送控制电流
    CAN_cmd_chassis(chassis_move.motor_chassis[0].give_current, chassis_move.motor_chassis[1].give_current,
                    chassis_move.motor_chassis[2].give_current, chassis_move.motor_chassis[3].give_current);
}

// ------------------- 核心函数实现 -------------------

/**
 * @brief 初始化"chassis_move"变量
 */
void chassis_init(chassis_move_t *chassis_move_init)
{
    if (chassis_move_init == NULL)
    {
        return;
    }

    // PID 参数
    const static fp32 motor_speed_pid[3] = {M3505_MOTOR_SPEED_PID_KP, M3505_MOTOR_SPEED_PID_KI, M3505_MOTOR_SPEED_PID_KD};
    const static fp32 chassis_yaw_pid[3] = {CHASSIS_FOLLOW_GIMBAL_PID_KP, CHASSIS_FOLLOW_GIMBAL_PID_KI, CHASSIS_FOLLOW_GIMBAL_PID_KD};
    
    // 一阶滤波器系数
    const static fp32 chassis_x_order_filter[1] = {CHASSIS_ACCEL_X_NUM};
    const static fp32 chassis_y_order_filter[1] = {CHASSIS_ACCEL_Y_NUM};
    uint8_t i;

    // 底盘开机状态为原始
    chassis_move_init->chassis_mode = CHASSIS_VECTOR_RAW;
    
    // 获取指针
    chassis_move_init->chassis_RC = get_remote_control_point();
    // chassis_move_init->chassis_INS_angle = get_INS_angle_point();
    // chassis_move_init->chassis_yaw_motor = get_yaw_motor_point();
    // chassis_move_init->chassis_pitch_motor = get_pitch_motor_point();
    
    // 初始化电机数据和速度PID
    for (i = 0; i < 4; i++)
    {
        chassis_move_init->motor_chassis[i].chassis_motor_measure = get_chassis_motor_measure_point(i);
        PID_init(&chassis_move_init->motor_speed_pid[i], PID_POSITION, motor_speed_pid, M3505_MOTOR_SPEED_PID_MAX_OUT, M3505_MOTOR_SPEED_PID_MAX_IOUT);
    }
    
    // 初始化角度PID
    PID_init(&chassis_move_init->chassis_angle_pid, PID_POSITION, chassis_yaw_pid, CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT, CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT);
    
    // 初始化一阶低通滤波器
    first_order_filter_init(&chassis_move_init->chassis_cmd_slow_set_vx, CHASSIS_CONTROL_TIME, chassis_x_order_filter);
    first_order_filter_init(&chassis_move_init->chassis_cmd_slow_set_vy, CHASSIS_CONTROL_TIME, chassis_y_order_filter);

    // 设置速度限制
    chassis_move_init->vx_max_speed = NORMAL_MAX_CHASSIS_SPEED_X;
    chassis_move_init->vx_min_speed = -NORMAL_MAX_CHASSIS_SPEED_X;
    chassis_move_init->vy_max_speed = NORMAL_MAX_CHASSIS_SPEED_Y;
    chassis_move_init->vy_min_speed = -NORMAL_MAX_CHASSIS_SPEED_Y;

    // 更新一次数据，确保初始值正确
    chassis_feedback_update(chassis_move_init);
}

/**
 * @brief 设置底盘控制模式 (原逻辑，假设外部实现)
 */
static void chassis_set_mode(chassis_move_t *chassis_move_mode)
{
    if (chassis_move_mode == NULL)
    {
        return;
    }
    // 假设 'chassis_behaviour_mode_set' 在外部实现
    // chassis_behaviour_mode_set(chassis_move_mode);
}

/**
 * @brief 底盘模式改变时，重置或保存参数
 */
static void chassis_mode_change_control_transit(chassis_move_t *chassis_move_transit)
{
    if (chassis_move_transit == NULL || chassis_move_transit->last_chassis_mode == chassis_move_transit->chassis_mode)
    {
        return;
    }

    // 切入跟随云台模式
    if (chassis_move_transit->chassis_mode == CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW)
    {
        chassis_move_transit->chassis_relative_angle_set = 0.0f;
    }
    // 切入跟随底盘角度模式
    else if (chassis_move_transit->chassis_mode == CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW)
    {
        chassis_move_transit->chassis_yaw_set = chassis_move_transit->chassis_yaw;
    }
    // 切入不跟随云台模式 (速度控制)
    else if (chassis_move_transit->chassis_mode == CHASSIS_VECTOR_NO_FOLLOW_YAW)
    {
        chassis_move_transit->chassis_yaw_set = chassis_move_transit->chassis_yaw;
    }
    // Raw 模式切换不需要额外处理
    
    chassis_move_transit->last_chassis_mode = chassis_move_transit->chassis_mode;
}

/**
 * @brief 底盘测量数据更新 (电机速度，欧拉角度，机器人速度)
 */
static void chassis_feedback_update(chassis_move_t *chassis_move_update)
{
    if (chassis_move_update == NULL)
    {
        return;
    }

    uint8_t i = 0;
    for (i = 0; i < 4; i++)
    {
        // 更新电机速度和加速度 (加速度由PID D项近似)
        chassis_move_update->motor_chassis[i].speed = CHASSIS_MOTOR_RPM_TO_VECTOR_SEN * chassis_move_update->motor_chassis[i].chassis_motor_measure->speed_rpm;
        chassis_move_update->motor_chassis[i].accel = chassis_move_update->motor_speed_pid[i].Dbuf[0] * CHASSIS_CONTROL_FREQUENCE;
    }

    // 全向轮 正运动学解算 (简化，仅用于近似反馈)
    // 假设全向轮的轮子轴线与底盘 X/Y 轴呈 45 度角 (与麦轮的轮轴方向相同，但轮片方向不同)
    // 全向轮的正解非常复杂且依赖轮子在圆周上的角度 $\alpha_i$
    // 这里保留原始麦轮正解的简化形式，仅作为近似速度反馈
    fp32 v0, v1, v2, v3;
    v0 = chassis_move_update->motor_chassis[0].speed;
    v1 = chassis_move_update->motor_chassis[1].speed;
    v2 = chassis_move_update->motor_chassis[2].speed;
    v3 = chassis_move_update->motor_chassis[3].speed;

    // $V_x$: 前进速度 (近似)
    chassis_move_update->vx = (-v0 + v1 + v2 - v3) * 0.25f; 
    // $V_y$: 平移速度 (近似)
    chassis_move_update->vy = (-v0 - v1 + v2 + v3) * 0.25f; 
    // $\omega_z$: 旋转角速度 (近似)
    chassis_move_update->wz = (-v0 - v1 - v2 - v3) * 0.25f / MOTOR_DISTANCE_TO_CENTER; 
    
    // // 计算底盘姿态角度
    // chassis_move_update->chassis_yaw = rad_format(*(chassis_move_update->chassis_INS_angle + INS_YAW_ADDRESS_OFFSET) - chassis_move_update->chassis_yaw_motor->relative_angle);
    // chassis_move_update->chassis_pitch = rad_format(*(chassis_move_update->chassis_INS_angle + INS_PITCH_ADDRESS_OFFSET) - chassis_move_update->chassis_pitch_motor->relative_angle);
    // chassis_move_update->chassis_roll = *(chassis_move_update->chassis_INS_angle + INS_ROLL_ADDRESS_OFFSET);
}

/**
 * @brief 根据遥控器通道值和键鼠值，计算纵向和横移速度
 */
void chassis_rc_to_control_vector(fp32 *vx_set, fp32 *vy_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (chassis_move_rc_to_vector == NULL || vx_set == NULL || vy_set == NULL)
    {
        return;
    }
    
    int16_t vx_channel, vy_channel;
    fp32 vx_set_channel, vy_set_channel;
    
    // 遥控器摇杆死区限制
    rc_deadband_limit(chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_X_CHANNEL], vx_channel, CHASSIS_RC_DEADLINE);
    rc_deadband_limit(chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_Y_CHANNEL], vy_channel, CHASSIS_RC_DEADLINE);

    // 遥控器输入转换为速度设定值
    vx_set_channel = vx_channel * CHASSIS_VX_RC_SEN;
    vy_set_channel = vy_channel * -CHASSIS_VY_RC_SEN; // 假设Y轴摇杆方向与Y轴速度方向相反

    // // 键鼠控制覆盖摇杆速度
    // if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_FRONT_KEY)
    // {
    //     vx_set_channel = chassis_move_rc_to_vector->vx_max_speed;
    // }
    // else if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_BACK_KEY)
    // {
    //     vx_set_channel = chassis_move_rc_to_vector->vx_min_speed;
    // }

    // if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_LEFT_KEY)
    // {
    //     vy_set_channel = chassis_move_rc_to_vector->vy_max_speed;
    // }
    // else if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_RIGHT_KEY)
    // {
    //     vy_set_channel = chassis_move_rc_to_vector->vy_min_speed;
    // }

    // 一阶低通滤波平滑设定值
    first_order_filter_cali(&chassis_move_rc_to_vector->chassis_cmd_slow_set_vx, vx_set_channel);
    first_order_filter_cali(&chassis_move_rc_to_vector->chassis_cmd_slow_set_vy, vy_set_channel);
    
    // 停止信号，直接减速到零
    if (fabs(vx_set_channel) < CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN)
    {
        chassis_move_rc_to_vector->chassis_cmd_slow_set_vx.out = 0.0f;
    }

    if (fabs(vy_set_channel) < CHASSIS_RC_DEADLINE * CHASSIS_VY_RC_SEN)
    {
        chassis_move_rc_to_vector->chassis_cmd_slow_set_vy.out = 0.0f;
    }

    *vx_set = chassis_move_rc_to_vector->chassis_cmd_slow_set_vx.out;
    *vy_set = chassis_move_rc_to_vector->chassis_cmd_slow_set_vy.out;
}

/**
 * @brief 设置底盘控制设置值，包括速度和角速度
 */
static void chassis_set_contorl(chassis_move_t *chassis_move_control)
{
    if (chassis_move_control == NULL)
    {
        return;
    }

    fp32 vx_set = 0.0f, vy_set = 0.0f, angle_set = 0.0f;
    
    // 获取行为控制器的三个控制设置值
    // chassis_behaviour_control_set(&vx_set, &vy_set, &angle_set, chassis_move_control); // 假设此函数在外部实现

    // 假设这里调用了遥控器转换函数作为简化
    chassis_rc_to_control_vector(&vx_set, &vy_set, chassis_move_control);
    // 假设 angle_set 来自遥控器 WZ_CHANNEL
    // angle_set = chassis_move_control->chassis_RC->rc.ch[CHASSIS_WZ_CHANNEL] * CHASSIS_WZ_RC_SEN;


    // 根据模式设置最终的 vx_set, vy_set, wz_set
    if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW)
    {
        fp32 sin_yaw = 0.0f, cos_yaw = 0.0f;
        
        // 速度指令从云台坐标系转换到底盘坐标系
        // 旋转角度为 云台相对底盘角度 的负值
        // float angle_to_rotate = -chassis_move_control->chassis_yaw_motor->relative_angle;
        // arm_sin_cos_f32(angle_to_rotate, &sin_yaw, &cos_yaw);
        
        chassis_move_control->vx_set = cos_yaw * vx_set + sin_yaw * vy_set;
        chassis_move_control->vy_set = -sin_yaw * vx_set + cos_yaw * vy_set;
        
        // 设置控制相对云台角度
        // chassis_move_control->chassis_relative_angle_set = rad_format(angle_set); // angle_set 此时是相对角度设定
        
        // 计算旋转PID角速度
        // chassis_move_control->wz_set = -PID_calc(&chassis_move_control->chassis_angle_pid, 
        //                                         chassis_move_control->chassis_yaw_motor->relative_angle, 
        //                                         chassis_move_control->chassis_relative_angle_set);
                                                
        // 速度限幅
        chassis_move_control->vx_set = fp32_constrain(chassis_move_control->vx_set, chassis_move_control->vx_min_speed, chassis_move_control->vx_max_speed);
        chassis_move_control->vy_set = fp32_constrain(chassis_move_control->vy_set, chassis_move_control->vy_min_speed, chassis_move_control->vy_max_speed);
    }
    else if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW)
    {
        fp32 delat_angle = 0.0f;
        
        // 设置底盘控制的绝对角度
        // chassis_move_control->chassis_yaw_set = rad_format(angle_set); // angle_set 此时是绝对角度设定
        // delat_angle = rad_format(chassis_move_control->chassis_yaw_set - chassis_move_control->chassis_yaw);
        
        // 计算旋转的角速度
        chassis_move_control->wz_set = PID_calc(&chassis_move_control->chassis_angle_pid, 0.0f, delat_angle);
        
        // 速度限幅
        chassis_move_control->vx_set = fp32_constrain(vx_set, chassis_move_control->vx_min_speed, chassis_move_control->vx_max_speed);
        chassis_move_control->vy_set = fp32_constrain(vy_set, chassis_move_control->vy_min_speed, chassis_move_control->vy_max_speed);
    }
    else if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_NO_FOLLOW_YAW)
    {
        // "angle_set" 是旋转速度控制
        // chassis_move_control->wz_set = angle_set;
        chassis_move_control->vx_set = fp32_constrain(vx_set, chassis_move_control->vx_min_speed, chassis_move_control->vx_max_speed);
        chassis_move_control->vy_set = fp32_constrain(vy_set, chassis_move_control->vy_min_speed, chassis_move_control->vy_max_speed);
    }
    else if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_RAW)
    {
        // 在原始模式，设置值是发送到CAN总线 (电流值)
        chassis_move_control->vx_set = vx_set * MAX_MOTOR_CAN_CURRENT; 
        chassis_move_control->vy_set = vy_set * MAX_MOTOR_CAN_CURRENT;
        chassis_move_control->wz_set = angle_set * MAX_MOTOR_CAN_CURRENT;
        
        // 清除滤波器输出
        chassis_move_control->chassis_cmd_slow_set_vx.out = 0.0f;
        chassis_move_control->chassis_cmd_slow_set_vy.out = 0.0f;
    }
}

/**
 * @brief 全向轮逆运动学解算。四个全向轮的速度由 $V_x, V_y, \omega_z$ 计算得到。
 * @param[in] vx_set: 纵向速度 (m/s)
 * @param[in] vy_set: 横向速度 (m/s)
 * @param[in] wz_set: 旋转角速度 (rad/s)
 * @param[out] wheel_speed: 四个轮子的目标线速度 (m/s)
 * @retval None
 */
static void chassis_vector_to_omni_wheel_speed(const fp32 vx_set, const fp32 vy_set, const fp32 wz_set, fp32 wheel_speed[4])
{
    // 假设轮子在圆周上，半径为 R = MOTOR_DISTANCE_TO_CENTER
    fp32 D = MOTOR_DISTANCE_TO_CENTER;
    
    // 全向轮逆解公式: Vi = Vx*cos(ai) + Vy*sin(ai) + wz*D
    // Wheel 0: 前左 (135 deg)
    // Wheel 1: 前右 (45 deg)
    // Wheel 2: 后右 (-45 deg)
    // Wheel 3: 后左 (-135 deg)

    // Wheel 0 (前左, 135 deg)
    wheel_speed[0] = vx_set * (-COS_45) + vy_set * (SIN_45) + wz_set * D;
    
    // Wheel 1 (前右, 45 deg)
    wheel_speed[1] = vx_set * (COS_45) + vy_set * (SIN_45) + wz_set * D;
    
    // Wheel 2 (后右, -45 deg)
    wheel_speed[2] = vx_set * (COS_45) + vy_set * (-SIN_45) + wz_set * D;

    // Wheel 3 (后左, -135 deg)
    wheel_speed[3] = vx_set * (-COS_45) + vy_set * (-SIN_45) + wz_set * D;
}


/**
 * @brief 控制循环，计算电机电流值
 */
static void chassis_control_loop(chassis_move_t *chassis_move_control_loop)
{
    fp32 max_vector = 0.0f, vector_rate = 0.0f;
    fp32 temp = 0.0f;
    fp32 wheel_speed[4] = {0.0f};
    uint8_t i = 0;

    // 运动分解 (全向轮逆解)
    chassis_vector_to_omni_wheel_speed(chassis_move_control_loop->vx_set,
                                       chassis_move_control_loop->vy_set, 
                                       chassis_move_control_loop->wz_set, 
                                       wheel_speed);

    if (chassis_move_control_loop->chassis_mode == CHASSIS_VECTOR_RAW)
    {
        // Raw模式下，设置值直接视为电流并返回
        chassis_move_control_loop->motor_chassis[0].give_current = (int16_t)(chassis_move_control_loop->vx_set);
        chassis_move_control_loop->motor_chassis[1].give_current = (int16_t)(chassis_move_control_loop->vy_set);
        chassis_move_control_loop->motor_chassis[2].give_current = (int16_t)(chassis_move_control_loop->wz_set);
        chassis_move_control_loop->motor_chassis[3].give_current = (int16_t)(chassis_move_control_loop->wz_set);       
        return;
    }

    // 轮速限幅
    for (i = 0; i < 4; i++)
    {
        chassis_move_control_loop->motor_chassis[i].speed_set = wheel_speed[i];
        temp = fabs(chassis_move_control_loop->motor_chassis[i].speed_set);
        if (max_vector < temp)
        {
            max_vector = temp;
        }
    }

    // 超速降速
    if (max_vector > MAX_WHEEL_SPEED)
    {
        vector_rate = MAX_WHEEL_SPEED / max_vector;
        for (i = 0; i < 4; i++)
        {
            chassis_move_control_loop->motor_chassis[i].speed_set *= vector_rate;
        }
    }

    // PID 计算 (速度环)
    for (i = 0; i < 4; i++)
    {
        PID_calc(&chassis_move_control_loop->motor_speed_pid[i], 
                 chassis_move_control_loop->motor_chassis[i].speed, 
                 chassis_move_control_loop->motor_chassis[i].speed_set);
    }

    // 赋值电流值
    for (i = 0; i < 4; i++)
    {
        chassis_move_control_loop->motor_chassis[i].give_current = (int16_t)(chassis_move_control_loop->motor_speed_pid[i].out);
    }
}    



pid_type_def chassis_speed_1=
{
    .mode = PID_DELTA,
    .Kp = INFANTRY_CHASSIS_SPEED_KP,
    .Ki = 0.1f,
    .Kd = 0.0f,
    .max_out = MAX_MOTOR_CAN_CURRENT,
    .max_iout = 1000.0f
};
pid_type_def chassis_speed_2=
{
    .mode = PID_DELTA,
    .Kp = INFANTRY_CHASSIS_SPEED_KP_M2,
    .Ki = 0.08f,
    .Kd = 0.0f,
    .max_out = MAX_MOTOR_CAN_CURRENT,
    .max_iout = 1000.0f
};  
pid_type_def chassis_speed_3=
{
    .mode = PID_DELTA,
    .Kp = INFANTRY_CHASSIS_SPEED_KP,
    .Ki = 0.1f,
    .Kd = 0.0f,
    .max_out = MAX_MOTOR_CAN_CURRENT,
    .max_iout = 1000.0f
};
pid_type_def chassis_speed_4=
{
    .mode = PID_DELTA,
    .Kp = INFANTRY_CHASSIS_SPEED_KP,
    .Ki = 0.1f,
    .Kd = 0.0f,
    .max_out = MAX_MOTOR_CAN_CURRENT,
    .max_iout = 1000.0f
};
void Test_3508_PID(int16_t set_1,int16_t set_2,int16_t set_3, int16_t set_4)
{
    const motor_measure_t *chassis_motor_1 = get_chassis_motor_measure_point(0);
    const motor_measure_t *chassis_motor_2 = get_chassis_motor_measure_point(1);
    const motor_measure_t *chassis_motor_3 = get_chassis_motor_measure_point(2);
    const motor_measure_t *chassis_motor_4 = get_chassis_motor_measure_point(3);
    int16_t chassis_speed_out_1,chassis_speed_out_2,chassis_speed_out_3,chassis_speed_out_4;

    chassis_speed_out_1 = PID_calc(&chassis_speed_1, chassis_motor_1->speed_rpm, set_1);
    chassis_speed_out_2 = PID_calc(&chassis_speed_2, chassis_motor_2->speed_rpm, set_2);
    chassis_speed_out_3 = PID_calc(&chassis_speed_3, chassis_motor_3->speed_rpm, set_3);
    chassis_speed_out_4 = PID_calc(&chassis_speed_4, chassis_motor_4->speed_rpm, set_4);

    CAN_cmd_chassis(
        chassis_speed_out_1,
        chassis_speed_out_2,
        chassis_speed_out_3,
        chassis_speed_out_4
    );
}

static int Cacl_prm_to_Velocity(int16_t prm);
static int Cacl_Velocity_to_Vx(int16_t velocity_motor);
static int Cacl_Velocity_to_Vy(int16_t velocity_motor);
static int ramp(int16_t velocity_motor);
extern TIM_HandleTypeDef htim14;
static int g_speed=0;
extern float angle[3];

static uint8_t chassis_small_gyro_enable = 0u;
static int8_t chassis_small_gyro_dir = 1;
static uint16_t chassis_small_gyro_key_last = 0u;
static float chassis_small_gyro_target_rpm = 0.0f;
static uint8_t chassis_low_hp_speed_boost = 0u;

static float chassis_small_gyro_get_speed_from_key(uint16_t key_value)
{
    float speed;

    if ((key_value & KEY_PRESSED_OFFSET_SHIFT) && !(key_value & KEY_PRESSED_OFFSET_CTRL))
    {
        speed = (float)CHASSIS_SMALL_GYRO_SPEED_HIGH;
    }
    else if ((key_value & KEY_PRESSED_OFFSET_CTRL) && !(key_value & KEY_PRESSED_OFFSET_SHIFT))
    {
        speed = (float)CHASSIS_SMALL_GYRO_SPEED_LOW;
    }
    else
    {
        speed = (float)CHASSIS_KEYBOARD_WZ_SPEED;
    }

    if (chassis_low_hp_speed_boost != 0u)
    {
        if ((key_value & KEY_PRESSED_OFFSET_SHIFT) && !(key_value & KEY_PRESSED_OFFSET_CTRL))
        {
            speed = (float)CHASSIS_SMALL_GYRO_SPEED_BOOST;
        }
        else if ((key_value & KEY_PRESSED_OFFSET_CTRL) && !(key_value & KEY_PRESSED_OFFSET_SHIFT))
        {
            speed = (float)CHASSIS_KEYBOARD_WZ_SPEED;
        }
        else
        {
            speed = (float)CHASSIS_SMALL_GYRO_SPEED_HIGH;
        }
    }

    return speed;
}

static void chassis_small_gyro_update_from_keyboard(uint16_t key_value)
{
    uint8_t f_now = (key_value & KEY_PRESSED_OFFSET_F) ? 1u : 0u;
    uint8_t g_now = (key_value & KEY_PRESSED_OFFSET_G) ? 1u : 0u;
    uint8_t f_last = (chassis_small_gyro_key_last & KEY_PRESSED_OFFSET_F) ? 1u : 0u;
    uint8_t g_last = (chassis_small_gyro_key_last & KEY_PRESSED_OFFSET_G) ? 1u : 0u;

    if ((f_now != 0u) && (f_last == 0u))
    {
        chassis_small_gyro_enable ^= 1u;
    }

    if ((g_now != 0u) && (g_last == 0u))
    {
        chassis_small_gyro_dir = (int8_t)(-chassis_small_gyro_dir);
        if (chassis_small_gyro_dir == 0)
        {
            chassis_small_gyro_dir = 1;
        }
    }

    chassis_small_gyro_key_last = key_value;

    if (chassis_small_gyro_enable != 0u)
    {
        chassis_small_gyro_target_rpm = chassis_small_gyro_get_speed_from_key(key_value) * (float)chassis_small_gyro_dir;
    }
    else
    {
        chassis_small_gyro_target_rpm = 0.0f;
    }
}
/**
 * @brief 坐标变换：将云台坐标系的速度转换到底盘坐标系
 * @param vx_gimbal: 云台坐标系X方向速度（前后）
 * @param vy_gimbal: 云台坐标系Y方向速度（左右）
 * @param gimbal_yaw: 云台当前yaw角（度）
 * @param vx_chassis: 输出底盘坐标系X速度
 * @param vy_chassis: 输出底盘坐标系Y速度
 */

 static void transform_vector_to_chassis(float vx_gimbal, float vy_gimbal, float gimbal_yaw,float *vx_chassis, float *vy_chassis)
 {
     /* Use the measured gimbal-to-chassis heading directly (MOVE:rel).
      * The state machine already applies the heading sign convention.
      * Both normal drive and spinning drive use this conversion. */
     float relative_yaw = gimbal_yaw;
     // 角度转弧度
     float rad = relative_yaw * 3.1415926f / 180.0f;
     float cos_yaw = cosf(rad);
     float sin_yaw = sinf(rad);
     // 坐标变换公式：
     // 底盘vx = 云台vx * cos(yaw) - 云台vy * sin(yaw)
     // 底盘vy = 云台vx * sin(yaw) + 云台vy * cos(yaw)
     *vx_chassis = vx_gimbal * cos_yaw - vy_gimbal * sin_yaw;
     *vy_chassis = vx_gimbal * sin_yaw + vy_gimbal * cos_yaw;
 }

static int16_t get_gimbal_small_gyro_compensation(void)
{
    uint32_t now_tick = HAL_GetTick();
    uint32_t update_tick = get_usart1_gimbal_gyro_update_tick();
    float gyro_comp_speed_rpm;
    float gyro_comp_z_set;

    if ((uint32_t)(now_tick - update_tick) > CHASSIS_GYRO_FRAME_TIMEOUT_MS)
    {
        return 0;
    }

    if (get_usart1_gimbal_gyro_enable() == 0u)
    {
        return 0;
    }

    gyro_comp_speed_rpm = get_usart1_gimbal_gyro_comp_speed_rpm();
    gyro_comp_z_set = CHASSIS_GYRO_COMP_SIGN * gyro_comp_speed_rpm * CHASSIS_GYRO_RPM_TO_Z_SET;

    if (gyro_comp_z_set > 20000.0f)
    {
        gyro_comp_z_set = 20000.0f;
    }
    else if (gyro_comp_z_set < -20000.0f)
    {
        gyro_comp_z_set = -20000.0f;
    }

    return (int16_t)gyro_comp_z_set;
}

void RC_Control_Chassis(chassis_mode_e mode)
{

    const motor_measure_t *chassis_motor_1 = get_chassis_motor_measure_point(0);
    const motor_measure_t *chassis_motor_2 = get_chassis_motor_measure_point(1);
    const motor_measure_t *chassis_motor_3 = get_chassis_motor_measure_point(2);
    const motor_measure_t *chassis_motor_4 = get_chassis_motor_measure_point(3);
    // const RC_ctrl_t *rc_data = get_remote_control_point();
    const RC_ctrl_t *rc_data = get_usart1_remote_control_point();
    int16_t motor1_set, motor2_set, motor3_set, motor4_set;
    fp32 chassis_speed_out_1, chassis_speed_out_2, chassis_speed_out_3, chassis_speed_out_4;

    int16_t vx_set = (int16_t)(rc_data->rc.ch[3] * CHASSIS_RC_VX_SET_SCALE *
                              INFANTRY_CHASSIS_VX_DIRECTION);
    int16_t vy_set = (int16_t)(rc_data->rc.ch[2] * CHASSIS_RC_VY_SET_SCALE *
                              INFANTRY_CHASSIS_VY_DIRECTION);
    int16_t z_set = (chassis_small_gyro_enable != 0u) ?
                    (int16_t)chassis_small_gyro_target_rpm : 0;
    int16_t dial = rc_data->rc.ch[4];
    int16_t wz_set;
    if (dial > 660) dial = 660;
    if (dial < -660) dial = -660;
    /* Manual dial rotation; left-down keeps the fixed small-gyro speed. */
    if ((chassis_small_gyro_enable == 0u) &&
        ((dial > INFANTRY_RC_DEADZONE) || (dial < -INFANTRY_RC_DEADZONE)))
        z_set = (int16_t)((float)dial * INFANTRY_CHASSIS_DIAL_SCALE);
    wz_set = z_set; /* Right stick still belongs exclusively to the gimbal. */
    if ((rc_data->rc.ch[3] <= INFANTRY_RC_DEADZONE) &&
        (rc_data->rc.ch[3] >= -INFANTRY_RC_DEADZONE)) vx_set = 0;
    if ((rc_data->rc.ch[2] <= INFANTRY_RC_DEADZONE) &&
        (rc_data->rc.ch[2] >= -INFANTRY_RC_DEADZONE)) vy_set = 0;

#if (INFANTRY_SLIPRING_TEST != 0u)
    vx_set = ((rc_data->rc.ch[3] > INFANTRY_TEST_RC_DEADZONE) ||
              (rc_data->rc.ch[3] < -INFANTRY_TEST_RC_DEADZONE)) ?
              (int16_t)(rc_data->rc.ch[3] * INFANTRY_TEST_CHASSIS_RC_SCALE) : 0;
    vy_set = ((rc_data->rc.ch[2] > INFANTRY_TEST_RC_DEADZONE) ||
              (rc_data->rc.ch[2] < -INFANTRY_TEST_RC_DEADZONE)) ?
              -(int16_t)(rc_data->rc.ch[2] * INFANTRY_TEST_CHASSIS_RC_SCALE) : 0;
    z_set = 0;
    wz_set = 0;
#endif
    if ((vx_set == 0) && (vy_set == 0) && (z_set == 0))
    {
        Chassis_ResetSpeedPID();
        chassis_output_clear();
        return;
    }

    if(mode==CHASSIS_VECTOR_RAW)
    {
        motor1_set = vx_set * (COS_45) + vy_set * (-SIN_45) + wz_set * MOTOR_DISTANCE_TO_CENTER;
        motor2_set = vx_set * (COS_45) + vy_set * (SIN_45)+ wz_set * MOTOR_DISTANCE_TO_CENTER;
        motor3_set = vx_set * (-COS_45) + vy_set * (SIN_45)+ wz_set * MOTOR_DISTANCE_TO_CENTER;
        motor4_set = vx_set * (-COS_45) + vy_set * (-SIN_45)+ wz_set * MOTOR_DISTANCE_TO_CENTER;
    }
    else if(mode==CHASSIS_VECTOR_NO_FOLLOW_YAW)
    {
        motor1_set = vx_set * (COS_45) + vy_set * (-SIN_45) + wz_set * MOTOR_DISTANCE_TO_CENTER;
        motor2_set = vx_set * (COS_45) + vy_set * (SIN_45) + wz_set * MOTOR_DISTANCE_TO_CENTER;
        motor3_set = vx_set * (-COS_45) + vy_set * (SIN_45) + wz_set * MOTOR_DISTANCE_TO_CENTER;
        motor4_set = vx_set * (-COS_45) + vy_set * (-SIN_45) + wz_set * MOTOR_DISTANCE_TO_CENTER;
    }
    else if (mode==CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW)
    {
        // 云台跟随模式：以云台前方为前方
        float vx_chassis, vy_chassis;
        float gimbal_yaw = yaw_angle;
        
        // 将云台坐标系的速度转换到底盘坐标系
        transform_vector_to_chassis(vx_set, vy_set, gimbal_yaw, &vx_chassis, &vy_chassis);

        // 使用转换后的速度进行运动学解算
        motor1_set = vx_chassis * (COS_45) + vy_chassis * (-SIN_45) + MOTOR_DISTANCE_TO_CENTER * z_set; // 假设 wz_set 是角速度，乘以距离得到线速度
        motor2_set = vx_chassis * (COS_45) + vy_chassis * (SIN_45) + MOTOR_DISTANCE_TO_CENTER * z_set;
        motor3_set = vx_chassis * (-COS_45) + vy_chassis * (SIN_45) + MOTOR_DISTANCE_TO_CENTER * z_set;
        motor4_set = vx_chassis * (-COS_45) + vy_chassis * (-SIN_45) + MOTOR_DISTANCE_TO_CENTER * z_set;
    }
    


    chassis_speed_out_1 = PID_calc(&chassis_speed_1, chassis_motor_1->speed_rpm, motor1_set);
    chassis_speed_out_2 = PID_calc(&chassis_speed_2, chassis_motor_2->speed_rpm, motor2_set);
    chassis_speed_out_3 = PID_calc(&chassis_speed_3, chassis_motor_3->speed_rpm, motor3_set);
    chassis_speed_out_4 = PID_calc(&chassis_speed_4, chassis_motor_4->speed_rpm, motor4_set);

    int v1 = Cacl_prm_to_Velocity(chassis_motor_1->speed_rpm);
    int vx = Cacl_Velocity_to_Vx(v1);   
    // printf("%d,%d\r\n",v1,vx/2);

    chassisoutput.chassis_motor_1 = chassis_speed_out_1;
    chassisoutput.chassis_motor_2 = chassis_speed_out_2;
    chassisoutput.chassis_motor_3 = chassis_speed_out_3;
    chassisoutput.chassis_motor_4 = chassis_speed_out_4;

    // CAN_cmd_chassis((int16_t)chassis_speed_out_1, (int16_t)chassis_speed_out_2,
    //                   (int16_t)chassis_speed_out_3, (int16_t)chassis_speed_out_4);
}


void Keyboard_Control_Chassis(chassis_mode_e mode)
{
    const motor_measure_t *chassis_motor_1 = get_chassis_motor_measure_point(0);
    const motor_measure_t *chassis_motor_2 = get_chassis_motor_measure_point(1);
    const motor_measure_t *chassis_motor_3 = get_chassis_motor_measure_point(2);
    const motor_measure_t *chassis_motor_4 = get_chassis_motor_measure_point(3);
    uint16_t key_value = get_usart1_keyboard_value();
    int16_t speed_level = CHASSIS_KEYBOARD_SPEED_MID;
    int16_t vx_set = 0;
    int16_t vy_set = 0;
    int16_t z_set = 0;
    int16_t wz_set = 0;
    int16_t manual_wz_speed = 0;
    int16_t motor1_set = 0;
    int16_t motor2_set = 0;
    int16_t motor3_set = 0;
    int16_t motor4_set = 0;
    fp32 chassis_speed_out_1;
    fp32 chassis_speed_out_2;
    fp32 chassis_speed_out_3;
    fp32 chassis_speed_out_4;

    if ((key_value & KEY_PRESSED_OFFSET_SHIFT) && !(key_value & KEY_PRESSED_OFFSET_CTRL))
    {
        speed_level = CHASSIS_KEYBOARD_SPEED_HIGH;
    }
    else if ((key_value & KEY_PRESSED_OFFSET_CTRL) && !(key_value & KEY_PRESSED_OFFSET_SHIFT))
    {
        speed_level = CHASSIS_KEYBOARD_SPEED_LOW;
    }

    if (chassis_low_hp_speed_boost != 0u)
    {
        if (speed_level == CHASSIS_KEYBOARD_SPEED_LOW)
        {
            speed_level = CHASSIS_KEYBOARD_SPEED_MID;
        }
        else if (speed_level == CHASSIS_KEYBOARD_SPEED_MID)
        {
            speed_level = CHASSIS_KEYBOARD_SPEED_HIGH;
        }
        else
        {
            speed_level = CHASSIS_KEYBOARD_SPEED_BOOST;
        }
    }

    if (key_value & KEY_PRESSED_OFFSET_W)
    {
        vx_set += speed_level;
    }
    else if (key_value & KEY_PRESSED_OFFSET_S)
    {
        vx_set -= speed_level;
    }

    if (key_value & KEY_PRESSED_OFFSET_A)
    {
        vy_set += speed_level;
    }
    else if (key_value & KEY_PRESSED_OFFSET_D)
    {
        vy_set -= speed_level;
    }

    chassis_small_gyro_update_from_keyboard(key_value);
    z_set = (int16_t)chassis_small_gyro_target_rpm;

    if (chassis_small_gyro_enable == 0u)
    {
        manual_wz_speed = (int16_t)chassis_small_gyro_get_speed_from_key(key_value);

        if (key_value & KEY_PRESSED_OFFSET_Q)
        {
            z_set = (int16_t)(-manual_wz_speed);
        }
        else if (key_value & KEY_PRESSED_OFFSET_E)
        {
            z_set = manual_wz_speed;
        }
    }

    wz_set = z_set;

    if (mode == CHASSIS_VECTOR_RAW)
    {
        motor1_set = (int16_t)(vx_set * COS_45 + vy_set * (-SIN_45) + wz_set * MOTOR_DISTANCE_TO_CENTER);
        motor2_set = (int16_t)(vx_set * COS_45 + vy_set * SIN_45 + wz_set * MOTOR_DISTANCE_TO_CENTER);
        motor3_set = (int16_t)(vx_set * (-COS_45) + vy_set * SIN_45 + wz_set * MOTOR_DISTANCE_TO_CENTER);
        motor4_set = (int16_t)(vx_set * (-COS_45) + vy_set * (-SIN_45) + wz_set * MOTOR_DISTANCE_TO_CENTER);
    }
    else if (mode == CHASSIS_VECTOR_NO_FOLLOW_YAW)
    {
        motor1_set = (int16_t)(vx_set * COS_45 + vy_set * (-SIN_45));
        motor2_set = (int16_t)(vx_set * COS_45 + vy_set * SIN_45);
        motor3_set = (int16_t)(vx_set * (-COS_45) + vy_set * SIN_45);
        motor4_set = (int16_t)(vx_set * (-COS_45) + vy_set * (-SIN_45));
    }
    else if (mode == CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW)
    {
        float vx_chassis;
        float vy_chassis;

        transform_vector_to_chassis((float)vx_set, (float)vy_set, yaw_angle, &vx_chassis, &vy_chassis);

        motor1_set = (int16_t)(vx_chassis * COS_45 + vy_chassis * (-SIN_45) + MOTOR_DISTANCE_TO_CENTER * z_set);
        motor2_set = (int16_t)(vx_chassis * COS_45 + vy_chassis * SIN_45 + MOTOR_DISTANCE_TO_CENTER * z_set);
        motor3_set = (int16_t)(vx_chassis * (-COS_45) + vy_chassis * SIN_45 + MOTOR_DISTANCE_TO_CENTER * z_set);
        motor4_set = (int16_t)(vx_chassis * (-COS_45) + vy_chassis * (-SIN_45) + MOTOR_DISTANCE_TO_CENTER * z_set);
    }
    else
    {
        motor1_set = (int16_t)(vx_set * COS_45 + vy_set * (-SIN_45) + MOTOR_DISTANCE_TO_CENTER * z_set);
        motor2_set = (int16_t)(vx_set * COS_45 + vy_set * SIN_45 + MOTOR_DISTANCE_TO_CENTER * z_set);
        motor3_set = (int16_t)(vx_set * (-COS_45) + vy_set * SIN_45 + MOTOR_DISTANCE_TO_CENTER * z_set);
        motor4_set = (int16_t)(vx_set * (-COS_45) + vy_set * (-SIN_45) + MOTOR_DISTANCE_TO_CENTER * z_set);
    }

    chassis_speed_out_1 = PID_calc(&chassis_speed_1, chassis_motor_1->speed_rpm, motor1_set);
    chassis_speed_out_2 = PID_calc(&chassis_speed_2, chassis_motor_2->speed_rpm, motor2_set);
    chassis_speed_out_3 = PID_calc(&chassis_speed_3, chassis_motor_3->speed_rpm, motor3_set);
    chassis_speed_out_4 = PID_calc(&chassis_speed_4, chassis_motor_4->speed_rpm, motor4_set);

    chassisoutput.chassis_motor_1 = (int16_t)chassis_speed_out_1;
    chassisoutput.chassis_motor_2 = (int16_t)chassis_speed_out_2;
    chassisoutput.chassis_motor_3 = (int16_t)chassis_speed_out_3;
    chassisoutput.chassis_motor_4 = (int16_t)chassis_speed_out_4;
}

void Keyboard_Control_Chassis_Reset(void)
{
    chassis_small_gyro_enable = 0u;
    chassis_small_gyro_dir = 1;
    chassis_small_gyro_key_last = 0u;
    chassis_small_gyro_target_rpm = 0.0f;
}

void chassis_output_clear(void)
{
    chassisoutput.chassis_motor_1 = 0;
    chassisoutput.chassis_motor_2 = 0;
    chassisoutput.chassis_motor_3 = 0;
    chassisoutput.chassis_motor_4 = 0;
}

void Chassis_ResetSpeedPID(void)
{
    PID_clear(&chassis_speed_1);
    PID_clear(&chassis_speed_2);
    PID_clear(&chassis_speed_3);
    PID_clear(&chassis_speed_4);
}

void Chassis_GetWheelSpeedTargets(int16_t targets[4])
{
    targets[0] = (int16_t)chassis_speed_1.set;
    targets[1] = (int16_t)chassis_speed_2.set;
    targets[2] = (int16_t)chassis_speed_3.set;
    targets[3] = (int16_t)chassis_speed_4.set;
}

void chassis_output_send(void)
{
    CAN_cmd_chassis(
        chassisoutput.chassis_motor_1,
        chassisoutput.chassis_motor_2, 
        chassisoutput.chassis_motor_3, 
        chassisoutput.chassis_motor_4 
    );
}

static int Cacl_prm_to_Velocity(int16_t prm)
{
    return prm * M3508_RPM_TO_VECTOR; // 0.152*3.1415926/60
}
static int Cacl_Velocity_to_Vx(int16_t velocity_motor)
{
    return velocity_motor * COS_45;
}
static int Cacl_Velocity_to_Vy(int16_t velocity_motor)
{
    return velocity_motor * SIN_45;
}
static int ramp(int16_t velocity_motor)
{
    if(g_speed<velocity_motor)
    {
        HAL_TIM_Base_Start_IT(&htim14);
    }
    
    else
    {
        HAL_TIM_Base_Stop_IT(&htim14);        
    }
    
    return g_speed;
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim->Instance==TIM14)
    {
        g_speed+=80;
    }
}
/**
 * @brief 获取底盘绕Z轴的旋转角速度。此函数通过读取四个全向轮电机的速度，并根据全向轮的运动学关系计算得到底盘的旋转角速度。
 * 
 * @return  底盘绕Z轴的旋转角速度
 */
float Get_Chassis_Z_Rpm(void)
{
    const motor_measure_t *chassis_motor_1 = get_chassis_motor_measure_point(0);
    const motor_measure_t *chassis_motor_2 = get_chassis_motor_measure_point(1);
    const motor_measure_t *chassis_motor_3 = get_chassis_motor_measure_point(2);
    const motor_measure_t *chassis_motor_4 = get_chassis_motor_measure_point(3);

    const float reduction_ratio = 19.0f;
    const float wheel_radius_m = 0.075f;
    const float chassis_radius_m = 0.25f;
    /* 3508电机转速(rpm) -> 轮速(rpm) -> 底盘自转速度(rpm) */
    float motor_rpm_avg = - (chassis_motor_1->speed_rpm + chassis_motor_2->speed_rpm + chassis_motor_3->speed_rpm + chassis_motor_4->speed_rpm) * 0.25f;
    float wheel_rpm = motor_rpm_avg / reduction_ratio;

    return wheel_rpm * (wheel_radius_m / chassis_radius_m);
}

void Send_Chassis_Rpm_Data(void)
{
    usart1_tx_status_t tx_status;

    memset(&tx_status, 0, sizeof(tx_status));
    tx_status.chassis_gyro_speed_rpm = Get_Chassis_Z_Rpm();
    if (Chassis_Small_Gyro_Is_Enabled() != 0u)
    {
        tx_status.flags |= INFANTRY_CHASSIS_FLAG_SMALL_GYRO;
    }

    data_to_usart1(&tx_status);
}

uint8_t Chassis_Small_Gyro_Is_Enabled(void)
{
    return chassis_small_gyro_enable;
}

void Chassis_Set_Small_Gyro(uint8_t enable)
{
    chassis_small_gyro_enable = (enable != 0u) ? 1u : 0u;
    chassis_small_gyro_target_rpm = (chassis_small_gyro_enable != 0u) ?
                                    INFANTRY_CHASSIS_SMALL_GYRO_SPEED : 0.0f;
}

float Chassis_Small_Gyro_Get_Target_Rpm(void)
{
    return chassis_small_gyro_target_rpm;
}


void Chassis_Set_Low_Hp_Speed_Boost(uint8_t enable)
{
    chassis_low_hp_speed_boost = (enable != 0u) ? 1u : 0u;
}



























