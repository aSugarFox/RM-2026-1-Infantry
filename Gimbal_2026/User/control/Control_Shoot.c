/*
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     2026-4-12       zxy             射击控制代码
   ****************************(C) COPYRIGHT 2024****************************        
  */
#include "Control_Shoot.h"

pid_type_def fric_speed_1=
{
    .mode = PID_POSITION,
    .Kp = 2.5f,
    .Ki = 0.1f,
    .Kd = 0.0f,
    .max_out = MAX_MOTOR_CAN_CURRENT,
    .max_iout = 1000.0f
};
pid_type_def fric_speed_2=
{
    .mode = PID_POSITION,
    .Kp = 2.5f,
    .Ki = 0.1f,
    .Kd = 0.0f,
    .max_out = MAX_MOTOR_CAN_CURRENT,
    .max_iout = 1000.0f
};  
pid_type_def trigger_speed=
{
    .mode = PID_POSITION,
    .Kp = 5.0f,
    .Ki = 0.0f,
    .Kd = 0.0f,
    .max_out = 8000.0f,
    .max_iout = 1000.0f
};  

void RC_Control_Bodan(int16_t bodan_set)
{
    const motor_measure_t *bodan_motor = get_trigger_motor_measure_point();
    const RC_ctrl_t *rc_data = get_remote_control_point();
    fp32 trigger_speed_out;
    trigger_speed_out   = PID_calc(&trigger_speed, bodan_motor->speed_rpm, bodan_set);
    gimbal_out.trigger = trigger_speed_out;
}
void RC_Control_Fric(int fric_1_set, int fric_2_set)
{
    const motor_measure_t *fric_1 = get_fric_wheel_left_motor_measure_point();
    const motor_measure_t *fric_2 = get_fric_wheel_right_motor_measure_point();
    const RC_ctrl_t *rc_data = get_remote_control_point();
    fp32 chassis_speed_out_1, chassis_speed_out_2;
    chassis_speed_out_1 = PID_calc(&fric_speed_1, fric_1->speed_rpm, fric_1_set);
    chassis_speed_out_2 = PID_calc(&fric_speed_2, fric_2->speed_rpm, fric_2_set);
    CAN_cmd_fric_wheel((int16_t)chassis_speed_out_1, (int16_t)chassis_speed_out_2,0,0);
}


/* 卡弹检测与恢复 */
static uint8_t detect_and_recover_jam(shoot_control_t *ctrl)
{
    if (ctrl == NULL) return 0;
    
    const motor_measure_t *trigger_motor = get_trigger_motor_measure_point();
    if (trigger_motor == NULL) return 0;
    
    /* 检测卡弹 */
    if (ctrl->trigger_target > 0 && 
        fabs(trigger_motor->speed_rpm) < STUCK_SPEED_THRESHOLD) {
        ctrl->stuck_timer++;
    } else {
        ctrl->stuck_timer = 0;
    }
    
    /* 如果卡弹超过阈值，进入恢复模式 */
    if (ctrl->stuck_timer > STUCK_DETECT_TIME) {
        ctrl->is_jammed = 1;
        ctrl->mode = SHOOT_MODE_STUCK_RECOVER;
        // printf("检测到卡弹，进入恢复模式\r\n");
        return 1;
    }
    
    return 0;
}

/* 位置环PID控制 */
static pid_type_def position_pid = {
    .mode = PID_POSITION,
    .Kp = 50.0f,
    .Ki = 0.5f,
    .Kd = 1.0f,
    .max_out = 5000.0f,
    .max_iout = 1000.0f
};

void motor_rotate_one_circle_speed(void)
{
    const motor_measure_t *motor = get_trigger_motor_measure_point(); // 获取电机指针
    const RC_ctrl_t *rc_data = get_remote_control_point();// 获取遥控器数据指针
    if (!motor) return;
    
    static uint32_t start_time = 0;
    static uint8_t is_rotating = 0;
    
    if (!is_rotating) {
        /* 开始旋转 */
        is_rotating = 1;
        start_time = HAL_GetTick();
        
        /* 设置目标速度，计算需要的时间 */
        float target_speed = 2000.0f;  // 2000 RPM
        /* 转一圈需要的时间（秒）= 60 / 转速 */
        /* 由于减速比36:1，输出轴转一圈，电机需要转36圈 */
        float motor_circles = 36.0f;  // 电机需要转的圈数
        float needed_time = (motor_circles * 60.0f / target_speed) * 1000.0f;  // 转换为毫秒
        
        // printf("开始旋转一圈，预计时间: %.1fms\r\n", needed_time);
        
        /* 设置电机速度 */
        RC_Control_Bodan(target_speed);  // 正转
    }
    
    /* 检查是否到达时间 */
    uint32_t current_time = HAL_GetTick();
    int shoot_flag=0;
    if(rc_data->rc.ch[4]>200)
    {
        shoot_flag=1;
    }
    if (current_time - start_time > 500 && shoot_flag==1) 
    {  
        /* 停止电机 */
        RC_Control_Bodan(0);
        is_rotating = 0;
        shoot_flag=0;
    }
}



void motor_rotate_one_circle_position(void)
{
    static motor_2006_controller_t ctrl = {0};
    static uint8_t initialized = 0;
    
    if (!initialized) {
        ctrl.mode = MOTOR_ROTATE_ONE_CIRCLE;
        ctrl.is_running = 0;
        initialized = 1;
    }
    
    const motor_measure_t *motor = get_trigger_motor_measure_point();
    if (!motor) return;
    
    if (!ctrl.is_running) {
        /* 开始旋转一圈 */
        ctrl.is_running = 1;
        ctrl.start_ecd = motor->ecd;
        ctrl.total_ecd = 0;
        
        /* 目标：输出轴转一圈 = 电机转36圈 */
        /* 36圈 = 36 * 8192 = 294912个编码器值 */
        ctrl.target_angle = 360.0f;  // 输出轴转360度
        float motor_angle = ctrl.target_angle * MOTOR_2006_REDUCTION_RATIO;  // 电机需要转的角度
    }
    
    /* 计算当前总编码器值（考虑溢出） */
    int32_t delta_ecd = motor->ecd - ctrl.start_ecd;
    
    /* 处理编码器溢出 */
    if (delta_ecd > HALF_ECD_RANGE) {
        delta_ecd -= ECD_RANGE;
    } else if (delta_ecd < -HALF_ECD_RANGE) {
        delta_ecd += ECD_RANGE;
    }
    
    ctrl.total_ecd = delta_ecd;
    
    /* 转换为输出轴角度 */
    ctrl.current_angle = (ctrl.total_ecd * 360.0f) / (MOTOR_2006_ENCODER_RESOLUTION * MOTOR_2006_REDUCTION_RATIO);
    
    /* 计算角度误差 */
    float angle_error = ctrl.target_angle - ctrl.current_angle;
    
    float speed_set = PID_calc(&position_pid, ctrl.current_angle, ctrl.target_angle);
    
    /* 限制速度范围 */
    if (speed_set > 2000.0f) speed_set = 2000.0f;
    if (speed_set < -2000.0f) speed_set = -2000.0f;
    
    /* 控制电机 */
    RC_Control_Bodan((int16_t)speed_set);
    
    /* 检查是否到达目标 */
    if (fabs(angle_error) < 2.0f) 
    {  // 误差小于2度认为到达
        RC_Control_Bodan(0);
        ctrl.is_running = 0;
    }
}

