/**
  ******************************************************************************
  * @attention
  * @file       Control_Gimbal.c/h
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
#include "Control_Gimbal.h"
#include "usart6_remote_control.h"
#include <math.h>
#include <stdlib.h>


#define TUCHUAN_MOUSE_PITCH_FINE_MIN_SCALE  0.45f   // 鼠标俯仰微动最小倍率；越小微调越细，越大越直接
#define TUCHUAN_MOUSE_PITCH_FINE_RANGE      100.0f  // 鼠标俯仰从微调恢复到全增益的范围
#define TUCHUAN_MOUSE_IDLE_REBASE_YAW_ERR_DEG        3.0f //1.5f   // 无鼠标输入时，Yaw 目标角回贴当前角的误差阈值；静止仍慢漂就增大，松手后黏手就减小
#define TUCHUAN_MOUSE_IDLE_REBASE_PITCH_ERR_DEG      0.8f  //0.8  // 无鼠标输入时，Pitch 目标角回贴当前角的误差阈值；静止仍慢漂就增大，停下后发闷就减小
#define TUCHUAN_MOUSE_IDLE_REBASE_SPEED_RPM          18.0f  //10.0  // 无鼠标输入时允许回贴基准角的最大电机转速；静止漂移还在就增大，松手后发飘就减小
#define TUCHUAN_MOUSE_IDLE_LOCK_DELAY_MS               30u
#define TUCHUAN_GYRO_CHASSIS_MOUSE_CMD_FILTER_TAU_MS  24.0f    // 鼠标 yaw 估计角速度低通时间常数，用于减少和底盘补偿互相打架
#define TUCHUAN_GYRO_CHASSIS_MOUSE_DECOUPLE_GAIN      0.85f    // 从 yaw 反馈里扣除的鼠标指令比例，防止手动瞄准时互抢
#define TUCHUAN_GYRO_CHASSIS_CMD_MAX_DELTA_DPS_PER_S  2400.0f  // 补偿命令每秒最大变化率；越小越稳，越大越跟手
#define TUCHUAN_GYRO_CHASSIS_TRACK_TAU_MS             12.0f    // 快速跟踪云台补偿基准角的时间常数；越小响应越快，越大越平滑
#define TUCHUAN_GYRO_CHASSIS_TRACK_SYNC_TRIGGER_DPS   18.0f    // 补偿目标突变时直接同步阈值；越小越容易在切档和反向时立即跟上
#define TUCHUAN_GYRO_CHASSIS_TRACK_MIN_RPM            0.35f    // 快速跟踪链路最小有效底盘转速；用于滤掉极小静止噪声
#define TUCHUAN_GYRO_CHASSIS_TARGET_BASE_DPS          TUCHUAN_GYRO_YAW_SPEED_DPS // 底盘目标自转基础角速度；开小陀螺瞬间预补偿先按它估算
#define TUCHUAN_GYRO_CHASSIS_TARGET_FF_HOLD_MS        10u        // 目标挡位前馈保持时间；当前先关闭目标前馈，恢复到只跟实测底盘转速
#define TUCHUAN_GYRO_CHASSIS_TARGET_CHANGE_DPS        8.0f      // 认为目标挡位变化的最小角速度差；越小越容易触发前馈
#define TUCHUAN_GYRO_CHASSIS_TARGET_FF_BLEND          0.0f      // 目标前馈混合占比；当前先关闭目标前馈，恢复到只跟实测底盘转速

// 云台输出量
gimbal_output_t gimbal_out = {0};
static volatile fp32 g_tuchuan_gyro_comp_speed_dps = 0.0f;
static volatile uint8_t g_tuchuan_gyro_enabled = 0u;
static volatile int8_t g_tuchuan_gyro_direction = 1;
static volatile uint8_t g_tuchuan_mouse_resync_pending = 0u;
/*
 * 小陀螺补偿专用 PID：
 * - 目标：让云台实际yaw角速度跟随“底盘反向补偿目标角速度”，降低纯增益方案下的低速漂移。
 * - 用法：在底盘补偿前馈基础上叠加 PID 修正量，尤其针对“鼠标不动时”的稳态误差。
 */
static pid_type_def g_tuchuan_chassis_comp_pid = {
    .mode = PID_POSITION,
        .Kp = 0.16f,      // 底盘补偿速度环比例；无鼠标时跟随底盘偏慢就增大，出现抖动/抢控制就减小
        .Ki = 0.003f,     // 底盘补偿速度环积分；长时间总差一点就增大，松手后拖尾或反冲就减小
    .Kd = 0.0f,           // 底盘补偿速度环微分；一般先不动，快速振荡明显时再少量增加
        .max_out = 28.0f, // 底盘补偿 PID 最大修正角速度；补偿不够可增大，瞬时过补偿可减小
        .max_iout = 8.0f  // 底盘补偿 PID 积分限幅；积分顶死就减小，静差长期消不掉可增大
};
#define TUCHUAN_GYRO_SPEED_SCALE_MIN               0.20f     // 纯小陀螺模式最小速度倍率；Ctrl 降速还太快就减小，最低速太慢就增大
#define TUCHUAN_GYRO_SPEED_SCALE_MAX               2.50f     // 纯小陀螺模式最大速度倍率；Shift 加速不够就增大，最高速太猛就减小
#define TUCHUAN_GYRO_SPEED_SCALE_STEP              0.05f     // 纯小陀螺模式每次长按重复的倍率步长；调速太粗减小，太慢增大
#define TUCHUAN_GYRO_SPEED_ADJ_LONGPRESS_TICKS     20u       // 纯小陀螺模式长按开始调速门槛；误触发就增大，想更快进入调速就减小
#define TUCHUAN_GYRO_SPEED_ADJ_REPEAT_TICKS        4u        // 纯小陀螺模式长按连发间隔；调速过快减小灵敏度就增大，调速太慢就减小
/*
 * 小陀螺滤波参数（单位：ms）
 * 1) CMD_FILTER_TAU_MS：对小陀螺目标角速度做一阶低通，抑制中低速下目标突变带来的抖动。
 * 2) FEEDBACK_FILTER_TAU_MS：对 yaw 速度反馈值(speed_rpm)做一阶低通，降低测速噪声导致的环路抖动。
 *
 * 调整方向：
 * - 数值增大：更稳、更“肉”，抖动更小但响应更慢。
 * - 数值减小：更灵敏，但中低速时更容易出现波动。
 */
#define TUCHUAN_GYRO_CMD_FILTER_TAU_MS             15.0f    // 纯小陀螺目标角速度低通时间常数；中低速发抖增大，响应慢减小
#define TUCHUAN_GYRO_FEEDBACK_FILTER_TAU_MS        25.0f     // Yaw 反馈速度低通时间常数；测速噪声导致齿感增大，制动迟钝减小
/*
 * 底盘角速度反向补偿参数：
 * 1) CHASSIS_CMD_FILTER_TAU_MS：补偿指令的一阶低通时间常数，越大越稳、越小越跟手。
 * 2) CHASSIS_MAX_COMP_DPS：限制补偿角速度上限，避免异常串口值导致瞬时大电流。
 * 3) RAD_TO_DPS：兼容旧接口（chassis_rad）的 rad/s -> deg/s 转换。
 * 4) CHASSIS_RPM_TO_DPS：新接口（USART1状态帧）的 rpm -> deg/s 转换。
 * 5) STATUS_TIMEOUT_MS：底盘状态帧超时阈值，超时后补偿输入按0处理。
 * 6) LOW/MID/HIGH：Ctrl/默认/Shift 三档补偿倍率。
 */
#define TUCHUAN_GYRO_CHASSIS_CMD_FILTER_TAU_MS     42.0f     // 底盘补偿命令低通时间常数；补偿一卡一卡增大，跟手不够减小
#define TUCHUAN_GYRO_CHASSIS_MAX_COMP_DPS          420.0f    // 底盘补偿角速度上限；异常时过猛减小，高速转动跟不上增大
#define TUCHUAN_GYRO_RAD_TO_DPS                    57.2957795f // 旧链路 rad/s 转 deg/s 系数；只有单位变化时才改
#define TUCHUAN_GYRO_CHASSIS_RPM_TO_DPS            6.0f      // 底盘 rpm 转 deg/s 系数；底盘上报单位变化或减速比变化时改
#define TUCHUAN_GYRO_CHASSIS_STATUS_TIMEOUT_MS     120u      // 底盘状态帧超时门限；链路偶发断续可增大，想更快判失联可减小
#define TUCHUAN_GYRO_CHASSIS_SPEED_LOW_SCALE       0.55f     // Ctrl 低速档倍率；低速档仍太快减小，太肉增大
#define TUCHUAN_GYRO_CHASSIS_SPEED_MID_SCALE       1.00f     // 默认中速档倍率；整体偏快减小，整体偏慢增大
#define TUCHUAN_GYRO_CHASSIS_SPEED_HIGH_SCALE      1.55f     // Shift 高速档倍率；高速档不够猛增大，太冲减小
/*
 * 底盘串口角速度融合参数（仅使用底盘实时rpm）：
 * 1) RPM_COMP_GAIN：rpm->dps总增益，决定补偿强度。
 * 2) RPM_FILTER_TAU_MS：rpm一阶低通时间常数，抑制测速毛刺导致的卡顿。
 * 3) RPM_DEADZONE_ENTER/EXIT：迟滞死区阈值，避免零附近来回抖动。
 * 4) NO_MOUSE_TRACK_GAIN：鼠标不动时的跟随增益，提升“相对静止”能力。
 */
#define TUCHUAN_GYRO_CHASSIS_RPM_COMP_GAIN         2.05f     // 底盘 rpm 到补偿角速度总增益；转起来跟不上增大，云台反向过头减小
#define TUCHUAN_GYRO_CHASSIS_RPM_FILTER_TAU_MS     50.0f     // 底盘 rpm 低通时间常数；低速卡顿/毛刺增大，转向跟随慢减小
#define TUCHUAN_GYRO_CHASSIS_RPM_DEADZONE_ENTER    1.80f     // 补偿进入死区阈值；零附近误触发增大，起补偿太晚减小
#define TUCHUAN_GYRO_CHASSIS_RPM_DEADZONE_EXIT     0.90f     // 补偿退出死区阈值；零附近来回抖动减小，拖尾太长增大
#define TUCHUAN_GYRO_CHASSIS_NO_MOUSE_TRACK_GAIN   1.35f     // 无鼠标输入时额外跟随增益；松手仍跟不上增大，松手过冲减小
/* 无鼠标输入时仅保留很小阈值，避免静止噪声引起微抖。 */
#define TUCHUAN_GYRO_CHASSIS_IDLE_MIN_COMP_RPM     0.35f     // 无鼠标时最小有效 rpm；静止微抖就增大，低速慢圈补偿断续就减小
/*
 * 小陀螺补偿速度闭环参数：
 * 1) PID_ENABLE_MOUSE_IDLE_ONLY：1=仅在“鼠标无输入”时启用PID修正，0=关闭外层速度PID修正。
 * 2) PID_MAX_ABS_OUTPUT_DPS：PID修正量幅值上限（单位deg/s），防止瞬时过补偿。
 * 3) YAW_FEEDBACK_RPM_TO_DPS：yaw反馈rpm转deg/s的系数。
 */
#define TUCHUAN_GYRO_CHASSIS_PID_ENABLE_MOUSE_IDLE_ONLY   0u        // 1=只在鼠标静止时开补偿 PID；想手动瞄准时完全避免 PID 干预可设 1
#define TUCHUAN_GYRO_CHASSIS_PID_MAX_ABS_OUTPUT_DPS       28.0f     // 底盘补偿 PID 修正限幅；修正不够增大，叠加后发冲减小
#define TUCHUAN_GYRO_YAW_FEEDBACK_RPM_TO_DPS              6.0f      // Yaw 电机 rpm 转 deg/s 系数；电机反馈单位或减速比变更时改
/*
 * 无鼠标位置回正参数（位置式补偿）：
 * 1) IDLE_POS_HOLD_ENABLE：1=启用无鼠标位置回正，0=关闭。
 * 2) IDLE_POS_LOCK_DELAY_MS：鼠标停止后等待该时间再锁定参考角，避免刚松手就抢控制。
 * 3) IDLE_POS_KP_DPS_PER_DEG：角度误差(°)转修正角速度(deg/s)的比例。
 * 4) IDLE_POS_MAX_DPS：位置回正输出上限，防止大误差瞬时冲击。
 * 5) IDLE_POS_ERR_DEADBAND_DEG：位置误差死区，减小微小抖动来回修正。
 * 6) IDLE_POS_CMD_FILTER_TAU_MS：位置回正角速度一阶低通时间常数。
 */
#define TUCHUAN_GYRO_IDLE_POS_HOLD_ENABLE            0u        // 无鼠标位置回正开关；松手希望自动锁住朝向就开，完全手感优先就关
#define TUCHUAN_GYRO_IDLE_POS_LOCK_DELAY_MS          10u      // 松手多久后开始锁角；刚松手就抢控制则增大，锁定太晚减小
#define TUCHUAN_GYRO_IDLE_POS_KP_DPS_PER_DEG         1.50f     // 角度误差转回正角速度比例；松手回正慢增大，微抖或回摆减小
#define TUCHUAN_GYRO_IDLE_POS_MAX_DPS                25.0f     // 回正最大角速度；回正太冲减小，偏角大时拉不回增大
#define TUCHUAN_GYRO_IDLE_POS_ERR_DEADBAND_DEG       0.25f     // 回正角度死区；准星附近来回修正增大，残余偏差明显减小
#define TUCHUAN_GYRO_IDLE_POS_CMD_FILTER_TAU_MS      50.0f     // 回正命令低通时间常数；回正抽动增大，回正太肉减小
/* 底盘正转速时默认让云台反向补偿（底盘正 -> 云台负） */
#define TUCHUAN_GYRO_CHASSIS_COMP_SIGN             (-1.0f)    // 补偿方向符号；若底盘正转时云台也同向跑，改成 +1.0f 试验
/* Yaw位置环PID*/
/* 底盘补偿尽量保持接近 1:1 的物理映射，先避免额外放大。 */
#undef TUCHUAN_GYRO_CHASSIS_RPM_COMP_GAIN
#define TUCHUAN_GYRO_CHASSIS_RPM_COMP_GAIN          1.08f
#undef TUCHUAN_GYRO_CHASSIS_NO_MOUSE_TRACK_GAIN
#define TUCHUAN_GYRO_CHASSIS_NO_MOUSE_TRACK_GAIN    1.00f
#undef TUCHUAN_GYRO_CHASSIS_PID_MAX_ABS_OUTPUT_DPS
#define TUCHUAN_GYRO_CHASSIS_PID_MAX_ABS_OUTPUT_DPS 6.0f
pid_type_def yaw_position_pid = {
    .mode = PID_POSITION,
    .Kp = 15.0f,
    .Ki = 0.3f,
    .Kd = 1.5f,
    .max_out = 180.0f,
    .max_iout = 50.0f
};
/* 位置保持控制器 */
position_hold_t yaw_position_hold = {
    .mode = GIMBAL_MANUAL,
    .hold_angle = 0.0f,
    .hold_start_time = 0,
    .hold_kp = 25.0f,
    .max_hold_current = 5000.0f
};
pid_type_def yaw_angle_pid=
{
    .mode = PID_POSITION,
    .Kp = 1.5f,        // Yaw 角度环比例；指向目标太慢增大，左右来回摆减小
    .Ki = 0.0f,        // Yaw 角度环积分；通常先保持 0，存在固定残差时再小幅增加
    .Kd = 0.0f,        // Yaw 角度环微分；转停过头或回摆时增大，噪声敏感就减小
    .max_out = 200.0f, // Yaw 角度环输出上限；大动作不够快增大，动作太猛减小
    .max_iout = 0.0f   // Yaw 角度环积分限幅；只有启用 Ki 后才需要一起调
};
pid_type_def yaw_speed_pid=
{
    .mode = PID_DELTA,
    .Kp = 250.0f,      // Yaw 速度环比例；电机发软跟不上增大，啸叫/振荡减小
    .Ki = 0.8f,        // Yaw 速度环积分；稳态转速差大增大，发热或拖尾明显减小
    .Kd = 0.0f,        // Yaw 速度环微分；一般先不动，快速抖动时再少量加入
    .max_out = 28000.0f, // Yaw 速度环输出限幅；打不满电流时增大，电流冲击大减小
    .max_iout = 500.0f // Yaw 速度环积分限幅；积分累积过大减小，长期无力可小幅增大
};
/* Pitch位置环PID*/
pid_type_def pitch_position_pid = {
    .mode = PID_POSITION,
    .Kp = 25.0f,           // 位置环比例系数
    .Ki = 0.0f,            // 位置环积分系数0
    .Kd = 0.0f,            // 位置环微分系数
    .max_out = 30.0f,    // 输出到角度环的最大值
    .max_iout = 5.0f
};
pid_type_def pitch_angle_pid=
{
    .mode = PID_POSITION,
    .Kp = 2.2f,        // Pitch 角度环比例；抬头低头慢增大，上下过冲减小
    .Ki = 0.02f,       // Pitch 角度环积分；长期抬不准时增大，停下后慢慢爬动减小
    .Kd = 0.12f,       // Pitch 角度环微分；点头回弹时增大，噪声放大则减小
    .max_out = 260.0f, // Pitch 角度环输出上限；大角度动作不够快增大，动作太硬减小
    .max_iout = 0.0f   // Pitch 角度环积分限幅；只有需要更多 Ki 时再一起调
};
pid_type_def pitch_speed_pid=
{
    .mode = PID_DELTA,
    .Kp = 190.0f,      // Pitch 速度环比例；俯仰跟手差增大，上下抖动或啸叫减小
    .Ki = 0.60f,       // Pitch 速度环积分；静态托不住或速度差大增大，发热/拖尾减小
    .Kd = 1.2f,        // Pitch 速度环微分；上下制动不住增大，噪声多或抖动减小
    .max_out = 28000.0f, // Pitch 速度环输出限幅；扭矩不够增大，冲击太大减小
    .max_iout = 350.0f // Pitch 速度环积分限幅；积分堆积明显减小，长期无力可微增
};
/* 位置保持控制器 */
position_hold_t pitch_position_hold = {
    .mode = GIMBAL_MANUAL,
    .hold_angle = 0.0f,
    .hold_start_time = 0,
    .hold_kp = 25.0f,
    .max_hold_current = 5000.0f
};

/*
 * Remote-control sticks use a dedicated, softer PID set so we can calm down
 * RC behavior without changing the current keymouse feel.
 */
static pid_type_def g_rc_yaw_position_pid = {
    .mode = PID_POSITION,
    .Kp = 16.0f,
    .Ki = 0.20f,
    .Kd = 1.0f,
    .max_out = 850.0f,
    .max_iout = 120.0f
};

static pid_type_def g_rc_yaw_angle_pid = {
    .mode = PID_POSITION,
    .Kp = 1.15f,
    .Ki = 0.0f,
    .Kd = 0.0f,
    .max_out = 160.0f,
    .max_iout = 0.0f
};

static pid_type_def g_rc_yaw_speed_pid = {
    .mode = PID_DELTA,
    .Kp = 210.0f,
    .Ki = 0.55f,
    .Kd = 0.0f,
    .max_out = 24000.0f,
    .max_iout = 400.0f
};

static pid_type_def g_rc_pitch_position_pid = {
    .mode = PID_POSITION,
    .Kp = 22.0f,
    .Ki = 0.0f,
    .Kd = 0.0f,
    .max_out = 850.0f,
    .max_iout = 180.0f
};

static pid_type_def g_rc_pitch_angle_pid = {
    .mode = PID_POSITION,
    .Kp = 1.75f,
    .Ki = 0.012f,
    .Kd = 0.08f,
    .max_out = 220.0f,
    .max_iout = 0.0f
};

static pid_type_def g_rc_pitch_speed_pid = {
    .mode = PID_DELTA,
    .Kp = 165.0f,
    .Ki = 0.45f,
    .Kd = 0.9f,
    .max_out = 24000.0f,
    .max_iout = 280.0f
};






/* Vision auto-aim dedicated angle PID (retuned from runtime log 20260323_072737). */
static pid_type_def g_vision_yaw_angle_pid = {
    .mode = PID_POSITION,
    .Kp = 2.02f,       // 视觉近距离 Yaw 比例；吸附慢增大，左右摆动减小
    .Ki = 0.012f,      // 视觉近距离 Yaw 积分；长期偏一边增大，回中拖尾减小
    .Kd = 16.8f,       // 视觉近距离 Yaw 微分；抑制快摆超调，噪声抖动大就减小
    .max_out = 300.0f, // 视觉近距离 Yaw 输出上限；锁目标不够快增大，切目标太猛减小
    .max_iout = 45.0f  // 视觉近距离 Yaw 积分限幅；积分顶死减小，静差大增大
};

static pid_type_def g_vision_pitch_angle_pid = {
    .mode = PID_POSITION,
    .Kp = 4.10f,       // 视觉近距离 Pitch 比例；上下吸附慢增大，点头过冲减小
    .Ki = 0.020f,      // 视觉近距离 Pitch 积分；长期抬不准增大，停下后缓慢爬动减小
    .Kd = 7.6f,        // 视觉近距离 Pitch 微分；上下回弹增大，噪声放大就减小
    .max_out = 280.0f, // 视觉近距离 Pitch 输出上限；大偏差抬头不够快增大，动作太猛减小
    .max_iout = 60.0f  // 视觉近距离 Pitch 积分限幅；积分饱和减小，静差大增大
};

static pid_type_def g_vision_yaw_angle_pid_far = {
    .mode = PID_POSITION,
    .Kp = 1.35f,       // 视觉远距离 Yaw 比例；远点吸附慢增大，远距离左右晃减小
    .Ki = 0.0f,        // 视觉远距离 Yaw 积分；一般先关，远距离长期偏差明显时再小幅开启
    .Kd = 21.5f,       // 视觉远距离 Yaw 微分；远距离防超调核心参数，出现噪声抖动就减小
    .max_out = 235.0f, // 视觉远距离 Yaw 输出上限；远点拉不过来增大，晃动太猛减小
    .max_iout = 30.0f  // 视觉远距离 Yaw 积分限幅；若开启 Ki 后积分过大就减小
};

static pid_type_def g_vision_pitch_angle_pid_far = {
    .mode = PID_POSITION,
    .Kp = 2.65f,       // 视觉远距离 Pitch 比例；远距离上下跟随慢增大，点头过冲减小
    .Ki = 0.004f,      // 视觉远距离 Pitch 积分；远距离慢性偏差增大，停下后慢慢飘减小
    .Kd = 11.8f,       // 视觉远距离 Pitch 微分；远距离上下回弹增大，噪声大减小
    .max_out = 210.0f, // 视觉远距离 Pitch 输出上限；远点抬头不够快增大，动作太冲减小
    .max_iout = 35.0f  // 视觉远距离 Pitch 积分限幅；积分饱和减小，静差大增大
};
static fp32 g_vision_x_error_lpf = 0.0f;
static fp32 g_vision_y_error_lpf = 0.0f;
static uint8_t g_vision_xy_far_active = 0u;
static fp32 g_vision_last_yaw_cmd = 0.0f;
static fp32 g_vision_last_pitch_cmd = 0.0f;
static uint8_t g_vision_small_gyro_comp_inited = 0u;
static uint32_t g_vision_small_gyro_last_tick = 0u;
static fp32 g_vision_small_gyro_chassis_rpm_lpf = 0.0f;
static uint8_t g_vision_small_gyro_deadzone_active = 0u;
static fp32 g_vision_small_gyro_cmd_lpf_dps = 0.0f;

#define VISION_SMALL_GYRO_COMP_STATUS_TIMEOUT_MS     120u
#define VISION_SMALL_GYRO_COMP_RPM_TO_DPS            6.0f
#define VISION_SMALL_GYRO_COMP_RPM_GAIN              1.08f
#define VISION_SMALL_GYRO_COMP_RPM_FILTER_TAU_MS     50.0f
#define VISION_SMALL_GYRO_COMP_RPM_DEADZONE_ENTER    1.80f
#define VISION_SMALL_GYRO_COMP_RPM_DEADZONE_EXIT     0.90f
#define VISION_SMALL_GYRO_COMP_MAX_DPS               420.0f
#define VISION_SMALL_GYRO_COMP_CMD_FILTER_TAU_MS     42.0f
#define VISION_SMALL_GYRO_COMP_CMD_MAX_DELTA_DPS_PER_S 2400.0f

static fp32 vision_clamp_delta(fp32 target, fp32 last, fp32 max_delta)
{
    fp32 delta = target - last;

    if (delta > max_delta)
    {
        delta = max_delta;
    }
    else if (delta < -max_delta)
    {
        delta = -max_delta;
    }

    return last + delta;
}

void Vision_Control_Gimbal_AngleError_Reset(void)
{
    PID_clear(&g_vision_yaw_angle_pid);
    PID_clear(&g_vision_pitch_angle_pid);
    PID_clear(&g_vision_yaw_angle_pid_far);
    PID_clear(&g_vision_pitch_angle_pid_far);
    g_vision_x_error_lpf = 0.0f;
    g_vision_y_error_lpf = 0.0f;
    g_vision_xy_far_active = 0u;
    g_vision_last_yaw_cmd = 0.0f;
    g_vision_last_pitch_cmd = 0.0f;
    g_vision_small_gyro_comp_inited = 0u;
    g_vision_small_gyro_last_tick = 0u;
    g_vision_small_gyro_chassis_rpm_lpf = 0.0f;
    g_vision_small_gyro_deadzone_active = 0u;
    g_vision_small_gyro_cmd_lpf_dps = 0.0f;
}
/* 编码器到角度的转换函数 */
float encoder_to_angle(int16_t encoder)
{
    return (encoder * 360.0f) / 8192.0f;  // 假设电机是8192线
}

/* 获取当前Pitch角度 */
float get_pitch_angle(const motor_measure_t *motor)
{
    if (!motor) return 0.0f;
    return encoder_to_angle(motor->ecd);
}
/*获取当前Yaw角度*/
float get_yaw_angle(const motor_measure_t *motor)
{
    if (!motor) return 0.0f;
    return encoder_to_angle(motor->ecd);
}
/* 将角度限制在 0° 到 360° 范围内，适用于一般角度表示。 */
static fp32 wrap_angle_deg_360(fp32 angle)
{
    while (angle >= ANGLE_360)
    {
        angle -= ANGLE_360;
    }
    while (angle < 0.0f)
    {
        angle += ANGLE_360;
    }
    return angle;
}
/* 将角度限制在 -180° 到 180° 范围内，适用于计算最短旋转路径。 */
static fp32 wrap_angle_deg_180(fp32 angle)
{
    while (angle > ANGLE_180)
    {
        angle -= ANGLE_360;
    }
    while (angle < -ANGLE_180)
    {
        angle += ANGLE_360;
    }
    return angle;
}
/* 视觉小陀螺专用底盘转速反向补偿前馈，基于底盘实时rpm，带死区和低通滤波。 */
static fp32 vision_small_gyro_comp_ff_rpm(void)
{
    uint32_t now_tick;
    uint32_t dt_ms;
    uint32_t chassis_status_tick;
    uint32_t chassis_status_age_ms;
    fp32 chassis_speed_rpm_raw;
    fp32 chassis_speed_rpm_for_comp;
    fp32 comp_cmd_alpha;
    fp32 comp_raw_dps;
    fp32 comp_cmd_last_dps;
    fp32 comp_cmd_max_delta_dps;

    fp32 yaw_comp_ff_rpm = 0.0f;

    if (get_tuchuan_gimbal_gyro_enabled() == 0u)
    {
        g_vision_small_gyro_comp_inited = 0u;
        g_vision_small_gyro_last_tick = 0u;
        g_vision_small_gyro_chassis_rpm_lpf = 0.0f;
        g_vision_small_gyro_deadzone_active = 0u;
        g_vision_small_gyro_cmd_lpf_dps = 0.0f;
        return 0.0f;
    }

    now_tick = HAL_GetTick();
    if ((g_vision_small_gyro_comp_inited == 0u) ||
        (g_vision_small_gyro_last_tick == 0u) ||
        (now_tick <= g_vision_small_gyro_last_tick))
    {
        dt_ms = 10u;
    }
    else
    {
        dt_ms = now_tick - g_vision_small_gyro_last_tick;
        if (dt_ms > 20u)
        {
            dt_ms = 20u;
        }
    }
    g_vision_small_gyro_last_tick = now_tick;

    if (g_vision_small_gyro_comp_inited == 0u)
    {
        g_vision_small_gyro_comp_inited = 1u;
        g_vision_small_gyro_chassis_rpm_lpf = 0.0f;
        g_vision_small_gyro_deadzone_active = 0u;
        g_vision_small_gyro_cmd_lpf_dps = 0.0f;
    }

    chassis_status_tick = get_usart1_chassis_status_update_tick();
    if ((chassis_status_tick != 0u) && (now_tick >= chassis_status_tick))
    {
        chassis_status_age_ms = now_tick - chassis_status_tick;
    }
    else
    {
        chassis_status_age_ms = 0xFFFFFFFFu;
    }

    if (chassis_status_age_ms > VISION_SMALL_GYRO_COMP_STATUS_TIMEOUT_MS)
    {
        g_vision_small_gyro_chassis_rpm_lpf = 0.0f;
        g_vision_small_gyro_deadzone_active = 0u;
        g_vision_small_gyro_cmd_lpf_dps = 0.0f;
        return 0.0f;
    }

    chassis_speed_rpm_raw = (fp32)get_usart1_chassis_gyro_speed_rpm();

    /* 先去掉底盘转速低通，直接使用实时转速提高视觉小陀螺前馈响应。 */
    g_vision_small_gyro_chassis_rpm_lpf = chassis_speed_rpm_raw;

    if (g_vision_small_gyro_deadzone_active == 0u)
    {
        if (fabs(g_vision_small_gyro_chassis_rpm_lpf) >= VISION_SMALL_GYRO_COMP_RPM_DEADZONE_ENTER)
        {
            g_vision_small_gyro_deadzone_active = 1u;
        }
    }
    else if (fabs(g_vision_small_gyro_chassis_rpm_lpf) <= VISION_SMALL_GYRO_COMP_RPM_DEADZONE_EXIT)
    {
        g_vision_small_gyro_deadzone_active = 0u;
    }

    if (g_vision_small_gyro_deadzone_active != 0u)
    {
        chassis_speed_rpm_for_comp = g_vision_small_gyro_chassis_rpm_lpf;
    }
    else
    {
        chassis_speed_rpm_for_comp = 0.0f;
    }

    comp_raw_dps = chassis_speed_rpm_for_comp *
                   VISION_SMALL_GYRO_COMP_RPM_TO_DPS *
                   VISION_SMALL_GYRO_COMP_RPM_GAIN *
                   TUCHUAN_GYRO_CHASSIS_COMP_SIGN;

    if (comp_raw_dps > VISION_SMALL_GYRO_COMP_MAX_DPS)
    {
        comp_raw_dps = VISION_SMALL_GYRO_COMP_MAX_DPS;
    }
    else if (comp_raw_dps < -VISION_SMALL_GYRO_COMP_MAX_DPS)
    {
        comp_raw_dps = -VISION_SMALL_GYRO_COMP_MAX_DPS;
    }

    comp_cmd_alpha = ((fp32)dt_ms) / (VISION_SMALL_GYRO_COMP_CMD_FILTER_TAU_MS + (fp32)dt_ms);
    if (comp_cmd_alpha < 0.0f)
    {
        comp_cmd_alpha = 0.0f;
    }
    else if (comp_cmd_alpha > 1.0f)
    {
        comp_cmd_alpha = 1.0f;
    }

    comp_cmd_last_dps = g_vision_small_gyro_cmd_lpf_dps;
    g_vision_small_gyro_cmd_lpf_dps +=
        comp_cmd_alpha * (comp_raw_dps - g_vision_small_gyro_cmd_lpf_dps);

    comp_cmd_max_delta_dps =
        VISION_SMALL_GYRO_COMP_CMD_MAX_DELTA_DPS_PER_S * ((fp32)dt_ms / 1000.0f);
    if (comp_cmd_max_delta_dps < 0.5f)
    {
        comp_cmd_max_delta_dps = 0.5f;
    }
    g_vision_small_gyro_cmd_lpf_dps =
        vision_clamp_delta(g_vision_small_gyro_cmd_lpf_dps, comp_cmd_last_dps, comp_cmd_max_delta_dps);

    if ((chassis_speed_rpm_for_comp == 0.0f) && (fabs(g_vision_small_gyro_cmd_lpf_dps) < 0.3f))
    {
        g_vision_small_gyro_cmd_lpf_dps = 0.0f;
    }

    yaw_comp_ff_rpm =
        (g_vision_small_gyro_cmd_lpf_dps / TUCHUAN_GYRO_YAW_FEEDBACK_RPM_TO_DPS) *
        VISION_SMALL_GYRO_YAW_FF_GAIN;

    return yaw_comp_ff_rpm;
}

// 俯仰轴鼠标输入形状函数，提供更细腻的微调同时保留大动作速度。输入为鼠标Y轴偏移和时间增量，输出为角度步进。
static fp32 shape_tuchuan_pitch_mouse_step(int16_t mouse_y, uint32_t dt_ms)
{
    fp32 abs_y;
    fp32 effective_y;
    fp32 ramp_t;
    fp32 ramp_ease;
    fp32 scale;
    fp32 shaped_input;
    fp32 step;

    if (mouse_y == 0)
    {
        return 0.0f;
    }

    abs_y = (fp32)abs(mouse_y);
    effective_y = abs_y - (fp32)TUCHUAN_MOUSE_DEADZONE_Y;
    if (effective_y <= 0.0f)
    {
        return 0.0f;
    }

    ramp_t = effective_y / TUCHUAN_MOUSE_PITCH_FINE_RANGE;
    if (ramp_t < 0.0f)
    {
        ramp_t = 0.0f;
    }
    else if (ramp_t > 1.0f)
    {
        ramp_t = 1.0f;
    }

    /* 平滑恢复到全增益，让俯仰微调更细，同时保留大动作速度。 */
    ramp_ease = ramp_t * ramp_t * (3.0f - 2.0f * ramp_t);
    scale = TUCHUAN_MOUSE_PITCH_FINE_MIN_SCALE +
            (1.0f - TUCHUAN_MOUSE_PITCH_FINE_MIN_SCALE) * ramp_ease;
    shaped_input = effective_y * scale + (fp32)TUCHUAN_MOUSE_DEADZONE_Y * ramp_t;
    if (shaped_input > abs_y)
    {
        shaped_input = abs_y;
    }

    step = shaped_input * TUCHUAN_MOUSE_PITCH_GAIN * ((fp32)dt_ms / 10.0f);
    if (mouse_y < 0)
    {
        step = -step;
    }

    return step;
}

static fp32 vision_pitch_gravity_ff_current(const motor_measure_t *pitch_motor)
{
    fp32 pitch_angle_deg;
    fp32 angle_ratio;

    if (pitch_motor == NULL)
    {
        return 0.0f;
    }

    pitch_angle_deg = encoder_to_angle(pitch_motor->ecd);
    angle_ratio = (PITCH_ANGLE_MAX - pitch_angle_deg) / (PITCH_ANGLE_MAX - PITCH_ANGLE_MIN);

    if (angle_ratio < 0.0f)
    {
        angle_ratio = 0.0f;
    }
    else if (angle_ratio > 1.0f)
    {
        angle_ratio = 1.0f;
    }

    return VISION_XY_PITCH_GRAVITY_FF_BASE_CURRENT +
           VISION_XY_PITCH_GRAVITY_FF_GAIN_CURRENT * angle_ratio;
}


static fp32 pitch_target_guard_inside(const motor_measure_t *pitch_motor, fp32 target_angle)
{
    const fp32 safe_min_angle = encoder_to_angle(PITCH_SAFE_MIN);
    const fp32 safe_max_angle = encoder_to_angle(PITCH_SAFE_MAX);
    const fp32 recover_min_angle = encoder_to_angle(PITCH_SAFE_MIN + 10);
    const fp32 recover_max_angle = encoder_to_angle(PITCH_SAFE_MAX - 10);
    int16_t encoder;

    if (pitch_motor == NULL)
    {
        return target_angle;
    }

    encoder = pitch_motor->ecd;

    if (target_angle < safe_min_angle)
    {
        target_angle = safe_min_angle;
    }
    if (target_angle > safe_max_angle)
    {
        target_angle = safe_max_angle;
    }

    if (encoder <= PITCH_SAFE_MIN)
    {
        if (target_angle < recover_min_angle)
        {
            target_angle = recover_min_angle;
        }
    }
    else if (encoder >= PITCH_SAFE_MAX)
    {
        if (target_angle > recover_max_angle)
        {
            target_angle = recover_max_angle;
        }
    }

    return target_angle;
}
/* 设置云台模式 */
void set_gimbal_mode(gimbal_mode_t mode)
{
    if (pitch_position_hold.mode == mode) return;
    
    char buffer[64];
    
    pitch_position_hold.mode = mode;
    
    if (mode == GIMBAL_POSITION_HOLD) {
        // 记录当前位置
        const motor_measure_t *pitch_motor = get_pitch_gimbal_motor_measure_point();
        pitch_position_hold.hold_angle = get_pitch_angle(pitch_motor);
        pitch_position_hold.hold_start_time = HAL_GetTick();
        
        // 重置位置环PID积分项
        pitch_position_pid.Iout = 0;
        
        // snprintf(buffer, sizeof(buffer), "进入位置保持模式，保持角度: %.1f°\r\n", 
        //         pitch_position_hold.hold_angle);
    } else if (mode == GIMBAL_MANUAL) {
        // snprintf(buffer, sizeof(buffer), "进入手动模式\r\n");
    }
    
    // HAL_UART_Transmit(&huart6, (uint8_t *)buffer, strlen(buffer), HAL_MAX_DELAY);
}


/* 检查是否需要切换到位置保持模式 */
static gimbal_mode_t check_mode_transition(const RC_ctrl_t *rc_data, 
                                          const motor_measure_t *pitch_motor)
{
    static uint32_t last_input_time = 0;
    uint32_t current_time = HAL_GetTick();
    
    /* 检查遥控器输入 */
    int16_t pitch_input = -rc_data->rc.ch[1];  // Pitch通道
    
    /* 判断是否有有效输入（排除死区） */
    int has_input = (abs(pitch_input) > CHANNEL_DEADZONE);
    
    if (has_input) {
        last_input_time = current_time;
        return GIMBAL_MANUAL;
    }

    /* 如果无输入超过100ms，切换到位置保持 */
    if (current_time - last_input_time > 100) 
    {  
        return GIMBAL_POSITION_HOLD;
    }
    
    return GIMBAL_MANUAL;
}

/* 位置保持模式控制函数 */
static void position_hold_control(const motor_measure_t *pitch_motor, 
                                 float *pitch_angle_error)
{
    if (!pitch_motor) return;
    
    /* 获取当前角度 */
    float current_angle = get_pitch_angle(pitch_motor);
    
    /* 计算位置误差 */
    float position_error = pitch_position_hold.hold_angle - current_angle;
    
    /* 角度归一化处理 */
    while (position_error > 180.0f) position_error -= 360.0f;
    while (position_error < -180.0f) position_error += 360.0f;
    
    /* 位置环PID计算 - 输出目标角度误差（给角度环） */
    fp32 hold_target_angle = pitch_target_guard_inside(pitch_motor, pitch_position_hold.hold_angle);
    pitch_position_hold.hold_angle = hold_target_angle;

    float target_angle_error = PID_calc(&pitch_position_pid, current_angle, hold_target_angle);
    

    /* 限制最大位置误差输出 */
    float max_allowed_error = 30.0f;  // 最大30度误差
    if (target_angle_error > max_allowed_error) {
        target_angle_error = max_allowed_error;
    } else if (target_angle_error < -max_allowed_error) {
        target_angle_error = -max_allowed_error;
    }
    
    /* 更新角度误差 */
    *pitch_angle_error = target_angle_error;
}

/* 安全模式控制函数 */
static void safe_mode_control(const motor_measure_t *pitch_motor, 
                             float *pitch_angle_error)
{
    if (!pitch_motor) return;
    
    /* 在安全模式下，强制将目标角度设定在安全位置 */
    int16_t encoder = pitch_motor->ecd;
    float safe_angle = 0.0f;
    
    if (encoder <= PITCH_ENCODER_MIN + 100) {
        // 接近下限，设定安全角度为当前位置+10度
        safe_angle = get_pitch_angle(pitch_motor) + 10.0f;
    } else if (encoder >= PITCH_ENCODER_MAX - 100) {
        // 接近上限，设定安全角度为当前位置-10度
        safe_angle = get_pitch_angle(pitch_motor) - 10.0f;
    }
    
    /* 计算角度误差 */
    float current_angle = get_pitch_angle(pitch_motor);
    *pitch_angle_error = safe_angle - current_angle;
    
    /* 限制误差范围 */
    if (*pitch_angle_error > 15.0f) *pitch_angle_error = 15.0f;
    if (*pitch_angle_error < -15.0f) *pitch_angle_error = -15.0f;
}






/* 设置云台模式 */
void set_gimbal_yaw_mode(gimbal_mode_t mode)
{
    if (yaw_position_hold.mode == mode) return;
    yaw_position_hold.mode = mode;
    
    if (mode == GIMBAL_POSITION_HOLD) {
        // 记录当前位置
        const motor_measure_t *yaw_motor = get_yaw_gimbal_motor_measure_point();
        yaw_position_hold.hold_angle = get_yaw_angle(yaw_motor);
        yaw_position_hold.hold_start_time = HAL_GetTick();

        // yaw_position_hold.hold_angle);
    } else if (mode == GIMBAL_MANUAL) {

    }
    else if(mode == GIMBAL_GYRO)
    {

    }

}


/* 检查是否需要切换到位置保持模式 */
static gimbal_mode_t check_mode_yaw_transition(const RC_ctrl_t *rc_data, 
                                          const motor_measure_t *yaw_motor)
{
    static uint32_t last_input_time = 0;
    uint32_t current_time = HAL_GetTick();
    
    /* 检查遥控器输入 */
    int16_t yaw_input = -rc_data->rc.ch[0];  // Yaw通道
    
    /* 判断是否有有效输入（排除死区） */
    int has_input = (abs(yaw_input) > CHANNEL_DEADZONE);
    
    if (has_input) {
        last_input_time = current_time;
        if(abs(rc_data->rc.ch[4]) < 10 )//如果没有旋转指令输入，切换到手动模式
        {
            return GIMBAL_MANUAL;
        }
        else 
        {
            return GIMBAL_GYRO;       //如果有旋转指令输入，开启小陀螺模式
        }
        
    }

    /* 如果无输入超过100ms，切换到位置保持 */
    if (current_time - last_input_time > 100) 
    {  
        return GIMBAL_POSITION_HOLD;
    }
    
    return GIMBAL_MANUAL;
}

/* 位置保持模式控制函数 */
static void yaw_position_hold_control(const motor_measure_t *yaw_motor, 
                                 float *yaw_angle_error)
{
    char buffer[64];
    if (!yaw_motor) return;
    
    /* 获取当前角度 */
    float current_angle = get_yaw_angle(yaw_motor);
    
    /* 计算位置误差 */
    float position_error = yaw_position_hold.hold_angle - current_angle;
    
    /* 角度归一化处理 */
    while (position_error > 180.0f) position_error -= 360.0f;
    while (position_error < -180.0f) position_error += 360.0f;
    
    /* 位置环PID计算 - 输出目标角度误差（给角度环） */
    float target_angle_error = PID_calc(&yaw_position_pid, current_angle, 
                                        yaw_position_hold.hold_angle);
                                    
    /* 限制最大位置误差输出 */
    float max_allowed_error = 30.0f;  // 最大30度误差
    if (target_angle_error > max_allowed_error) {
        target_angle_error = max_allowed_error;
    } else if (target_angle_error < -max_allowed_error) {
        target_angle_error = -max_allowed_error;
    }
    
    /* 更新角度误差 */
    *yaw_angle_error = target_angle_error;
}








/* 临时测试代码：绕过角度环，直接给速度环设定值调PID用的可以删了 */
void Test_Pitch_Speed_Control(int16_t speed_set)
{
    const motor_measure_t *yaw_motor = get_yaw_gimbal_motor_measure_point();
    
    /* 测试不同的目标速度 */
    static float test_speed = 0;
    static int direction = 1;
    
    // test_speed += direction * 100.0f;  // 每次增加100RPM
    
    // if(test_speed > 2000.0f) {
    //     direction = -1;
    // } else if(test_speed < -2000.0f) {
    //     direction = 1;
    // }
    test_speed = speed_set;
    float speed_output = PID_calc(&yaw_speed_pid, yaw_motor->speed_rpm, test_speed);
    CAN_cmd_gimbal((int16_t)speed_output, 0, 0,0);
        /* 打印速度信息 */
        char buffer[64];
        // snprintf(buffer, sizeof(buffer), "%.0f,%d,%d\r\n",
        //         test_speed, yaw_motor->speed_rpm,yaw_motor->given_current);
        // HAL_UART_Transmit(&huart6, (uint8_t *)buffer, strlen(buffer), HAL_MAX_DELAY);
}


/* 云台控制模块初始化函数 */
void Control_Gimbal_Init(void)
{
    BSP_CAN_Init();
    remote_control_init();
}
/* 获取小陀螺补偿速度，单位度每秒 */
float get_tuchuan_gimbal_gyro_comp_speed(void)
{
    return g_tuchuan_gyro_comp_speed_dps;
}
/* 获取小陀螺补偿是否启用状态 */
uint8_t get_tuchuan_gimbal_gyro_enabled(void)
{
    return g_tuchuan_gyro_enabled;
}
/* 重置小陀螺补偿状态，通常在云台重启或检测到陀螺数据异常时调用 */
void reset_tuchuan_gimbal_gyro_link(void)
{
    g_tuchuan_gyro_comp_speed_dps = 0.0f;
    g_tuchuan_gyro_enabled = 0u;
    g_tuchuan_gyro_direction = 1;
    g_tuchuan_mouse_resync_pending = 1u;
    PID_clear(&g_tuchuan_chassis_comp_pid);
}
/* 请求云台陀螺仪数据重新同步，通常在检测到数据异常或丢失时调用 */
void request_tuchuan_mouse_gimbal_resync(void)
{
    g_tuchuan_mouse_resync_pending = 1u;
}

/*@brief  云台控制主循环函数
 *         根据遥控器输入和当前云台状态，计算目标角度和速度，并发送控制命令
    *         包含模式切换逻辑：手动控制、小陀螺、位置保持
    * 最早的遥控控制云台函数，比后来鼠标稳很多
 */
void RC_Control_Gimbal(void)
{
    // 控制循环代码
    const RC_ctrl_t *rc_data = get_remote_control_point();
    const motor_measure_t *yaw_motor = get_yaw_gimbal_motor_measure_point();
    const motor_measure_t *pitch_motor = get_pitch_gimbal_motor_measure_point();
    char buffer[64];
    float wz = -rc_data->rc.ch[4] * 0.1; 
    
    fp32 yaw_angle_out = 0.0f, yaw_speed_out = 0.0f;
    pid_type_def saved_yaw_position_pid = yaw_position_pid;
    pid_type_def saved_yaw_angle_pid = yaw_angle_pid;
    pid_type_def saved_yaw_speed_pid = yaw_speed_pid;
    pid_type_def saved_pitch_position_pid = pitch_position_pid;
    pid_type_def saved_pitch_angle_pid = pitch_angle_pid;
    pid_type_def saved_pitch_speed_pid = pitch_speed_pid;

    yaw_position_pid = g_rc_yaw_position_pid;
    yaw_angle_pid = g_rc_yaw_angle_pid;
    yaw_speed_pid = g_rc_yaw_speed_pid;
    pitch_position_pid = g_rc_pitch_position_pid;
    pitch_angle_pid = g_rc_pitch_angle_pid;
    pitch_speed_pid = g_rc_pitch_speed_pid;

    // 将yaw云台角度规整到0-360度范围内
    fp32 yaw_angle = (yaw_motor->ecd * ANGLE_360) / MOTOR_ECD_RANGE; // 使用浮点数表示yaw角度
    int16_t yaw_channel_input = rc_data->rc.ch[0]; // 获取遥控器通道0的输入值
    fp32 yaw_angle_set = yaw_angle - (yaw_channel_input * CONTROL_SENSITIVITY); // 根据输入值调整目标角度

    // 计算角度差，考虑360°的周期性
    fp32 yaw_angle_error = yaw_angle_set - yaw_angle;
    if (yaw_angle_error > ANGLE_180) yaw_angle_error -= ANGLE_360;
    else if (yaw_angle_error < -ANGLE_180) yaw_angle_error += ANGLE_360;

    gimbal_mode_t new_yaw_mode = check_mode_yaw_transition(rc_data, yaw_motor);
    // 模式切换
    if (new_yaw_mode != yaw_position_hold.mode) {
        set_gimbal_yaw_mode(new_yaw_mode);
    }
    switch (yaw_position_hold.mode) {   
        case GIMBAL_MANUAL:
            {
                // 手动模式 - 遥控器控制
                int16_t yaw_input = rc_data->rc.ch[0];
                
                // 死区处理
                if (abs(yaw_input) < CHANNEL_DEADZONE) yaw_input = 0;
                
                // 获取当前角度
                fp32 yaw_angle = get_yaw_angle(yaw_motor);
                
                // 计算目标角度
                fp32 yaw_target_angle = yaw_angle - (yaw_input * CONTROL_SENSITIVITY);
                
                // 计算角度误差
                yaw_angle_error = yaw_target_angle - yaw_angle;
                
                // 角度归一化
                while (yaw_angle_error > 180.0f) yaw_angle_error -= 360.0f;
                while (yaw_angle_error < -180.0f) yaw_angle_error += 360.0f;
                yaw_angle_out = PID_calc(&yaw_angle_pid, 0.0f, yaw_angle_error); // 设定角度误差  
                yaw_speed_out = PID_calc(&yaw_speed_pid, yaw_motor->speed_rpm, yaw_angle_out);
            }
            break;
        case GIMBAL_GYRO:
            {
                // 小陀螺模式 - 根据旋转指令调整目标角度
                int16_t yaw_input = rc_data->rc.ch[0];
                int16_t rotation_input = rc_data->rc.ch[4]; // 旋转指令
                
                // 死区处理
                if (abs(yaw_input) < CHANNEL_DEADZONE) yaw_input = 0;
                if (abs(rotation_input) < CHANNEL_DEADZONE) rotation_input = 0;
                
                // 获取当前角度
                fp32 yaw_angle = get_yaw_angle(yaw_motor);
                
                // 根据旋转输入调整目标角度
                fp32 yaw_target_angle = yaw_angle - (yaw_input * CONTROL_SENSITIVITY) + (rotation_input * 0.1f);
                
                // 计算角度误差
                yaw_angle_error = yaw_target_angle - yaw_angle;
                
                // 角度归一化
                while (yaw_angle_error > 180.0f) yaw_angle_error -= 360.0f;
                while (yaw_angle_error < -180.0f) yaw_angle_error += 360.0f;
                
                yaw_speed_out = PID_calc(&yaw_speed_pid, yaw_motor->speed_rpm, chassis_rad);
            }
            break;    
        case GIMBAL_POSITION_HOLD:
            // 位置保持模式
            yaw_position_hold_control(yaw_motor, &yaw_angle_error);
            yaw_angle_out = PID_calc(&yaw_angle_pid, 0.0f, yaw_angle_error); // 设定角度误差  
            yaw_speed_out = PID_calc(&yaw_speed_pid, yaw_motor->speed_rpm, yaw_angle_out);
            break;
    }
        // yaw_angle_out = PID_calc(&yaw_angle_pid, 0.0f, yaw_angle_error); // 设定角度误差  
        // yaw_speed_out = PID_calc(&yaw_speed_pid, yaw_motor->speed_rpm, yaw_angle_out);



    // 将pitch云台角度规整到0-360度范围内
    fp32 pitch_angle = (pitch_motor->ecd * ANGLE_360) / MOTOR_ECD_RANGE; // 使用浮点数表示pitch角度
    int16_t pitch_channel_input = -rc_data->rc.ch[1]; // 获取遥控器通道1的输入值


     /* 检查当前编码器位置是否安全 */
     int16_t pitch_encoder = pitch_motor->ecd;
     static uint8_t limit_warning = 0;
     
     if (pitch_encoder <= PITCH_SAFE_MIN) {
         // 接近下限，强制停止向负方向运动
         if (pitch_channel_input > 0) {  // 正输入会使角度减小，需要限制
             pitch_channel_input = 0;
             if (!limit_warning) {
                 
                 limit_warning = 1;
             }
         } else {
             limit_warning = 0;
         }
     } else if (pitch_encoder >= PITCH_SAFE_MAX) {
         // 接近上限，强制停止向正方向运动
         if (pitch_channel_input < 0) {  // 负输入会使角度增加，需要限制
             pitch_channel_input = 0;
             if (!limit_warning) {
                 
                 limit_warning = 1;
             }
         } else {
             limit_warning = 0;
         }
     } else {
         limit_warning = 0;
     }


    fp32 pitch_angle_set = pitch_angle - (pitch_channel_input * CONTROL_SENSITIVITY * 0.5); // 根据输入值调整目标角度

    // 计算角度差，考虑360°的周期性
    fp32 pitch_angle_error = pitch_angle_set - pitch_angle;
    if (pitch_angle_error > ANGLE_180) pitch_angle_error -= ANGLE_360;
    else if (pitch_angle_error < -ANGLE_180) pitch_angle_error += ANGLE_360;


    /* 2. Pitch轴控制模式判断 */
    gimbal_mode_t new_mode = check_mode_transition(rc_data, pitch_motor);
    
    // 模式切换
    if (new_mode != pitch_position_hold.mode) {
        set_gimbal_mode(new_mode);
    }

    /* 3. 根据模式计算Pitch角度误差 */
    switch (pitch_position_hold.mode) {
        case GIMBAL_MANUAL:
            {
                // 手动模式 - 遥控器控制
                int16_t pitch_input = -rc_data->rc.ch[1];
                
                // 死区处理
                if (abs(pitch_input) < CHANNEL_DEADZONE) pitch_input = 0;
                
                // 获取当前角度
                fp32 pitch_angle = get_pitch_angle(pitch_motor);
                
                // 计算目标角度（添加限位检查）
                fp32 pitch_target_angle = pitch_angle - (pitch_input * CONTROL_SENSITIVITY);
                pitch_target_angle = pitch_target_guard_inside(pitch_motor, pitch_target_angle);
                
                // 计算角度误差
                pitch_angle_error = pitch_target_angle - pitch_angle;
                
                // 角度归一化
                while (pitch_angle_error > 180.0f) pitch_angle_error -= 360.0f;
                while (pitch_angle_error < -180.0f) pitch_angle_error += 360.0f;
            }
            break;
            
        case GIMBAL_POSITION_HOLD:
            // 位置保持模式
            position_hold_control(pitch_motor, &pitch_angle_error);
            break;
            
        case GIMBAL_SAFE:
            // 安全模式
            safe_mode_control(pitch_motor, &pitch_angle_error);
            break;
    }


    fp32 pitch_angle_out = PID_calc(&pitch_angle_pid, 0.0f, pitch_angle_error); // 设定角度误差
    fp32 pitch_speed_out = PID_calc(&pitch_speed_pid, pitch_motor->speed_rpm, pitch_angle_out);

    

    gimbal_out.yaw   = (int16_t)yaw_speed_out;
    // 调用限幅保护函数
    // gimbal_out.pitch =(int16_t)pitch_speed_out;
    gimbal_out.pitch = pitch_limit_protection((int16_t)pitch_speed_out, pitch_motor, pitch_angle_error);

    g_rc_yaw_position_pid = yaw_position_pid;
    g_rc_yaw_angle_pid = yaw_angle_pid;
    g_rc_yaw_speed_pid = yaw_speed_pid;
    g_rc_pitch_position_pid = pitch_position_pid;
    g_rc_pitch_angle_pid = pitch_angle_pid;
    g_rc_pitch_speed_pid = pitch_speed_pid;

    yaw_position_pid = saved_yaw_position_pid;
    yaw_angle_pid = saved_yaw_angle_pid;
    yaw_speed_pid = saved_yaw_speed_pid;
    pitch_position_pid = saved_pitch_position_pid;
    pitch_angle_pid = saved_pitch_angle_pid;
    pitch_speed_pid = saved_pitch_speed_pid;
}


/**
 * @brief       图传鼠标控制云台(底盘角速度反向补偿版)
 * @note        1) F键：开启/关闭底盘角速度反向补偿（仅开关）
 *              2) G键：切换小陀螺方向，并同步刷新目标前馈
 *              3) Shift/Ctrl：高/低速档，按住即生效，松开回默认中速档
 *              4) 鼠标仍用于微调瞄准，补偿优先跟目标挡位，实测追上后切回实时值
 * @retval      none    
 */
void Tuchuan_Mouse_Control_Gimbal_Gyro_ChassisComp(void)
{
    const usart6_tuchuan_remote_control_t *tuchuan_rc_data = get_usart6_tuchuan_remote_control_point();
    const motor_measure_t *yaw_motor = get_yaw_gimbal_motor_measure_point();
    const motor_measure_t *pitch_motor = get_pitch_gimbal_motor_measure_point();

    static uint8_t target_inited = 0u;
    static uint8_t mouse_control_started = 0u;
    static uint8_t last_f_pressed = 0u;
    static uint8_t last_g_pressed = 0u;
    static uint32_t last_tick = 0u;
    static fp32 yaw_comp_base_angle = 0.0f;
    static fp32 yaw_mouse_offset_angle = 0.0f;
    static fp32 pitch_target_angle = 0.0f;
    static fp32 chassis_comp_cmd_lpf_dps = 0.0f;
    static fp32 yaw_comp_track_dps = 0.0f;
    static fp32 yaw_speed_feedback_lpf_rpm = 0.0f;
    static fp32 yaw_mouse_cmd_lpf_dps = 0.0f;
    static fp32 chassis_speed_rpm_lpf = 0.0f;
    static uint8_t chassis_rpm_deadzone_active = 0u;
    static uint8_t idle_pos_hold_active = 0u;
    static uint32_t idle_pos_last_input_tick = 0u;
    static fp32 idle_pos_hold_ref_angle = 0.0f;
    static fp32 idle_pos_hold_cmd_lpf_dps = 0.0f;
    static fp32 last_yaw_comp_track_target_dps = 0.0f;
    static fp32 last_gyro_target_cmd_dps = 0.0f;
    static uint16_t gyro_target_ff_hold_ms = 0u;
    static uint8_t last_gyro_enabled_state = 0u;

    uint16_t keyboard_value;
    uint8_t f_pressed;
    uint8_t g_pressed;
    uint8_t shift_pressed;
    uint8_t ctrl_pressed;
    uint32_t now_tick;
    uint32_t dt_ms;
    int16_t mouse_x;
    int16_t mouse_y;
    int16_t abs_mouse_x;
    int16_t abs_mouse_y;
    uint8_t has_mouse_input;
    uint8_t mouse_x_active;
    uint8_t mouse_y_active;
    fp32 yaw_mouse_step;
    fp32 yaw_mouse_speed_scale;
    fp32 pitch_mouse_speed_scale;
    fp32 pitch_step;
    fp32 yaw_comp_step;
    fp32 gyro_speed_scale;
    fp32 chassis_target_speed_dps;
    fp32 chassis_comp_ff_source_dps;
    fp32 chassis_track_ff_source_dps;
    fp32 chassis_speed_dps;
    fp32 chassis_speed_track_dps;
    fp32 chassis_speed_rpm_raw;
    fp32 chassis_speed_rpm_for_comp;
    uint32_t chassis_status_tick;
    uint32_t chassis_status_age_ms;
    fp32 chassis_comp_raw_dps;
    fp32 chassis_comp_ff_dps;
    fp32 chassis_comp_pid_correction_dps;
    fp32 yaw_speed_feedback_dps;
    fp32 yaw_comp_feedback_dps;
    fp32 comp_cmd_alpha;
    fp32 yaw_mouse_cmd_alpha;
    fp32 comp_cmd_max_delta_dps;
    fp32 chassis_comp_cmd_last_dps;
    fp32 yaw_feedback_alpha;
    fp32 yaw_comp_track_alpha;
    fp32 yaw_speed_feedback_rpm;
    fp32 yaw_mouse_cmd_dps;
    fp32 yaw_comp_track_target_dps;
    fp32 yaw_comp_track_delta_dps;
    fp32 yaw_angle;
    fp32 pitch_angle;
    fp32 yaw_target_angle;
    fp32 yaw_angle_error;
    fp32 pitch_angle_error;
    fp32 yaw_angle_out;
    fp32 yaw_speed_out;
    fp32 pitch_angle_out;
    fp32 pitch_speed_out;
    fp32 pitch_hold_boost;
    fp32 idle_pos_error_deg;
    fp32 idle_pos_hold_cmd_raw_dps;
    fp32 idle_pos_hold_alpha;
    uint32_t idle_no_mouse_ms;
    uint8_t gyro_target_cmd_changed;
    uint8_t gyro_enable_state_changed;
    uint8_t pid_allow;

    if ((tuchuan_rc_data == NULL) || (yaw_motor == NULL) || (pitch_motor == NULL))
    {
        g_tuchuan_gyro_comp_speed_dps = 0.0f;
        return;
    }

    /*
     * 底盘补偿专用 PID 先保持很轻：
     * 主体补偿尽量靠底盘转速前馈和真实 yaw 速度反馈匹配，
     * 不靠大 PID 增益去硬拉。
     */
    g_tuchuan_chassis_comp_pid.Kp = 0.08f;
    g_tuchuan_chassis_comp_pid.Ki = 0.0008f;
    g_tuchuan_chassis_comp_pid.Kd = 0.0f;
    g_tuchuan_chassis_comp_pid.max_out = 6.0f;
    g_tuchuan_chassis_comp_pid.max_iout = 1.2f;

    keyboard_value = tuchuan_rc_data->keyboard_value;

    /*
     * F 键采用沿触发，避免按住时高频抖动导致反复开关。
     * 当前约定：F 只负责开关，不改方向；补偿方向固定由底盘rpm符号决定。
     */
    f_pressed = (uint8_t)(((keyboard_value & TUCHUAN_KEY_F) != 0u) ? 1u : 0u);
    if ((f_pressed != 0u) && (last_f_pressed == 0u))
    {
        g_tuchuan_gyro_enabled = (uint8_t)(g_tuchuan_gyro_enabled == 0u ? 1u : 0u);
        if (g_tuchuan_gyro_enabled != 0u)
        {
            /* 开启时清空滤波状态，避免上次关闭前的尾巴导致“先顿后跟”。 */
            yaw_comp_base_angle = get_yaw_angle(yaw_motor);
            yaw_mouse_offset_angle = 0.0f;
            pitch_target_angle = get_pitch_angle(pitch_motor);
            chassis_comp_cmd_lpf_dps = 0.0f;
            yaw_comp_track_dps = 0.0f;
            chassis_speed_rpm_lpf = 0.0f;
            yaw_mouse_cmd_lpf_dps = 0.0f;
            chassis_rpm_deadzone_active = 0u;
            idle_pos_hold_active = 0u;
            idle_pos_last_input_tick = HAL_GetTick();
            idle_pos_hold_ref_angle = yaw_comp_base_angle;
            idle_pos_hold_cmd_lpf_dps = 0.0f;
            last_yaw_comp_track_target_dps = 0.0f;
            last_gyro_target_cmd_dps = 0.0f;
            gyro_target_ff_hold_ms = TUCHUAN_GYRO_CHASSIS_TARGET_FF_HOLD_MS;
            PID_clear(&g_tuchuan_chassis_comp_pid);
        }
        else
        {
            yaw_comp_track_dps = 0.0f;
            last_yaw_comp_track_target_dps = 0.0f;
            last_gyro_target_cmd_dps = 0.0f;
            gyro_target_ff_hold_ms = 0u;
        }
    }
    last_f_pressed = f_pressed;

    g_pressed = (uint8_t)(((keyboard_value & TUCHUAN_KEY_G) != 0u) ? 1u : 0u);
    if ((g_pressed != 0u) && (last_g_pressed == 0u))
    {
        /* 关闭目标前馈后，补偿方向完全跟随底盘实测转速，这里不再单独改补偿方向。 */
    }
    last_g_pressed = g_pressed;

    /* 关闭目标前馈后，底盘补偿直接跟随实测转速，不再额外叠加本地挡位倍率。 */
    shift_pressed = (uint8_t)(((keyboard_value & TUCHUAN_KEY_SHIFT) != 0u) ? 1u : 0u);
    ctrl_pressed = (uint8_t)(((keyboard_value & TUCHUAN_KEY_CTRL) != 0u) ? 1u : 0u);
    (void)shift_pressed;
    (void)ctrl_pressed;
    gyro_speed_scale = 1.0f;

    now_tick = HAL_GetTick();
    if ((last_tick == 0u) || (now_tick <= last_tick))
    {
        dt_ms = 10u;
    }
    else
    {
        dt_ms = now_tick - last_tick;
        if (dt_ms > 20u)
        {
            dt_ms = 20u;
        }
    }
    last_tick = now_tick;

    /*
     * 一阶低通系数 alpha = dt / (tau + dt)
     * - comp_cmd_alpha：补偿目标角速度滤波系数
     * - yaw_feedback_alpha：Yaw 速度反馈滤波系数
     */
    comp_cmd_alpha = ((fp32)dt_ms) / (TUCHUAN_GYRO_CHASSIS_CMD_FILTER_TAU_MS + (fp32)dt_ms);
    if (comp_cmd_alpha < 0.0f)
    {
        comp_cmd_alpha = 0.0f;
    }
    else if (comp_cmd_alpha > 1.0f)
    {
        comp_cmd_alpha = 1.0f;
    }

    yaw_feedback_alpha = ((fp32)dt_ms) / (TUCHUAN_GYRO_FEEDBACK_FILTER_TAU_MS + (fp32)dt_ms);
    if (yaw_feedback_alpha < 0.0f)
    {
        yaw_feedback_alpha = 0.0f;
    }
    else if (yaw_feedback_alpha > 1.0f)
    {
        yaw_feedback_alpha = 1.0f;
    }

    yaw_mouse_cmd_alpha = ((fp32)dt_ms) / (TUCHUAN_GYRO_CHASSIS_MOUSE_CMD_FILTER_TAU_MS + (fp32)dt_ms);
    if (yaw_mouse_cmd_alpha < 0.0f)
    {
        yaw_mouse_cmd_alpha = 0.0f;
    }
    else if (yaw_mouse_cmd_alpha > 1.0f)
    {
        yaw_mouse_cmd_alpha = 1.0f;
    }

    yaw_angle = get_yaw_angle(yaw_motor);
    pitch_angle = get_pitch_angle(pitch_motor);

    if (target_inited == 0u)
    {
        yaw_comp_base_angle = yaw_angle;
        yaw_mouse_offset_angle = 0.0f;
        pitch_target_angle = pitch_angle;
        chassis_comp_cmd_lpf_dps = 0.0f;
        yaw_comp_track_dps = 0.0f;
        yaw_speed_feedback_lpf_rpm = (fp32)yaw_motor->speed_rpm;
        yaw_mouse_cmd_lpf_dps = 0.0f;

        /* 初始化无鼠标位置回正状态，避免上电后等待输入才能进入锁定。 */
        idle_pos_hold_active = 0u;
        idle_pos_last_input_tick = now_tick;
        idle_pos_hold_ref_angle = yaw_angle;
        idle_pos_hold_cmd_lpf_dps = 0.0f;
        last_yaw_comp_track_target_dps = 0.0f;
        last_gyro_target_cmd_dps = 0.0f;
        gyro_target_ff_hold_ms = 0u;

        target_inited = 1u;
    }

    if (g_tuchuan_mouse_resync_pending != 0u)
    {
        yaw_comp_base_angle = yaw_angle;
        yaw_mouse_offset_angle = 0.0f;
        pitch_target_angle = pitch_angle;
        yaw_comp_track_dps = 0.0f;
        yaw_speed_feedback_lpf_rpm = (fp32)yaw_motor->speed_rpm;
        yaw_mouse_cmd_lpf_dps = 0.0f;
        idle_pos_hold_active = 0u;
        idle_pos_last_input_tick = now_tick;
        idle_pos_hold_ref_angle = yaw_angle;
        idle_pos_hold_cmd_lpf_dps = 0.0f;
        last_yaw_comp_track_target_dps = 0.0f;
        last_gyro_target_cmd_dps = 0.0f;
        gyro_target_ff_hold_ms = 0u;
        mouse_control_started = 1u;
        target_inited = 1u;
        g_tuchuan_mouse_resync_pending = 0u;
    }

    mouse_x = tuchuan_rc_data->mouse_x;
    mouse_y = tuchuan_rc_data->mouse_y;

    if (abs(mouse_x) < TUCHUAN_MOUSE_DEADZONE_X)
    {
        mouse_x = 0;
    }
    if (abs(mouse_y) < TUCHUAN_MOUSE_DEADZONE_Y)
    {
        mouse_y = 0;
    }

    abs_mouse_x = abs(mouse_x);
    abs_mouse_y = abs(mouse_y);

    if ((abs_mouse_x >= TUCHUAN_MOUSE_X_DECOUPLE_THRESHOLD) &&
        (abs_mouse_y * TUCHUAN_MOUSE_XY_DECOUPLE_RATIO <= abs_mouse_x))
    {
        mouse_y = 0;
    }
    else if ((abs_mouse_y >= TUCHUAN_MOUSE_Y_DECOUPLE_THRESHOLD) &&
             (abs_mouse_x * TUCHUAN_MOUSE_XY_DECOUPLE_RATIO <= abs_mouse_y))
    {
        mouse_x = 0;
    }

    abs_mouse_x = abs(mouse_x);
    abs_mouse_y = abs(mouse_y);
    mouse_x_active = (uint8_t)((mouse_x != 0) ? 1u : 0u);
    mouse_y_active = (uint8_t)((mouse_y != 0) ? 1u : 0u);
    has_mouse_input = (uint8_t)((mouse_x_active != 0u) || (mouse_y_active != 0u));

    if (mouse_control_started == 0u)
    {
        yaw_comp_base_angle = yaw_angle;
        yaw_mouse_offset_angle = 0.0f;
        pitch_target_angle = pitch_angle;

        if ((has_mouse_input == 0u) && (g_tuchuan_gyro_enabled == 0u))
        {
            gimbal_out.yaw = 0;
            gimbal_out.pitch = 0;
            g_tuchuan_gyro_comp_speed_dps = 0.0f;
            chassis_comp_cmd_lpf_dps = 0.0f;
            yaw_comp_track_dps = 0.0f;
            yaw_speed_feedback_lpf_rpm = (fp32)yaw_motor->speed_rpm;
            yaw_mouse_cmd_lpf_dps = 0.0f;
            last_yaw_comp_track_target_dps = 0.0f;
            last_gyro_target_cmd_dps = 0.0f;
            gyro_target_ff_hold_ms = 0u;
            return;
        }

        mouse_control_started = 1u;
    }

    yaw_mouse_speed_scale = 1.0f + ((fp32)abs(mouse_x) * (10.0f / (fp32)dt_ms) * TUCHUAN_MOUSE_YAW_SPEED_BOOST);
    if (yaw_mouse_speed_scale > TUCHUAN_MOUSE_YAW_SPEED_SCALE_MAX)
    {
        yaw_mouse_speed_scale = TUCHUAN_MOUSE_YAW_SPEED_SCALE_MAX;
    }
    yaw_mouse_step = -(fp32)mouse_x * TUCHUAN_MOUSE_YAW_GAIN * yaw_mouse_speed_scale * ((fp32)dt_ms / 10.0f);

    // pitch_mouse_speed_scale = 1.0f + ((fp32)abs(mouse_y) * (10.0f / (fp32)dt_ms) * TUCHUAN_MOUSE_PITCH_SPEED_BOOST);
    // pitch_step = (fp32)mouse_y * TUCHUAN_MOUSE_PITCH_GAIN * pitch_mouse_speed_scale * ((fp32)dt_ms / 10.0f);
    pitch_step = shape_tuchuan_pitch_mouse_step(mouse_y, dt_ms);

    if (yaw_mouse_step > TUCHUAN_MOUSE_X_MAX_STEP_DEG)
    {
        yaw_mouse_step = TUCHUAN_MOUSE_X_MAX_STEP_DEG;
    }
    else if (yaw_mouse_step < -TUCHUAN_MOUSE_X_MAX_STEP_DEG)
    {
        yaw_mouse_step = -TUCHUAN_MOUSE_X_MAX_STEP_DEG;
    }

    if (pitch_step > TUCHUAN_MOUSE_Y_MAX_STEP_DEG)
    {
        pitch_step = TUCHUAN_MOUSE_Y_MAX_STEP_DEG;
    }
    else if (pitch_step < -TUCHUAN_MOUSE_Y_MAX_STEP_DEG)
    {
        pitch_step = -TUCHUAN_MOUSE_Y_MAX_STEP_DEG;
    }

    /*
     * 无鼠标位置回正逻辑：
     * - 鼠标有输入：立即退出回正并刷新“最后输入时间”，保证跟手优先；
     * - 鼠标无输入且补偿开启一段时间后：锁定当前yaw角为参考角，若偏离则输出回正角速度。
     */
    yaw_mouse_cmd_dps = yaw_mouse_step * (1000.0f / (fp32)dt_ms);
    yaw_mouse_cmd_lpf_dps += yaw_mouse_cmd_alpha * (yaw_mouse_cmd_dps - yaw_mouse_cmd_lpf_dps);
    if ((mouse_x_active == 0u) && (fabs(yaw_mouse_cmd_lpf_dps) < 0.3f))
    {
        yaw_mouse_cmd_lpf_dps = 0.0f;
    }

    yaw_speed_feedback_rpm = (fp32)yaw_motor->speed_rpm;
    yaw_speed_feedback_lpf_rpm += yaw_feedback_alpha * (yaw_speed_feedback_rpm - yaw_speed_feedback_lpf_rpm);

    idle_pos_hold_cmd_raw_dps = 0.0f;
    idle_pos_hold_alpha = ((fp32)dt_ms) / (TUCHUAN_GYRO_IDLE_POS_CMD_FILTER_TAU_MS + (fp32)dt_ms);
    if (idle_pos_hold_alpha < 0.0f)
    {
        idle_pos_hold_alpha = 0.0f;
    }
    else if (idle_pos_hold_alpha > 1.0f)
    {
        idle_pos_hold_alpha = 1.0f;
    }

#if (TUCHUAN_GYRO_IDLE_POS_HOLD_ENABLE != 0u)
    if (has_mouse_input != 0u)
    {
        idle_pos_last_input_tick = now_tick;
        idle_pos_hold_active = 0u;
        idle_pos_hold_ref_angle = yaw_angle;
        idle_pos_hold_cmd_lpf_dps = 0.0f;
    }
    else if (g_tuchuan_gyro_enabled != 0u)
    {
        if ((idle_pos_last_input_tick == 0u) || (now_tick < idle_pos_last_input_tick))
        {
            idle_no_mouse_ms = 0u;
        }
        else
        {
            idle_no_mouse_ms = now_tick - idle_pos_last_input_tick;
        }

        if ((idle_pos_hold_active == 0u) && (idle_no_mouse_ms >= TUCHUAN_GYRO_IDLE_POS_LOCK_DELAY_MS))
        {
            idle_pos_hold_active = 1u;
            idle_pos_hold_ref_angle = yaw_angle;
            idle_pos_hold_cmd_lpf_dps = 0.0f;
            PID_clear(&g_tuchuan_chassis_comp_pid);
        }

        if (idle_pos_hold_active != 0u)
        {
            idle_pos_error_deg = idle_pos_hold_ref_angle - yaw_angle;
            while (idle_pos_error_deg > ANGLE_180)
            {
                idle_pos_error_deg -= ANGLE_360;
            }
            while (idle_pos_error_deg < -ANGLE_180)
            {
                idle_pos_error_deg += ANGLE_360;
            }

            if (fabs(idle_pos_error_deg) < TUCHUAN_GYRO_IDLE_POS_ERR_DEADBAND_DEG)
            {
                idle_pos_error_deg = 0.0f;
            }

            idle_pos_hold_cmd_raw_dps = idle_pos_error_deg * TUCHUAN_GYRO_IDLE_POS_KP_DPS_PER_DEG;
            if (idle_pos_hold_cmd_raw_dps > TUCHUAN_GYRO_IDLE_POS_MAX_DPS)
            {
                idle_pos_hold_cmd_raw_dps = TUCHUAN_GYRO_IDLE_POS_MAX_DPS;
            }
            else if (idle_pos_hold_cmd_raw_dps < -TUCHUAN_GYRO_IDLE_POS_MAX_DPS)
            {
                idle_pos_hold_cmd_raw_dps = -TUCHUAN_GYRO_IDLE_POS_MAX_DPS;
            }
        }
    }
    else
    {
        idle_pos_hold_active = 0u;
        idle_pos_hold_ref_angle = yaw_angle;
        idle_pos_hold_cmd_lpf_dps = 0.0f;
    }
#else
    (void)idle_no_mouse_ms;
#endif

    idle_pos_hold_cmd_lpf_dps += idle_pos_hold_alpha * (idle_pos_hold_cmd_raw_dps - idle_pos_hold_cmd_lpf_dps);

    /*
     * 关键补偿逻辑：
     * 1) 优先读取 USART1 底盘状态帧中的 chassis_gyro_speed_rpm。
     * 2) 若状态帧超时，则补偿输入按0处理。
     * 3) 全流程不再回退旧接口，避免新旧链路混用造成符号与量纲混乱。
     */
    chassis_status_tick = get_usart1_chassis_status_update_tick();
    if ((chassis_status_tick != 0u) && (now_tick >= chassis_status_tick))
    {
        chassis_status_age_ms = now_tick - chassis_status_tick;
    }
    else
    {
        chassis_status_age_ms = 0xFFFFFFFFu;
    }

    /*
     * 新链路策略：仅使用USART1上报的底盘实时rpm。
     * - 状态帧新鲜：读取实时rpm。
     * - 状态帧超时：输入按0处理。
     */
    if (chassis_status_age_ms <= TUCHUAN_GYRO_CHASSIS_STATUS_TIMEOUT_MS)
    {
        chassis_speed_rpm_raw = (fp32)get_usart1_chassis_gyro_speed_rpm();
    }
    else
    {
        chassis_speed_rpm_raw = 0.0f;
    }

    /* rpm一阶低通：降低测速噪声导致的“卡一下卡一下”。 */
    /* 先去掉底盘转速低通，直接用实时转速进入补偿判定，优先提升响应。 */
    chassis_speed_rpm_lpf = chassis_speed_rpm_raw;

    /*
     * 迟滞死区：
     * - 进入阈值高一点，避免噪声触发；
     * - 退出阈值低一点，避免边界来回抖动。
     */
    /*
     * 低速稳定性改造：
     * 1) 无鼠标且补偿开启时，走“连续补偿”路径，仅保留极小阈值过滤静止噪声；
     *    避免硬死区导致的补偿断续（表现为稳一下、慢一下、累计慢圈）。
     * 2) 有鼠标输入或补偿关闭时，仍使用迟滞死区，优先抑制噪声触发。
     */
    if ((has_mouse_input == 0u) && (g_tuchuan_gyro_enabled != 0u))
    {
        if (fabs(chassis_speed_rpm_lpf) >= TUCHUAN_GYRO_CHASSIS_IDLE_MIN_COMP_RPM)
        {
            chassis_speed_rpm_for_comp = chassis_speed_rpm_lpf;
        }
        else
        {
            chassis_speed_rpm_for_comp = 0.0f;
        }

        chassis_rpm_deadzone_active = 0u;
        chassis_speed_rpm_for_comp *= TUCHUAN_GYRO_CHASSIS_NO_MOUSE_TRACK_GAIN;
    }
    else
    {
        if (chassis_rpm_deadzone_active == 0u)
        {
            if (fabs(chassis_speed_rpm_lpf) >= TUCHUAN_GYRO_CHASSIS_RPM_DEADZONE_ENTER)
            {
                chassis_rpm_deadzone_active = 1u;
            }
        }
        else
        {
            if (fabs(chassis_speed_rpm_lpf) <= TUCHUAN_GYRO_CHASSIS_RPM_DEADZONE_EXIT)
            {
                chassis_rpm_deadzone_active = 0u;
            }
        }

        if (chassis_rpm_deadzone_active != 0u)
        {
            chassis_speed_rpm_for_comp = chassis_speed_rpm_lpf;
        }
        else
        {
            chassis_speed_rpm_for_comp = 0.0f;
        }
    }
    chassis_speed_dps = chassis_speed_rpm_for_comp *
                        TUCHUAN_GYRO_CHASSIS_RPM_TO_DPS *
                        TUCHUAN_GYRO_CHASSIS_RPM_COMP_GAIN;

    if (chassis_speed_dps > TUCHUAN_GYRO_CHASSIS_MAX_COMP_DPS)
    {
        chassis_speed_dps = TUCHUAN_GYRO_CHASSIS_MAX_COMP_DPS;
    }
    else if (chassis_speed_dps < -TUCHUAN_GYRO_CHASSIS_MAX_COMP_DPS)
    {
        chassis_speed_dps = -TUCHUAN_GYRO_CHASSIS_MAX_COMP_DPS;
    }

    chassis_speed_track_dps = 0.0f;
    if ((g_tuchuan_gyro_enabled != 0u) && (fabs(chassis_speed_rpm_raw) >= TUCHUAN_GYRO_CHASSIS_TRACK_MIN_RPM))
    {
        chassis_speed_track_dps = chassis_speed_rpm_raw *
                                  TUCHUAN_GYRO_CHASSIS_RPM_TO_DPS *
                                  TUCHUAN_GYRO_CHASSIS_RPM_COMP_GAIN;

        if (chassis_speed_track_dps > TUCHUAN_GYRO_CHASSIS_MAX_COMP_DPS)
        {
            chassis_speed_track_dps = TUCHUAN_GYRO_CHASSIS_MAX_COMP_DPS;
        }
        else if (chassis_speed_track_dps < -TUCHUAN_GYRO_CHASSIS_MAX_COMP_DPS)
        {
            chassis_speed_track_dps = -TUCHUAN_GYRO_CHASSIS_MAX_COMP_DPS;
        }
    }

    if (g_tuchuan_gyro_enabled != 0u)
    {
        chassis_target_speed_dps = (fp32)g_tuchuan_gyro_direction *
                                   TUCHUAN_GYRO_CHASSIS_TARGET_BASE_DPS *
                                   gyro_speed_scale;

        if (chassis_target_speed_dps > TUCHUAN_GYRO_CHASSIS_MAX_COMP_DPS)
        {
            chassis_target_speed_dps = TUCHUAN_GYRO_CHASSIS_MAX_COMP_DPS;
        }
        else if (chassis_target_speed_dps < -TUCHUAN_GYRO_CHASSIS_MAX_COMP_DPS)
        {
            chassis_target_speed_dps = -TUCHUAN_GYRO_CHASSIS_MAX_COMP_DPS;
        }
    }
    else
    {
        chassis_target_speed_dps = 0.0f;
    }

    gyro_target_cmd_changed = (uint8_t)((fabs(chassis_target_speed_dps - last_gyro_target_cmd_dps) >=
                                         TUCHUAN_GYRO_CHASSIS_TARGET_CHANGE_DPS) ? 1u : 0u);
    if (gyro_target_cmd_changed != 0u)
    {
        gyro_target_ff_hold_ms = TUCHUAN_GYRO_CHASSIS_TARGET_FF_HOLD_MS;
    }
    else if (gyro_target_ff_hold_ms != 0u)
    {
        if (gyro_target_ff_hold_ms > dt_ms)
        {
            gyro_target_ff_hold_ms = (uint16_t)(gyro_target_ff_hold_ms - (uint16_t)dt_ms);
        }
        else
        {
            gyro_target_ff_hold_ms = 0u;
        }
    }

    chassis_comp_ff_source_dps = chassis_speed_dps;
    chassis_track_ff_source_dps = chassis_speed_track_dps;
    if ((g_tuchuan_gyro_enabled != 0u) && (gyro_target_ff_hold_ms != 0u))
    {
        /*
         * 开启、切档、反向后的短窗口内：
         * 1) 主补偿前馈用更高占比靠近目标挡位；
         * 2) 基准角跟踪链直接按目标挡位走，优先把“转出去”压住；
         * 3) 一旦实测转速已经追上，再切回实测值，避免长期估算补偿。
         */
        chassis_comp_ff_source_dps += TUCHUAN_GYRO_CHASSIS_TARGET_FF_BLEND *
                                      (chassis_target_speed_dps - chassis_comp_ff_source_dps);
        chassis_track_ff_source_dps = chassis_target_speed_dps;

        if (((chassis_target_speed_dps * chassis_speed_dps) > 0.0f) &&
            (fabs(chassis_speed_dps) >= fabs(chassis_comp_ff_source_dps)))
        {
            chassis_comp_ff_source_dps = chassis_speed_dps;
        }

        if (((chassis_target_speed_dps * chassis_speed_track_dps) > 0.0f) &&
            (fabs(chassis_speed_track_dps) >= fabs(chassis_track_ff_source_dps)))
        {
            chassis_track_ff_source_dps = chassis_speed_track_dps;
        }
    }
    last_gyro_target_cmd_dps = chassis_target_speed_dps;

    /* 关闭目标前馈后，恢复为只根据底盘实测转速做补偿。 */
    chassis_comp_ff_dps = TUCHUAN_GYRO_CHASSIS_COMP_SIGN * chassis_speed_dps * gyro_speed_scale;
    chassis_comp_pid_correction_dps = 0.0f;
    yaw_speed_feedback_dps = yaw_speed_feedback_lpf_rpm * TUCHUAN_GYRO_YAW_FEEDBACK_RPM_TO_DPS;
    yaw_comp_feedback_dps = yaw_speed_feedback_dps;
    pid_allow = 0u;

    if (g_tuchuan_gyro_enabled != 0u)
    {
        /* 先关闭外层速度PID，当前以“前馈+位置回正”为主，优先抑制自激振荡。 */

#if (TUCHUAN_GYRO_CHASSIS_PID_ENABLE_MOUSE_IDLE_ONLY != 0u)
        /*
         * 为了保证鼠标跟手，PID修正默认只在“鼠标无输入”时介入。
         * 鼠标有输入时保留前馈为主，避免与人工瞄准对抗。
         */
        pid_allow = (uint8_t)((has_mouse_input == 0u) ? 1u : 0u);
#endif

#if (TUCHUAN_GYRO_CHASSIS_PID_ENABLE_MOUSE_IDLE_ONLY == 0u)
        if (pid_allow == 0u)
        {
            yaw_comp_feedback_dps = yaw_speed_feedback_dps -
                                    (yaw_mouse_cmd_lpf_dps * TUCHUAN_GYRO_CHASSIS_MOUSE_DECOUPLE_GAIN);
            pid_allow = 1u;
        }
#endif

        if (pid_allow != 0u)
        {
            /*
             * 反馈角速度：使用yaw电机速度反馈（rpm->deg/s）作为PID的ref。
             * 目标角速度：使用前馈补偿目标（chassis_comp_ff_dps）作为PID的set。
             */
            chassis_comp_pid_correction_dps = PID_calc(&g_tuchuan_chassis_comp_pid,
                                                       yaw_comp_feedback_dps,
                                                       chassis_comp_ff_dps);

            if (chassis_comp_pid_correction_dps > TUCHUAN_GYRO_CHASSIS_PID_MAX_ABS_OUTPUT_DPS)
            {
                chassis_comp_pid_correction_dps = TUCHUAN_GYRO_CHASSIS_PID_MAX_ABS_OUTPUT_DPS;
            }
            else if (chassis_comp_pid_correction_dps < -TUCHUAN_GYRO_CHASSIS_PID_MAX_ABS_OUTPUT_DPS)
            {
                chassis_comp_pid_correction_dps = -TUCHUAN_GYRO_CHASSIS_PID_MAX_ABS_OUTPUT_DPS;
            }
        }
        else
        {
            /* 鼠标输入阶段不让PID积累误差，避免松手后瞬时冲击。 */
            PID_clear(&g_tuchuan_chassis_comp_pid);
        }

        /* 位置基准角里先不叠加额外 PID 修正，只保留前馈主链路。 */
        chassis_comp_pid_correction_dps = 0.0f;
        PID_clear(&g_tuchuan_chassis_comp_pid);
        chassis_comp_raw_dps = chassis_comp_ff_dps;
    }
    else
    {
        chassis_comp_raw_dps = 0.0f;
        PID_clear(&g_tuchuan_chassis_comp_pid);
    }

    yaw_comp_track_target_dps = TUCHUAN_GYRO_CHASSIS_COMP_SIGN * chassis_speed_track_dps * gyro_speed_scale;
    yaw_comp_track_delta_dps = yaw_comp_track_target_dps - last_yaw_comp_track_target_dps;
    yaw_comp_track_alpha = ((fp32)dt_ms) / (TUCHUAN_GYRO_CHASSIS_TRACK_TAU_MS + (fp32)dt_ms);
    if (yaw_comp_track_alpha < 0.0f)
    {
        yaw_comp_track_alpha = 0.0f;
    }
    else if (yaw_comp_track_alpha > 1.0f)
    {
        yaw_comp_track_alpha = 1.0f;
    }

    gyro_enable_state_changed = (uint8_t)((g_tuchuan_gyro_enabled != last_gyro_enabled_state) ? 1u : 0u);
    if (g_tuchuan_gyro_enabled != 0u)
    {
        /*
         * 云台补偿基准角走快速链路：
         * 尽量贴近底盘实时转速，这样开小陀螺、切速度、反向时
         * 不会留下“先转出去一点再稳住”的残余角度。
         */
        if ((gyro_enable_state_changed != 0u) ||
            ((yaw_comp_track_target_dps * last_yaw_comp_track_target_dps) < 0.0f) ||
            (fabs(yaw_comp_track_delta_dps) >= TUCHUAN_GYRO_CHASSIS_TRACK_SYNC_TRIGGER_DPS))
        {
            yaw_comp_track_dps = yaw_comp_track_target_dps;
        }
        else
        {
            yaw_comp_track_dps +=
                yaw_comp_track_alpha * (yaw_comp_track_target_dps - yaw_comp_track_dps);
        }
    }
    else
    {
        yaw_comp_track_dps = 0.0f;
    }
    last_yaw_comp_track_target_dps = yaw_comp_track_target_dps;
    last_gyro_enabled_state = g_tuchuan_gyro_enabled;

    /* 对补偿指令做一阶低通，降低中低速下的抖动与齿感。 */
    chassis_comp_cmd_last_dps = chassis_comp_cmd_lpf_dps;
    chassis_comp_cmd_lpf_dps += comp_cmd_alpha * (chassis_comp_raw_dps - chassis_comp_cmd_lpf_dps);
    comp_cmd_max_delta_dps = TUCHUAN_GYRO_CHASSIS_CMD_MAX_DELTA_DPS_PER_S * ((fp32)dt_ms / 1000.0f);
    if (comp_cmd_max_delta_dps < 0.5f)
    {
        comp_cmd_max_delta_dps = 0.5f;
    }
    chassis_comp_cmd_lpf_dps = vision_clamp_delta(chassis_comp_cmd_lpf_dps,
                                                  chassis_comp_cmd_last_dps,
                                                  comp_cmd_max_delta_dps);

    /* 关闭补偿后给一个小阈值吸附到零，避免尾巴拖动。 */
    if ((g_tuchuan_gyro_enabled == 0u) && (fabs(chassis_comp_cmd_lpf_dps) < 0.3f))
    {
        chassis_comp_cmd_lpf_dps = 0.0f;
    }

    yaw_comp_step = yaw_comp_track_dps * ((fp32)dt_ms / 1000.0f);
    yaw_comp_base_angle += yaw_comp_step;
    yaw_comp_base_angle = wrap_angle_deg_360(yaw_comp_base_angle);

    /*
     * 不把单个零输入周期当成真的“松手”，
     * 这样保留累计目标角后，鼠标手感会更连续。
     */
    yaw_mouse_offset_angle += yaw_mouse_step;
    yaw_mouse_offset_angle = wrap_angle_deg_180(yaw_mouse_offset_angle);

    yaw_target_angle = yaw_comp_base_angle + yaw_mouse_offset_angle;
    yaw_target_angle = wrap_angle_deg_360(yaw_target_angle);

    pitch_target_angle += pitch_step;
    pitch_target_angle = pitch_target_guard_inside(pitch_motor, pitch_target_angle);

    yaw_angle_error = yaw_target_angle - yaw_angle;
    if (yaw_angle_error > ANGLE_180)
    {
        yaw_angle_error -= ANGLE_360;
    }
    else if (yaw_angle_error < -ANGLE_180)
    {
        yaw_angle_error += ANGLE_360;
    }

    pitch_angle_error = pitch_target_angle - pitch_angle;
    if (pitch_angle_error > ANGLE_180)
    {
        pitch_angle_error -= ANGLE_360;
    }
    else if (pitch_angle_error < -ANGLE_180)
    {
        pitch_angle_error += ANGLE_360;
    }

    /*
     * 鼠标停住后，把累计的鼠标偏置重新并回当前补偿基准角，
     * 保留底盘补偿链的手感，只清掉静止时残留的积分偏移。
     */
    if ((has_mouse_input == 0u) &&
        (fabs(yaw_angle_error) < TUCHUAN_MOUSE_IDLE_REBASE_YAW_ERR_DEG) &&
        (fabs((fp32)yaw_motor->speed_rpm) < TUCHUAN_MOUSE_IDLE_REBASE_SPEED_RPM))
    {
        yaw_comp_base_angle = yaw_target_angle;
        yaw_comp_base_angle = wrap_angle_deg_360(yaw_comp_base_angle);
        yaw_mouse_offset_angle = 0.0f;
        yaw_angle_error = 0.0f;
    }
    // yaw_position_hold_control(yaw_motor,&yaw_angle_error);
    yaw_angle_out = PID_calc(&yaw_angle_pid, 0.0f, yaw_angle_error);
    yaw_speed_out = PID_calc(&yaw_speed_pid, yaw_speed_feedback_lpf_rpm, yaw_angle_out);

    // position_hold_control(pitch_motor,&pitch_angle_error);
    pitch_angle_out = PID_calc(&pitch_angle_pid, 0.0f, pitch_angle_error);
    pitch_hold_boost = 1.0f;
    if ((abs_mouse_x >= TUCHUAN_MOUSE_X_DECOUPLE_THRESHOLD) && (mouse_y == 0))
    {
        pitch_hold_boost = 1.35f;
    }

    pitch_angle_out *= pitch_hold_boost;
    if (pitch_angle_out > pitch_angle_pid.max_out)
    {
        pitch_angle_out = pitch_angle_pid.max_out;
    }
    else if (pitch_angle_out < -pitch_angle_pid.max_out)
    {
        pitch_angle_out = -pitch_angle_pid.max_out;
    }

    pitch_speed_out = PID_calc(&pitch_speed_pid, pitch_motor->speed_rpm, pitch_angle_out);
    pitch_speed_out += vision_pitch_gravity_ff_current(pitch_motor);

    gimbal_out.yaw = (int16_t)yaw_speed_out;
    gimbal_out.pitch = pitch_limit_protection((int16_t)pitch_speed_out, pitch_motor, pitch_angle_error);

    if (g_tuchuan_gyro_enabled != 0u)
    {
        g_tuchuan_gyro_comp_speed_dps = yaw_comp_track_dps;
    }
    else
    {
        g_tuchuan_gyro_comp_speed_dps = 0.0f;
    }
}






/**
 * @brief       Pitch目标角度限制保护
 * @param[in]   target_angle: 目标角度（度）
 * @param[in]   pitch_motor: pitch电机测量值指针
 * @retval      限制后的目标角度
 */
static fp32 pitch_limit_protection_target(fp32 target_angle, const motor_measure_t *pitch_motor)
{
    /* 获取当前pitch角度 */
    fp32 current_angle = get_pitch_angle(pitch_motor);
    fp32 angle_error = target_angle - current_angle;
    
    /* 限制单次最大变化量，避免跳动过大 */
    const fp32 MAX_ANGLE_CHANGE_PER_FRAME = 10.0f;  // 每帧最大变化10度
    
    if (angle_error > MAX_ANGLE_CHANGE_PER_FRAME)
    {
        target_angle = current_angle + MAX_ANGLE_CHANGE_PER_FRAME;
    }
    else if (angle_error < -MAX_ANGLE_CHANGE_PER_FRAME)
    {
        target_angle = current_angle - MAX_ANGLE_CHANGE_PER_FRAME;
    }
    
    /* 限制pitch机械角度范围 */
    const fp32 PITCH_MIN_ANGLE = -30.0f;   // pitch最小角度（根据实际机械限制调整）
    const fp32 PITCH_MAX_ANGLE = 30.0f;    // pitch最大角度（根据实际机械限制调整）
    
    if (target_angle > PITCH_MAX_ANGLE)
    {
        target_angle = PITCH_MAX_ANGLE;
    }
    else if (target_angle < PITCH_MIN_ANGLE)
    {
        target_angle = PITCH_MIN_ANGLE;
    }
    
    return target_angle;
}

/**
 * @brief       视觉控制云台函数（yaw鼠标控制，pitch视觉控制）
 * @note        1) yaw轴完全由鼠标控制，复用鼠标云台控制逻辑
 *              2) pitch轴由视觉误差控制，带平滑滤波和角度限制
 *              3) 目标无效时pitch自动回中
 * @param[in]   x_offset: 视觉x偏差（yaw方向，但本函数不使用）
 * @param[in]   y_offset: 视觉y偏差（pitch方向）
 * @param[in]   target_valid: 视觉目标是否有效
 * @param[in]   control_far_active: 远距离控制激活标志
 * @param[in]   fallback_active: 降级模式激活标志
 * @retval      none：第一版视觉控制云台，测评的时候用过，后来没用了，但是比后来接了鼠标的那版视觉控制的好用因为后面东西太多太乱了
 */
void Vision_Control_Gimbal(int16_t x_offset,
    int16_t y_offset,
    uint8_t target_valid,
    uint8_t control_far_active,
    uint8_t fallback_active)
{
    const motor_measure_t *yaw_motor = get_yaw_gimbal_motor_measure_point();
    const motor_measure_t *pitch_motor = get_pitch_gimbal_motor_measure_point();
    const usart6_tuchuan_remote_control_t *tuchuan_rc_data = get_usart6_tuchuan_remote_control_point();

    /* 鼠标控制相关静态变量（复用鼠标云台逻辑） */
    static uint8_t mouse_control_inited = 0u;
    static fp32 yaw_target_angle = 0.0f;
    static fp32 yaw_comp_base_angle = 0.0f;
    static fp32 yaw_mouse_offset_angle = 0.0f;
    static uint32_t last_mouse_tick = 0u;
    static fp32 yaw_mouse_cmd_lpf_dps = 0.0f;
    static fp32 yaw_speed_feedback_lpf_rpm = 0.0f;

    /* pitch视觉控制相关静态变量 */
    static fp32 pitch_target_angle = 0.0f;
    static fp32 pitch_error_lpf = 0.0f;
    static uint32_t last_vision_tick = 0u;

    pid_type_def *pitch_angle_pid_use;
    fp32 x_error;
    fp32 y_error;
    fp32 yaw_angle;
    fp32 pitch_angle;
    fp32 yaw_angle_error;
    fp32 pitch_angle_error;
    fp32 yaw_angle_out;
    fp32 yaw_speed_out;
    fp32 pitch_angle_out;
    fp32 pitch_speed_out;
    fp32 yaw_mouse_step;
    fp32 yaw_mouse_speed_scale;
    fp32 yaw_mouse_cmd_dps;
    fp32 yaw_mouse_cmd_alpha;
    fp32 yaw_feedback_alpha;
    fp32 yaw_speed_feedback_rpm;
    uint32_t now_tick;
    uint32_t dt_ms;
    int16_t mouse_x;
    int16_t abs_mouse_x;
    uint8_t has_mouse_input;
    uint8_t far_active;
    uint8_t mouse_active_for_reset;

    if ((yaw_motor == NULL) || (pitch_motor == NULL) || (tuchuan_rc_data == NULL))
    {
        gimbal_out.yaw = 0;
        gimbal_out.pitch = 0;
        return;
    }

    now_tick = HAL_GetTick();

    /* 计算时间差 */
    if ((last_mouse_tick == 0u) || (now_tick <= last_mouse_tick))
    {
        dt_ms = 10u;
    }
    else
    {
        dt_ms = now_tick - last_mouse_tick;
        if (dt_ms > 20u)
        {
            dt_ms = 20u;
        }
    }

    /* 获取当前角度 */
    yaw_angle = get_yaw_angle(yaw_motor);
    pitch_angle = get_pitch_angle(pitch_motor);

    /* ==================== 鼠标控制 Yaw 轴 ==================== */
    /* 初始化鼠标控制状态 */
    if (mouse_control_inited == 0u)
    {
        yaw_comp_base_angle = yaw_angle;
        yaw_mouse_offset_angle = 0.0f;
        yaw_target_angle = yaw_angle;
        yaw_mouse_cmd_lpf_dps = 0.0f;
        yaw_speed_feedback_lpf_rpm = (fp32)yaw_motor->speed_rpm;
        mouse_control_inited = 1u;
        last_mouse_tick = now_tick;
    }

    /* 处理鼠标输入 */
    mouse_x = tuchuan_rc_data->mouse_x;

    /* 鼠标死区 */
    if (abs(mouse_x) < TUCHUAN_MOUSE_DEADZONE_X)
    {
        mouse_x = 0;
    }

    abs_mouse_x = abs(mouse_x);
    has_mouse_input = (uint8_t)((mouse_x != 0) ? 1u : 0u);

    /* 计算鼠标步进角度 */
    yaw_mouse_speed_scale = 1.0f + ((fp32)abs_mouse_x * (10.0f / (fp32)dt_ms) * TUCHUAN_MOUSE_YAW_SPEED_BOOST);
    if (yaw_mouse_speed_scale > TUCHUAN_MOUSE_YAW_SPEED_SCALE_MAX)
    {
        yaw_mouse_speed_scale = TUCHUAN_MOUSE_YAW_SPEED_SCALE_MAX;
    }

    yaw_mouse_step = -(fp32)mouse_x * TUCHUAN_MOUSE_YAW_GAIN * yaw_mouse_speed_scale * ((fp32)dt_ms / 10.0f);

    /* 限制单次最大步进 */
    if (yaw_mouse_step > TUCHUAN_MOUSE_X_MAX_STEP_DEG)
    {
        yaw_mouse_step = TUCHUAN_MOUSE_X_MAX_STEP_DEG;
    }
    else if (yaw_mouse_step < -TUCHUAN_MOUSE_X_MAX_STEP_DEG)
    {
        yaw_mouse_step = -TUCHUAN_MOUSE_X_MAX_STEP_DEG;
    }

    /* 鼠标指令滤波 */
    yaw_mouse_cmd_dps = yaw_mouse_step * (1000.0f / (fp32)dt_ms);
    yaw_mouse_cmd_alpha = ((fp32)dt_ms) / (TUCHUAN_GYRO_CHASSIS_MOUSE_CMD_FILTER_TAU_MS + (fp32)dt_ms);
    if (yaw_mouse_cmd_alpha < 0.0f) yaw_mouse_cmd_alpha = 0.0f;
    else if (yaw_mouse_cmd_alpha > 1.0f) yaw_mouse_cmd_alpha = 1.0f;

    yaw_mouse_cmd_lpf_dps += yaw_mouse_cmd_alpha * (yaw_mouse_cmd_dps - yaw_mouse_cmd_lpf_dps);

    if ((has_mouse_input == 0u) && (fabs(yaw_mouse_cmd_lpf_dps) < 0.3f))
    {
        yaw_mouse_cmd_lpf_dps = 0.0f;
    }

    /* 更新鼠标累积偏移 */
    yaw_mouse_offset_angle += yaw_mouse_step;
    yaw_mouse_offset_angle = wrap_angle_deg_180(yaw_mouse_offset_angle);

    /* Yaw基准角（目前无底盘补偿，可后续扩展） */
    yaw_target_angle = yaw_comp_base_angle + yaw_mouse_offset_angle;
    yaw_target_angle = wrap_angle_deg_360(yaw_target_angle);

    /* 计算yaw角度误差 */
    yaw_angle_error = yaw_target_angle - yaw_angle;
    if (yaw_angle_error > ANGLE_180)
    {
        yaw_angle_error -= ANGLE_360;
    }
    else if (yaw_angle_error < -ANGLE_180)
    {
        yaw_angle_error += ANGLE_360;
    }

    /* 鼠标静止且误差很小时，归零偏移量 */
    if ((has_mouse_input == 0u) &&
    (fabs(yaw_angle_error) < TUCHUAN_MOUSE_IDLE_REBASE_YAW_ERR_DEG) &&
    (fabs((fp32)yaw_motor->speed_rpm) < TUCHUAN_MOUSE_IDLE_REBASE_SPEED_RPM))
    {
        yaw_comp_base_angle = yaw_target_angle;
        yaw_comp_base_angle = wrap_angle_deg_360(yaw_comp_base_angle);
        yaw_mouse_offset_angle = 0.0f;
        yaw_angle_error = 0.0f;
    }

    /* Yaw轴PID计算 */
    yaw_angle_out = PID_calc(&yaw_angle_pid, 0.0f, yaw_angle_error);

    /* Yaw速度反馈滤波 */
    yaw_speed_feedback_rpm = (fp32)yaw_motor->speed_rpm;
    yaw_feedback_alpha = ((fp32)dt_ms) / (TUCHUAN_GYRO_FEEDBACK_FILTER_TAU_MS + (fp32)dt_ms);
    if (yaw_feedback_alpha < 0.0f) yaw_feedback_alpha = 0.0f;
    else if (yaw_feedback_alpha > 1.0f) yaw_feedback_alpha = 1.0f;

    yaw_speed_feedback_lpf_rpm += yaw_feedback_alpha * (yaw_speed_feedback_rpm - yaw_speed_feedback_lpf_rpm);
    yaw_speed_out = PID_calc(&yaw_speed_pid, yaw_speed_feedback_lpf_rpm, yaw_angle_out);

    /* ==================== 视觉控制 Pitch 轴 ==================== */
    /* 处理视觉输入 */
    if (target_valid != 0u)
    {
        y_error = (fp32)y_offset;
        last_vision_tick = now_tick;
    }
    else
    {
        y_error = 0.0f;
    }

    /* 视觉误差限制 */
    if (y_error > VISION_MAX_ERROR)
    {
        y_error = VISION_MAX_ERROR;
    }
    else if (y_error < -VISION_MAX_ERROR)
    {
        y_error = -VISION_MAX_ERROR;
    }

    /* 远距离模式选择 */
    far_active = (uint8_t)((control_far_active != 0u) ? 1u : 0u);
    pitch_angle_pid_use = (far_active != 0u) ? &g_vision_pitch_angle_pid_far : &g_vision_pitch_angle_pid;

    /* 视觉误差低通滤波（复用原来的平滑参数） */
    pitch_error_lpf += VISION_SMOOTHING * (y_error - pitch_error_lpf);

    /* 死区处理 */
    if ((pitch_error_lpf < VISION_Y_DEADZONE) && (pitch_error_lpf > -VISION_Y_DEADZONE))
    {
        pitch_error_lpf = 0.0f;
    }

    /* 计算pitch角度误差（单位：度） */
    pitch_angle_error = -pitch_error_lpf * VISION_Y_TO_PITCH_ERR_GAIN;

    /* 限制最大误差角度 */
    if (pitch_angle_error > VISION_XY_PITCH_MAX_ERR_DEG)
    {
        pitch_angle_error = VISION_XY_PITCH_MAX_ERR_DEG;
    }
    else if (pitch_angle_error < -VISION_XY_PITCH_MAX_ERR_DEG)
    {
        pitch_angle_error = -VISION_XY_PITCH_MAX_ERR_DEG;
    }

    /* 检查视觉目标是否超时（120ms） */
    uint32_t vision_age_ms = 0u;
    if (last_vision_tick != 0u)
    {
        vision_age_ms = (now_tick >= last_vision_tick) ? (now_tick - last_vision_tick) : 0u;
    }

    /* 目标有效且未超时，更新pitch目标角 */
    if ((target_valid != 0u) && (vision_age_ms <= 120u))
    {
        /* 根据误差更新目标角度（增量式） */
        pitch_target_angle = pitch_angle + pitch_angle_error;
        pitch_target_angle = pitch_target_guard_inside(pitch_motor, pitch_target_angle);

    /* 降级模式处理 */
        if (fallback_active != 0u)
        {
            pitch_angle_error *= VISION_XY_FALLBACK_SCALE;
            pitch_angle_pid_use->Iout = 0.0f;
        }
    }
    else
    {
        /* 无有效目标时，pitch缓慢回中 */
        pitch_target_angle = pitch_limit_protection_target(0.0f, pitch_motor);
        pitch_angle_error = 0.0f;
    }

    /* 计算pitch角度误差（用于PID） */
    fp32 pitch_current_error = pitch_target_angle - pitch_angle;
    if (pitch_current_error > ANGLE_180)
    {
        pitch_current_error -= ANGLE_360;
    }
    else if (pitch_current_error < -ANGLE_180)
    {
        pitch_current_error += ANGLE_360;
    }

    /* Pitch角度环PID */
    pitch_angle_out = PID_calc(pitch_angle_pid_use, 0.0f, pitch_current_error);

    /* 速度阻尼 */
    pitch_angle_out -= ((fp32)pitch_motor->speed_rpm) * VISION_XY_PITCH_SPEED_DAMP;

    /* Pitch速度环PID */
    pitch_speed_out = PID_calc(&pitch_speed_pid, pitch_motor->speed_rpm, pitch_angle_out);

    /* 重力前馈补偿 */
    pitch_speed_out += vision_pitch_gravity_ff_current(pitch_motor);

    /* 输出限制和保护 */
    gimbal_out.yaw = (int16_t)yaw_speed_out;
    gimbal_out.pitch = pitch_limit_protection((int16_t)pitch_speed_out, pitch_motor, pitch_current_error);

    /* 更新上次处理时间 */
    last_mouse_tick = now_tick;

    /* 目标无效时清除PID积分 */
    if ((target_valid == 0u) || (vision_age_ms > 120u))
    {
        PID_clear(pitch_angle_pid_use);
    }
}


/**
* @brief       视觉控制云台函数（小陀螺模式，yaw鼠标控制，pitch视觉控制）
* @note        1) yaw轴由鼠标控制，同时叠加小陀螺前馈补偿
*              2) pitch轴由视觉误差控制
*              3) 小陀螺模式下保持yaw轴跟随底盘旋转
* @param[in]   x_offset: 视觉x偏差（yaw方向，本函数不使用）
* @param[in]   y_offset: 视觉y偏差（pitch方向）
* @param[in]   target_valid: 视觉目标是否有效
* @param[in]   control_far_active: 远距离控制激活标志
* @param[in]   fallback_active: 降级模式激活标志
* @retval      none： 这个鼠标控制写的感觉还是有问题，主要在鼠标数据处理上，建议多了解下FPS的鼠标控制逻辑和处理
*/
void Vision_Control_Gimbal_SmallGyro(int16_t x_offset,
              int16_t y_offset,
              uint8_t target_valid,
              uint8_t control_far_active,
              uint8_t fallback_active)
{
    const motor_measure_t *yaw_motor = get_yaw_gimbal_motor_measure_point();
    const motor_measure_t *pitch_motor = get_pitch_gimbal_motor_measure_point();
    const usart6_tuchuan_remote_control_t *tuchuan_rc_data = get_usart6_tuchuan_remote_control_point();

    /* 鼠标控制相关静态变量 */
    static uint8_t mouse_control_inited = 0u;
    static fp32 yaw_target_angle = 0.0f;
    static fp32 yaw_comp_base_angle = 0.0f;
    static fp32 yaw_mouse_offset_angle = 0.0f;
    static uint32_t last_mouse_tick = 0u;
    static fp32 yaw_mouse_cmd_lpf_dps = 0.0f;
    static fp32 yaw_speed_feedback_lpf_rpm = 0.0f;

    /* pitch视觉控制相关静态变量 */
    static fp32 pitch_target_angle = 0.0f;
    static fp32 pitch_error_lpf = 0.0f;
    static uint32_t last_vision_tick = 0u;

    /* 小陀螺前馈相关静态变量 */
    static fp32 last_yaw_small_gyro_ff_rpm = 0.0f;

    pid_type_def *pitch_angle_pid_use;
    fp32 y_error;
    fp32 yaw_angle;
    fp32 pitch_angle;
    fp32 yaw_angle_error;
    fp32 pitch_angle_error;
    fp32 yaw_angle_out;
    fp32 yaw_speed_out;
    fp32 pitch_angle_out;
    fp32 pitch_speed_out;
    fp32 yaw_mouse_step;
    fp32 yaw_mouse_speed_scale;
    fp32 yaw_mouse_cmd_dps;
    fp32 yaw_mouse_cmd_alpha;
    fp32 yaw_feedback_alpha;
    fp32 yaw_speed_feedback_rpm;
    fp32 yaw_small_gyro_ff_rpm;
    uint32_t now_tick;
    uint32_t dt_ms;
    int16_t mouse_x;
    int16_t abs_mouse_x;
    uint8_t has_mouse_input;
    uint8_t far_active;
    uint32_t vision_age_ms;

    if ((yaw_motor == NULL) || (pitch_motor == NULL) || (tuchuan_rc_data == NULL))
    {
        gimbal_out.yaw = 0;
        gimbal_out.pitch = 0;
        return;
    }

    now_tick = HAL_GetTick();

    /* 计算时间差 */
    if ((last_mouse_tick == 0u) || (now_tick <= last_mouse_tick))
    {
        dt_ms = 10u;
    }
    else
    {
         dt_ms = now_tick - last_mouse_tick;
        if (dt_ms > 20u)
        {
            dt_ms = 20u;
        }
    }

    /* 获取当前角度 */
    yaw_angle = get_yaw_angle(yaw_motor);
    pitch_angle = get_pitch_angle(pitch_motor);

    /* ==================== 鼠标控制 Yaw 轴（叠加小陀螺前馈） ==================== */
    /* 初始化鼠标控制状态 */
    if (mouse_control_inited == 0u)
    {
    yaw_comp_base_angle = yaw_angle;
    yaw_mouse_offset_angle = 0.0f;
    yaw_target_angle = yaw_angle;
    yaw_mouse_cmd_lpf_dps = 0.0f;
    yaw_speed_feedback_lpf_rpm = (fp32)yaw_motor->speed_rpm;
    mouse_control_inited = 1u;
    last_mouse_tick = now_tick;
    }

    /* 处理鼠标输入 */
    mouse_x = tuchuan_rc_data->mouse_x;

    if (abs(mouse_x) < TUCHUAN_MOUSE_DEADZONE_X)
    {
        mouse_x = 0;
    }

    abs_mouse_x = abs(mouse_x);
    has_mouse_input = (uint8_t)((mouse_x != 0) ? 1u : 0u);

    /* 计算鼠标步进角度 */
    yaw_mouse_speed_scale = 1.0f + ((fp32)abs_mouse_x * (10.0f / (fp32)dt_ms) * TUCHUAN_MOUSE_YAW_SPEED_BOOST);
    if (yaw_mouse_speed_scale > TUCHUAN_MOUSE_YAW_SPEED_SCALE_MAX)
    {
        yaw_mouse_speed_scale = TUCHUAN_MOUSE_YAW_SPEED_SCALE_MAX;
    }

        yaw_mouse_step = -(fp32)mouse_x * TUCHUAN_MOUSE_YAW_GAIN * yaw_mouse_speed_scale * ((fp32)dt_ms / 10.0f);

    if (yaw_mouse_step > TUCHUAN_MOUSE_X_MAX_STEP_DEG)
    {
        yaw_mouse_step = TUCHUAN_MOUSE_X_MAX_STEP_DEG;
    }
    else if (yaw_mouse_step < -TUCHUAN_MOUSE_X_MAX_STEP_DEG)
    {
        yaw_mouse_step = -TUCHUAN_MOUSE_X_MAX_STEP_DEG;
    }

    /* 鼠标指令滤波 */
    yaw_mouse_cmd_dps = yaw_mouse_step * (1000.0f / (fp32)dt_ms);
    yaw_mouse_cmd_alpha = ((fp32)dt_ms) / (TUCHUAN_GYRO_CHASSIS_MOUSE_CMD_FILTER_TAU_MS + (fp32)dt_ms);
    if (yaw_mouse_cmd_alpha < 0.0f) yaw_mouse_cmd_alpha = 0.0f;
    else if (yaw_mouse_cmd_alpha > 1.0f) yaw_mouse_cmd_alpha = 1.0f;

    yaw_mouse_cmd_lpf_dps += yaw_mouse_cmd_alpha * (yaw_mouse_cmd_dps - yaw_mouse_cmd_lpf_dps);

    if ((has_mouse_input == 0u) && (fabs(yaw_mouse_cmd_lpf_dps) < 0.3f))
    {
        yaw_mouse_cmd_lpf_dps = 0.0f;
    }

    /* 更新鼠标累积偏移 */
    yaw_mouse_offset_angle += yaw_mouse_step;
    yaw_mouse_offset_angle = wrap_angle_deg_180(yaw_mouse_offset_angle);

    /* 获取小陀螺前馈值（底盘角速度补偿） */
    yaw_small_gyro_ff_rpm = 0.0f;
    if (get_tuchuan_gimbal_gyro_enabled() != 0u)
    {
        yaw_small_gyro_ff_rpm = vision_small_gyro_comp_ff_rpm();
    }

    /* Yaw基准角更新（叠加小陀螺前馈的积分效果） */
    fp32 yaw_ff_step = yaw_small_gyro_ff_rpm * TUCHUAN_GYRO_YAW_FEEDBACK_RPM_TO_DPS * ((fp32)dt_ms / 1000.0f);
    yaw_comp_base_angle += yaw_ff_step;
    yaw_comp_base_angle = wrap_angle_deg_360(yaw_comp_base_angle);

    /* 计算目标角度 */
    yaw_target_angle = yaw_comp_base_angle + yaw_mouse_offset_angle;
    yaw_target_angle = wrap_angle_deg_360(yaw_target_angle);

    /* 计算yaw角度误差 */
        yaw_angle_error = yaw_target_angle - yaw_angle;
    if (yaw_angle_error > ANGLE_180)
    {
        yaw_angle_error -= ANGLE_360;
    }
    else if (yaw_angle_error < -ANGLE_180)
    {
        yaw_angle_error += ANGLE_360;
    }

    /* 鼠标静止且误差很小时，归零偏移量 */
    if ((has_mouse_input == 0u) &&
    (fabs(yaw_angle_error) < TUCHUAN_MOUSE_IDLE_REBASE_YAW_ERR_DEG) &&
    (fabs((fp32)yaw_motor->speed_rpm) < TUCHUAN_MOUSE_IDLE_REBASE_SPEED_RPM))
    {
        yaw_comp_base_angle = yaw_target_angle;
        yaw_comp_base_angle = wrap_angle_deg_360(yaw_comp_base_angle);
        yaw_mouse_offset_angle = 0.0f;
        yaw_angle_error = 0.0f;
    }

    /* Yaw轴PID计算（角度环输出作为速度环目标） */
    yaw_angle_out = PID_calc(&yaw_angle_pid, 0.0f, yaw_angle_error);

    /* 速度反馈滤波 */
    yaw_speed_feedback_rpm = (fp32)yaw_motor->speed_rpm;
    yaw_feedback_alpha = ((fp32)dt_ms) / (TUCHUAN_GYRO_FEEDBACK_FILTER_TAU_MS + (fp32)dt_ms);
    if (yaw_feedback_alpha < 0.0f) yaw_feedback_alpha = 0.0f;
    else if (yaw_feedback_alpha > 1.0f) yaw_feedback_alpha = 1.0f;

    yaw_speed_feedback_lpf_rpm += yaw_feedback_alpha * (yaw_speed_feedback_rpm - yaw_speed_feedback_lpf_rpm);
    yaw_speed_out = PID_calc(&yaw_speed_pid, yaw_speed_feedback_lpf_rpm, yaw_angle_out);

    /* ==================== 视觉控制 Pitch 轴 ==================== */
    /* 处理视觉输入 */
    if (target_valid != 0u)
    {
        y_error = (fp32)y_offset;
        last_vision_tick = now_tick;
    }
    else
    {
        y_error = 0.0f;
    }

    /* 视觉误差限制 */
    if (y_error > VISION_MAX_ERROR)
    {
        y_error = VISION_MAX_ERROR;
    }
    else if (y_error < -VISION_MAX_ERROR)
    {
        y_error = -VISION_MAX_ERROR;
    }

    /* 远距离模式选择 */
    far_active = (uint8_t)((control_far_active != 0u) ? 1u : 0u);
    pitch_angle_pid_use = (far_active != 0u) ? &g_vision_pitch_angle_pid_far : &g_vision_pitch_angle_pid;

    /* 视觉误差低通滤波 */
    pitch_error_lpf += VISION_SMOOTHING * (y_error - pitch_error_lpf);

    /* 死区处理 */
    if ((pitch_error_lpf < VISION_Y_DEADZONE) && (pitch_error_lpf > -VISION_Y_DEADZONE))
    {
        pitch_error_lpf = 0.0f;
    }

    /* 计算pitch角度误差 */
    pitch_angle_error = -pitch_error_lpf * VISION_Y_TO_PITCH_ERR_GAIN;

    if (pitch_angle_error > VISION_XY_PITCH_MAX_ERR_DEG)
    {
        pitch_angle_error = VISION_XY_PITCH_MAX_ERR_DEG;
    }
    else if (pitch_angle_error < -VISION_XY_PITCH_MAX_ERR_DEG)
    {
        pitch_angle_error = -VISION_XY_PITCH_MAX_ERR_DEG;
    }

    /* 检查视觉目标超时 */
    vision_age_ms = 0u;
    if (last_vision_tick != 0u)
    {
        vision_age_ms = (now_tick >= last_vision_tick) ? (now_tick - last_vision_tick) : 0u;
    }

    /* 更新pitch目标角 */
    if ((target_valid != 0u) && (vision_age_ms <= 120u))
    {
        pitch_target_angle = pitch_angle + pitch_angle_error;
        pitch_target_angle = pitch_target_guard_inside(pitch_motor, pitch_target_angle);

        if (fallback_active != 0u)
        {
            pitch_angle_error *= VISION_XY_FALLBACK_SCALE;
            pitch_angle_pid_use->Iout = 0.0f;
        }
    }
    else
    {
        pitch_target_angle = pitch_limit_protection_target(0.0f, pitch_motor);
        pitch_angle_error = 0.0f;
    }

    /* 计算pitch当前误差 */
    fp32 pitch_current_error = pitch_target_angle - pitch_angle;
    if (pitch_current_error > ANGLE_180)
    {
        pitch_current_error -= ANGLE_360;
    }
    else if (pitch_current_error < -ANGLE_180)
    {
        pitch_current_error += ANGLE_360;
    }

        /* Pitch角度环PID */
        pitch_angle_out = PID_calc(pitch_angle_pid_use, 0.0f, pitch_current_error);
        pitch_angle_out -= ((fp32)pitch_motor->speed_rpm) * VISION_XY_PITCH_SPEED_DAMP;

        /* Pitch速度环PID */
        pitch_speed_out = PID_calc(&pitch_speed_pid, pitch_motor->speed_rpm, pitch_angle_out);
        pitch_speed_out += vision_pitch_gravity_ff_current(pitch_motor);

        /* 输出限制 */
        gimbal_out.yaw = (int16_t)yaw_speed_out;
        gimbal_out.pitch = pitch_limit_protection((int16_t)pitch_speed_out, pitch_motor, pitch_current_error);

        /* 更新状态 */
        last_mouse_tick = now_tick;
        last_yaw_small_gyro_ff_rpm = yaw_small_gyro_ff_rpm;

    /* 清除PID积分 */
    if ((target_valid == 0u) || (vision_age_ms > 120u))
    {
     PID_clear(pitch_angle_pid_use);
    }
}


/* 限幅保护函数实现 */
int16_t pitch_limit_protection(int16_t pitch_current, const motor_measure_t *pitch_motor, float pitch_angle_error)
{
    static int16_t last_out_current = 0;
    const int16_t hard_min_ecd = PITCH_ENCODER_MIN;
    const int16_t hard_max_ecd = PITCH_ENCODER_MAX;
    const int16_t soft_min_ecd = PITCH_SAFE_MIN;
    const int16_t soft_max_ecd = PITCH_SAFE_MAX;
    const int16_t recover_min_ecd = PITCH_SAFE_MIN + 10;
    const int16_t recover_max_ecd = PITCH_SAFE_MAX - 10;
    int16_t encoder;
    int32_t out_current;
    int32_t delta;
    int32_t max_delta;

    
    if (pitch_motor == NULL)
    {
        return 0;
    }

    encoder = pitch_motor->ecd;
    out_current = pitch_current;

    if ((pitch_current == 0) &&
        (pitch_angle_error < VISION_ANGLE_STATIC_LOCK_PITCH_DEG) &&
        (pitch_angle_error > -VISION_ANGLE_STATIC_LOCK_PITCH_DEG) &&
        (pitch_motor->speed_rpm < VISION_ANGLE_STATIC_LOCK_SPEED_RPM) &&
        (pitch_motor->speed_rpm > -VISION_ANGLE_STATIC_LOCK_SPEED_RPM))
    {
        last_out_current = 0;
        return 0;
    }

    // Hard limit recovery: if outside hard bound, force pull-back current toward inside target.
    if (encoder <= hard_min_ecd)
    {
        int16_t recover_err = (int16_t)(recover_min_ecd - encoder);
        out_current = 1400 + (int32_t)recover_err * 35;
        if (out_current > 6000)
        {
            out_current = 6000;
        }
    }
    else if (encoder >= hard_max_ecd)
    {
        int16_t recover_err = (int16_t)(encoder - recover_max_ecd);
        out_current = -(1400 + (int32_t)recover_err * 35);
        if (out_current < -6000)
        {
            out_current = -6000;
        }
    }
    else
    {
        // Soft zone: progressively suppress current that keeps pushing outward.
        if ((encoder < soft_min_ecd) && (out_current < 0))
        {
            fp32 ratio = (fp32)(encoder - hard_min_ecd) / (fp32)(soft_min_ecd - hard_min_ecd);
            if (ratio < 0.0f)
            {
                ratio = 0.0f;
            }
            else if (ratio > 1.0f)
            {
                ratio = 1.0f;
            }
            out_current = (int32_t)((fp32)out_current * ratio * ratio);
        }
        else if ((encoder > soft_max_ecd) && (out_current > 0))
        {
            fp32 ratio = (fp32)(hard_max_ecd - encoder) / (fp32)(hard_max_ecd - soft_max_ecd);
            if (ratio < 0.0f)
            {
                ratio = 0.0f;
            }
            else if (ratio > 1.0f)
            {
                ratio = 1.0f;
            }
            out_current = (int32_t)((fp32)out_current * ratio * ratio);
        }
    }

    if (out_current > 28000)
    {
        out_current = 28000;
    }
    else if (out_current < -28000)
    {
        out_current = -28000;
    }

        // Slew-rate: faster in center for hand-follow, slower near limits for stability.
    max_delta = 2200;
    if ((encoder > (soft_min_ecd + 80)) && (encoder < (soft_max_ecd - 80)))
    {
        max_delta = 4200;
    }

    delta = out_current - last_out_current;
    if (delta > max_delta)
    {
        out_current = last_out_current + max_delta;
    }
    else if (delta < -max_delta)
    {
        out_current = last_out_current - max_delta;
    }

    last_out_current = (int16_t)out_current;
    return (int16_t)out_current;
}
/* 清除云台输出 */
void gimbal_output_clear(void)
{
    gimbal_out.yaw     = 0;
    gimbal_out.pitch   = 0;
    gimbal_out.trigger = 0;
    gimbal_out.reserved = 0;
}

// 发送云台控制命令建议一个周期调用一次，不然容易电机数据冲突疯转抽搐
void gimbal_output_send(void)
{
    CAN_cmd_gimbal(
        gimbal_out.yaw,
        gimbal_out.pitch,
        gimbal_out.trigger,
        gimbal_out.reserved
    );
}
//靶车的代码放着，先不删了，后续可能还会用到
void Gimbal_ba(int16_t speed_set)
{
    const motor_measure_t *yaw_motor = get_yaw_gimbal_motor_measure_point();
    char buffer[64];
    int16_t speed_goal = speed_set;
    fp32 yaw_speed_out = PID_calc(&yaw_speed_pid, yaw_motor->speed_rpm, speed_goal);
 
    // snprintf(buffer, sizeof(buffer), "%.3f,%.3f\r\n",yaw_motor->speed_rpm,speed_goal);
    // HAL_UART_Transmit(&huart6, (uint8_t *)buffer, strlen(buffer), HAL_MAX_DELAY); 
    gimbal_out.yaw   = (int16_t)yaw_speed_out;
}

// 全局变量定义
static int16_t gimbal_angle_offset = 0;  // 云台角度偏移量
static float gimbal_angle = 0;            // 云台当前角度（-180° 到 180°）
static uint16_t last_encoder = 0;          // 上次编码器值，用于处理过零


/*
    * @brief       云台角度初始化
    * @note        计算初始偏移量，使当前编码器位置对应目标角度
    * @param[in]   init_target_angle: 初始目标角度（0-360°）
    * @retval      none
*/
// 初始化函数：在系统启动时调用一次
void Gimbal_Angle_Init(float init_target_angle)
{
    const motor_measure_t *yaw_motor = get_yaw_gimbal_motor_measure_point();
    uint16_t encoder = yaw_motor->ecd;
    
    // 将编码器值转换为角度（0-360°）
    float raw_angle = (encoder * 360.0f) / MOTOR_ECD_RANGE;
    
    // 计算偏移量，使当前角度等于目标角度
    gimbal_angle_offset = (int16_t)(init_target_angle - raw_angle);
    
    // 保存上次编码器值
    last_encoder = encoder;
    
    // 设置初始角度
    gimbal_angle = init_target_angle;
}

// 主函数：周期调用更新云台角度
void Gimbal_ecd_angle(void)
{
    const motor_measure_t *yaw_motor = get_yaw_gimbal_motor_measure_point();
    uint16_t encoder = yaw_motor->ecd;
    
    // 方法1：直接计算（适用于绝对编码器）
    float raw_angle = (encoder * 360.0f) / MOTOR_ECD_RANGE;  
    
    // 加上偏移量，得到相对于初始位置的角度
    float angle_with_offset = raw_angle -150;
    
    // 归一化
    while(angle_with_offset >=360) angle_with_offset -= 360;
    while(angle_with_offset < 0) angle_with_offset += 360;
    
    // 转换到-180°到180°范围
    gimbal_angle = angle_with_offset;
    if(gimbal_angle > 180) gimbal_angle -= 360;
    if(gimbal_angle < -180) gimbal_angle += 360;
    // 保存当前编码器值用于后续处理（如果需要）
    last_encoder = encoder;
}

// 获取云台角度
float Get_Gimbal_Angle(void)
{
    return gimbal_angle;
}































































