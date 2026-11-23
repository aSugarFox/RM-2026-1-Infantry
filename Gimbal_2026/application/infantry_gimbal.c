#include "infantry_gimbal.h"

#include "CAN_receive.h"
#include "Vision_Control.h"
#include "dm4310.h"
#include "infantry_config.h"
#include "infantry_imu.h"
#include "main.h"
#include "pid.h"
#include "usart6_remote_control.h"

static pid_type_def yaw_angle_pid;
static pid_type_def yaw_speed_pid;
static pid_type_def fric_left_pid;
static pid_type_def fric_right_pid;
static pid_type_def trigger_pid;
static uint32_t shooter_gate_drops;
static uint8_t shooter_output_enabled;
static infantry_shoot_mode_e shooter_effective_mode;

static float yaw_target_deg;
static float yaw_relative_target_deg;
static uint8_t yaw_dial_follow;
static float yaw_chassis_rate_rpm;
static float yaw_feedforward;
static float pitch_target_rad;
static uint8_t yaw_target_valid;
static int8_t yaw_manual_direction;
static uint8_t pitch_target_valid;
static uint32_t last_update_tick;
static uint32_t last_dm_special_tick;

#if (INFANTRY_SLIPRING_TEST != 0u)
static float yaw_test_speed_set;
static int16_t yaw_test_command;
#endif

typedef enum
{
    PITCH_CALIB_WAIT_DISABLED = 0,
    PITCH_CALIB_WAIT_MODE,
    PITCH_CALIB_ENABLE,
    PITCH_CALIB_RUNNING,
    PITCH_CALIB_RECOVER
} pitch_calib_phase_e;
static pitch_calib_phase_e pitch_calib_phase;
static uint8_t pitch_calib_stick_active;
static uint8_t vision_fire_allowed;

static float clampf(float value, float min_value, float max_value)
{
    if (value < min_value) return min_value;
    if (value > max_value) return max_value;
    return value;
}

static float wrap_deg(float angle)
{
    while (angle > 180.0f) angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}

static uint8_t pitch_position_in_limits(void)
{
    float position = DM4310_GetMeasure()->position_rad;
    return (uint8_t)((position >= DM4310_PITCH_UP_LIMIT_RAD - DM4310_PITCH_POSITION_TOLERANCE_RAD) &&
                     (position <= DM4310_PITCH_DOWN_LIMIT_RAD + DM4310_PITCH_POSITION_TOLERANCE_RAD));
}

/* Query -> disabled -> position mode -> seed current position -> enable.
   Used by normal operation AND the already-tested calibration mode. */
static uint8_t pitch_prepare(uint8_t enable, float max_speed)
{
    const dm4310_measure_t *motor = DM4310_GetMeasure();
    uint32_t now = HAL_GetTick();
#if (INFANTRY_PITCH_CALIBRATION_TEST != 0u)
    const float startup_margin = DM4310_PITCH_LIMIT_MARGIN_RAD;
#else
    const float startup_margin = DM4310_PITCH_RECOVERY_MARGIN_RAD;
#endif
    if ((enable == 0u) || (DM4310_IsHealthy() == 0u) ||
        (motor->position_rad < DM4310_PITCH_MEASURED_UP_RAD - startup_margin) ||
        (motor->position_rad > DM4310_PITCH_MEASURED_DOWN_RAD + startup_margin))
    {
        /* Disable also polls position: the motor does not stream unsolicited feedback. */
        if ((pitch_calib_phase != PITCH_CALIB_WAIT_DISABLED) ||
            ((uint32_t)(now - last_dm_special_tick) >= DM4310_CALIB_COMMAND_PERIOD_MS))
        {
            DM4310_Disable();
            DM4310_QueryControlMode();
            last_dm_special_tick = now;
        }
        pitch_calib_phase = PITCH_CALIB_WAIT_DISABLED;
        pitch_calib_stick_active = 0u;
        pitch_target_valid = 0u;
        pitch_target_rad = motor->position_rad;
        return 0u;
    }

    if (pitch_calib_phase == PITCH_CALIB_WAIT_DISABLED)
    {
        if (motor->error == 0u)
        {
            DM4310_RequestPositionMode();
            last_dm_special_tick = now;
            pitch_calib_phase = PITCH_CALIB_WAIT_MODE;
        }
        else if ((uint32_t)(now - last_dm_special_tick) >= DM4310_CALIB_COMMAND_PERIOD_MS)
        {
            DM4310_Disable();
            last_dm_special_tick = now;
        }
        return 0u;
    }

    if (pitch_calib_phase == PITCH_CALIB_WAIT_MODE)
    {
        if ((DM4310_PositionModeReady() != 0u) && (motor->error == 0u))
        {
            /* Mode switching clears the old setpoint: seed position before enabling. */
            pitch_target_rad = motor->position_rad;
            if (DM4310_SetPositionSpeed(pitch_target_rad, max_speed) != 0u)
                pitch_calib_phase = PITCH_CALIB_ENABLE;
        }
        else if ((uint32_t)(now - last_dm_special_tick) >= DM4310_CALIB_COMMAND_PERIOD_MS)
        {
            DM4310_Disable();
            if (motor->error == 0u) DM4310_RequestPositionMode();
            last_dm_special_tick = now;
        }
        return 0u;
    }

    if (pitch_calib_phase == PITCH_CALIB_ENABLE)
    {
        /* A separate control cycle keeps the seed frame ahead of the enable frame. */
        if (DM4310_Enable() != 0u)
        {
            last_dm_special_tick = now;
            pitch_calib_phase = PITCH_CALIB_RUNNING;
        }
        return 0u;
    }

    if (motor->error != 1u)
    {
        pitch_target_valid = 0u;
        pitch_calib_stick_active = 0u;
        if ((uint32_t)(now - last_dm_special_tick) >= DM4310_CALIB_COMMAND_PERIOD_MS)
            pitch_calib_phase = PITCH_CALIB_WAIT_DISABLED;
        return 0u;
    }

#if (INFANTRY_PITCH_CALIBRATION_TEST == 0u)
    if (pitch_position_in_limits() == 0u)
    {
        /* Ignore outward RC/vision commands until the measured pitch is back
         * inside the original soft limits. This phase must not unlock firing. */
        pitch_calib_phase = PITCH_CALIB_RECOVER;
        pitch_target_valid = 0u;
        pitch_target_rad = clampf(motor->position_rad,
            DM4310_PITCH_UP_LIMIT_RAD, DM4310_PITCH_DOWN_LIMIT_RAD);
        DM4310_SetPositionSpeed(pitch_target_rad,
            clampf(max_speed, 0.0f, DM4310_PITCH_RECOVERY_SPEED_RAD_S));
        return 0u;
    }
    if (pitch_calib_phase == PITCH_CALIB_RECOVER)
    {
        pitch_target_rad = clampf(motor->position_rad,
            DM4310_PITCH_UP_LIMIT_RAD, DM4310_PITCH_DOWN_LIMIT_RAD);
        pitch_calib_phase = PITCH_CALIB_RUNNING;
    }
#endif
    pitch_target_valid = 1u;
    return 1u;
}

void Infantry_Gimbal_Init(void)
{
    const float yaw_angle_param[3] = {
        INFANTRY_YAW_ANGLE_KP, INFANTRY_YAW_ANGLE_KI, INFANTRY_YAW_ANGLE_KD};
    const float yaw_speed_param[3] = {
        INFANTRY_YAW_SPEED_KP, INFANTRY_YAW_SPEED_KI, INFANTRY_YAW_SPEED_KD};
    const float fric_param[3] = {3.0f, 0.08f, 0.0f};
    const float trigger_param[3] = {5.0f, 0.05f, 0.0f};

    PID_init(&yaw_angle_pid, PID_POSITION, yaw_angle_param,
             INFANTRY_YAW_MAX_SPEED_RPM, 30.0f);
    /* Reuse the existing position-form PID so its integral has a real bound. */
    PID_init(&yaw_speed_pid, PID_POSITION, yaw_speed_param,
             INFANTRY_YAW_MAX_CURRENT, INFANTRY_YAW_MAX_INTEGRAL);
#if (INFANTRY_SLIPRING_TEST != 0u)
    yaw_speed_pid.max_out = INFANTRY_TEST_YAW_MAX_COMMAND;
    yaw_speed_pid.max_iout = INFANTRY_TEST_YAW_MAX_COMMAND;
    yaw_test_speed_set = 0.0f;
    yaw_test_command = 0;
#endif
    PID_init(&fric_left_pid, PID_POSITION, fric_param,
             INFANTRY_FRIC_MAX_CURRENT, 1500.0f);
    PID_init(&fric_right_pid, PID_POSITION, fric_param,
             INFANTRY_FRIC_MAX_CURRENT, 1500.0f);
    PID_init(&trigger_pid, PID_POSITION, trigger_param,
             INFANTRY_TRIGGER_MAX_CURRENT, 1000.0f);
    shooter_gate_drops = 0u;
    shooter_output_enabled = 0u;
    shooter_effective_mode = INFANTRY_SHOOT_OFF;

    yaw_target_valid = 0u;
    yaw_manual_direction = 0;
    yaw_dial_follow = 0u;
    yaw_relative_target_deg = 0.0f;
    yaw_chassis_rate_rpm = 0.0f;
    yaw_feedforward = 0.0f;
    pitch_target_valid = 0u;
    last_update_tick = HAL_GetTick();
    last_dm_special_tick = 0u;
    pitch_calib_phase = PITCH_CALIB_WAIT_DISABLED;
    pitch_calib_stick_active = 0u;
    vision_fire_allowed = 0u;
}

void Infantry_Gimbal_Update(const RC_ctrl_t *rc, uint8_t enable, uint8_t auto_aim_enable)
{
    const motor_measure_t *yaw_motor = get_yaw_gimbal_motor_measure_point();
    const dm4310_measure_t *pitch_motor = DM4310_GetMeasure();
    const vision_target_offset_t *vision = get_vision_target_offset_data_point();
    uint32_t now = HAL_GetTick();
    float dt = clampf((float)(now - last_update_tick) * 0.001f, 0.0f, 0.02f);
    float yaw_now = Infantry_IMU_GetYawDeg();
    float yaw_relative_now = INFANTRY_YAW_MOTOR_DIRECTION * (float)yaw_motor->ecd * (360.0f / 8192.0f);
    float yaw_control_angle, yaw_control_rate;
    float *yaw_control_target;
    uint8_t dial_follow;
    float yaw_error, yaw_speed_set, yaw_current;
    float yaw_integral_before, yaw_output;
    uint32_t chassis_tick = get_usart1_chassis_status_update_tick();
    uint8_t pitch_running;
    uint8_t vision_tracking;
    int16_t yaw_offset, pitch_offset;
    uint8_t valid, bullet_enable, control_far, fallback;
    last_update_tick = now;
    yaw_chassis_rate_rpm = 0.0f;
    yaw_feedforward = 0.0f;

    Vision_USB_Process_TargetOffset();
    vision_fire_allowed = 0u;
    enable = (uint8_t)((enable != 0u) && (rc != 0) &&
                       (Infantry_IMU_IsReady() != 0u) && (CAN_yaw_motor_online() != 0u));
    pitch_running = pitch_prepare(enable, DM4310_PITCH_MAX_SPEED_RAD_S);
    if (enable == 0u)
    {
        yaw_target_valid = 0u;
        yaw_manual_direction = 0;
        yaw_dial_follow = 0u;
        PID_clear(&yaw_angle_pid);
        PID_clear(&yaw_speed_pid);
        CAN_cmd_gimbal(0, 0, 0, 0);
        return;
    }
    vision_tracking = (uint8_t)((auto_aim_enable != 0u) && (vision->frame_count != 0u) &&
        (vision->valid != 0u) && (vision->fallback_active == 0u) &&
        ((uint32_t)(now - vision->last_update_time) <= INFANTRY_VISION_TIMEOUT_MS));
    /* Manual dial turns the whole robot: hold the yaw joint relative to the
     * chassis. Small-gyro mode and vision aiming retain world-heading hold. */
    dial_follow = (uint8_t)(switch_is_mid(rc->rc.s[RC_S_LEFT]) &&
        ((rc->rc.ch[4] > INFANTRY_RC_DEADZONE) || (rc->rc.ch[4] < -INFANTRY_RC_DEADZONE)) &&
        (vision_tracking == 0u));
    if ((yaw_target_valid == 0u) || (yaw_dial_follow != dial_follow))
    {
        /* Capture the current angle in the new frame; never return to the
         * pre-dial heading or carry the previous frame's integral across. */
        yaw_target_deg = yaw_now;
        yaw_relative_target_deg = yaw_relative_now;
        yaw_target_valid = 1u;
        yaw_dial_follow = dial_follow;
        yaw_manual_direction = 0;
        PID_clear(&yaw_angle_pid);
        PID_clear(&yaw_speed_pid);
    }
    yaw_control_target = (dial_follow != 0u) ? &yaw_relative_target_deg : &yaw_target_deg;
    yaw_control_angle = (dial_follow != 0u) ? yaw_relative_now : yaw_now;
    yaw_control_rate = (dial_follow != 0u) ?
        INFANTRY_YAW_MOTOR_DIRECTION * (float)yaw_motor->speed_rpm : Infantry_IMU_GetGyroZRpm();
    if (vision_tracking != 0u)
    {
        yaw_manual_direction = 0;
        vision_fire_allowed = vision->bullet_enable;
        if (Vision_GetTargetOffset(&yaw_offset, &pitch_offset, &valid,
                                    &bullet_enable, &control_far, &fallback) != 0u)
        {
            /* Error is relative to the measured aim, not the previous setpoint. */
            yaw_target_deg = wrap_deg(yaw_now + INFANTRY_VISION_YAW_DIRECTION *
                                      (float)yaw_offset * INFANTRY_VISION_YAW_GAIN);
            if (pitch_running != 0u)
                pitch_target_rad = pitch_motor->position_rad + INFANTRY_VISION_PITCH_DIRECTION *
                                   (float)pitch_offset * INFANTRY_VISION_PITCH_GAIN;
        }
    }
    else
    {
        /* Missing/expired vision falls back to manual movement. */
        int8_t direction = (rc->rc.ch[0] > INFANTRY_RC_DEADZONE) ? 1 :
                           ((rc->rc.ch[0] < -INFANTRY_RC_DEADZONE) ? -1 : 0);
        if (direction != 0)
        {
            if ((yaw_manual_direction != 0) && (direction != yaw_manual_direction))
            {
                /* An explicit stick reversal cancels the previous lead and
                 * integral, rather than spending seconds driving the old way. */
                *yaw_control_target = yaw_control_angle;
                PID_clear(&yaw_angle_pid);
                PID_clear(&yaw_speed_pid);
            }
            yaw_manual_direction = direction;
            *yaw_control_target = wrap_deg(*yaw_control_target - clampf((float)rc->rc.ch[0] / 660.0f,
                -1.0f, 1.0f) * INFANTRY_YAW_RC_SPEED_DPS * dt);
            /* Limit active-stick lead in the selected frame. Releasing the
             * stick preserves the target, even after a larger disturbance. */
            *yaw_control_target = wrap_deg(yaw_control_angle + clampf(
                wrap_deg(*yaw_control_target - yaw_control_angle),
                -INFANTRY_YAW_MAX_LEAD_DEG, INFANTRY_YAW_MAX_LEAD_DEG));
        }
        if ((pitch_running != 0u) &&
            ((rc->rc.ch[1] > INFANTRY_RC_DEADZONE) || (rc->rc.ch[1] < -INFANTRY_RC_DEADZONE)))
            pitch_target_rad += DM4310_PITCH_DIRECTION * clampf((float)rc->rc.ch[1] / 660.0f,
                -1.0f, 1.0f) * DM4310_PITCH_RC_SPEED_RAD_S * dt;
    }

    yaw_error = clampf(wrap_deg(*yaw_control_target - yaw_control_angle),
                       -INFANTRY_YAW_MAX_LEAD_DEG, INFANTRY_YAW_MAX_LEAD_DEG);
    yaw_speed_set = PID_calc(&yaw_angle_pid, 0.0f, yaw_error);
    /* World hold counteracts base rotation. Joint hold follows the base and
     * uses encoder RPM, so base-rate feedforward must be disabled there. */
    if ((chassis_tick != 0u) &&
        ((uint32_t)(now - chassis_tick) <= INFANTRY_LINK_TIMEOUT_MS) &&
        (get_usart1_chassis_imu_ready() != 0u))
    {
        yaw_chassis_rate_rpm = get_usart1_chassis_status()->chassis_gyro_speed_rpm;
        if (dial_follow == 0u)
            yaw_feedforward = -INFANTRY_YAW_CHASSIS_FF_PER_RPM * yaw_chassis_rate_rpm;
    }
    yaw_integral_before = yaw_speed_pid.Iout;
    (void)PID_calc(&yaw_speed_pid, yaw_control_rate, yaw_speed_set);
    yaw_output = yaw_speed_pid.Pout + yaw_speed_pid.Iout + yaw_speed_pid.Dout + yaw_feedforward;
    /* Stop integrating farther into the combined PID + feedforward limit.
     * Opposite-sign error can still unwind the integral during recovery. */
    if (((yaw_output > yaw_speed_pid.max_out) && (yaw_speed_pid.error[0] > 0.0f)) ||
        ((yaw_output < -yaw_speed_pid.max_out) && (yaw_speed_pid.error[0] < 0.0f)))
    {
        yaw_speed_pid.Iout = yaw_integral_before;
        yaw_output = yaw_speed_pid.Pout + yaw_speed_pid.Iout + yaw_speed_pid.Dout + yaw_feedforward;
    }
    yaw_speed_pid.out = clampf(yaw_output, -yaw_speed_pid.max_out, yaw_speed_pid.max_out);
    yaw_current = INFANTRY_YAW_MOTOR_DIRECTION * yaw_speed_pid.out;
    CAN_cmd_gimbal((int16_t)yaw_current, 0, 0, 0);

    if (pitch_running != 0u)
    {
        pitch_target_rad = clampf(pitch_target_rad,
            pitch_motor->position_rad - DM4310_PITCH_MAX_LEAD_RAD,
            pitch_motor->position_rad + DM4310_PITCH_MAX_LEAD_RAD);
        pitch_target_rad = clampf(pitch_target_rad,
            DM4310_PITCH_UP_LIMIT_RAD, DM4310_PITCH_DOWN_LIMIT_RAD);
        /* Hold the final setpoint at stick center; do not pull it back to a noisy sample. */
        DM4310_SetPositionSpeed(pitch_target_rad, DM4310_PITCH_MAX_SPEED_RAD_S);
    }
}

uint8_t Infantry_Vision_CanFire(void) { return vision_fire_allowed; }
float Infantry_Gimbal_GetPitchTarget(void) { return pitch_target_rad; }
uint8_t Infantry_Gimbal_GetPitchPhase(void) { return (uint8_t)pitch_calib_phase; }
float Infantry_Gimbal_GetYawTarget(void)
{
    if ((yaw_target_valid != 0u) && (yaw_dial_follow != 0u))
    {
        float relative_now = INFANTRY_YAW_MOTOR_DIRECTION *
            (float)get_yaw_gimbal_motor_measure_point()->ecd * (360.0f / 8192.0f);
        return wrap_deg(Infantry_IMU_GetYawDeg() + wrap_deg(yaw_relative_target_deg - relative_now));
    }
    return (yaw_target_valid != 0u) ? yaw_target_deg : Infantry_IMU_GetYawDeg();
}
uint8_t Infantry_Gimbal_IsDialFollowing(void) { return yaw_dial_follow; }
float Infantry_Gimbal_GetYawSpeedSet(void) { return yaw_speed_pid.set; }
float Infantry_Gimbal_GetChassisYawRate(void) { return yaw_chassis_rate_rpm; }
float Infantry_Gimbal_GetYawFeedforward(void) { return yaw_feedforward; }

void Infantry_Shooter_Update(infantry_shoot_mode_e mode, uint8_t enable)
{
    const motor_measure_t *left = get_fric_wheel_left_motor_measure_point();
    const motor_measure_t *right = get_fric_wheel_right_motor_measure_point();
    const motor_measure_t *trigger = get_trigger_motor_measure_point();
    float left_set = 0.0f;
    float right_set = 0.0f;
    float trigger_set = 0.0f;
    float left_current;
    float right_current;
    float trigger_current;

    /* Count readiness loss during output, including an operator left-stop.
     * A normal right-switch OFF is not a readiness loss. */
    if ((shooter_output_enabled != 0u) && (enable == 0u)) shooter_gate_drops++;
    shooter_output_enabled = (uint8_t)((enable != 0u) && (mode != INFANTRY_SHOOT_OFF));
    shooter_effective_mode = (shooter_output_enabled != 0u) ? mode : INFANTRY_SHOOT_OFF;

    if ((enable == 0u) || (mode == INFANTRY_SHOOT_OFF))
    {
        PID_clear(&fric_left_pid);
        PID_clear(&fric_right_pid);
        PID_clear(&trigger_pid);
        CAN_cmd_shoot(0, 0, 0);
        return;
    }

    left_set = INFANTRY_FRIC_LEFT_DIRECTION * INFANTRY_FRIC_SPEED_RPM;
    right_set = INFANTRY_FRIC_RIGHT_DIRECTION * INFANTRY_FRIC_SPEED_RPM;
    if (mode == INFANTRY_SHOOT_FIRE)
    {
        trigger_set = INFANTRY_TRIGGER_DIRECTION * INFANTRY_TRIGGER_FORWARD_RPM;
    }
    else if (mode == INFANTRY_SHOOT_REVERSE)
    {
        trigger_set = -INFANTRY_TRIGGER_DIRECTION * INFANTRY_TRIGGER_REVERSE_RPM;
    }
    left_current = PID_calc(&fric_left_pid, (float)left->speed_rpm, left_set);
    right_current = PID_calc(&fric_right_pid, (float)right->speed_rpm, right_set);
    trigger_current = PID_calc(&trigger_pid, (float)trigger->speed_rpm, trigger_set);
    CAN_cmd_shoot((int16_t)left_current, (int16_t)right_current, (int16_t)trigger_current);
}

void Infantry_Shooter_GetDebug(infantry_shooter_debug_t *debug)
{
    const pid_type_def *pids[3] = {&trigger_pid, &fric_left_pid, &fric_right_pid};
    const motor_measure_t *motors[3] = {get_trigger_motor_measure_point(),
        get_fric_wheel_left_motor_measure_point(), get_fric_wheel_right_motor_measure_point()};
    uint32_t primask = __get_PRIMASK();
    uint8_t i;

    /* Keep rpm and measured-current fields from the same feedback snapshot. */
    __disable_irq();
    for (i = 0u; i < 3u; i++)
    {
        debug->actual_rpm[i] = motors[i]->speed_rpm;
        debug->feedback_current[i] = motors[i]->given_current;
    }
    if (primask == 0u) __enable_irq();
    for (i = 0u; i < 3u; i++)
    {
        debug->target_rpm[i] = (int16_t)pids[i]->set;
        debug->command[i] = (int16_t)pids[i]->out;
    }
    debug->gate_drops = shooter_gate_drops;
    debug->enabled = shooter_output_enabled;
    debug->mode = (uint8_t)shooter_effective_mode;
}

uint8_t Infantry_Gimbal_IsReady(void)
{
    return (uint8_t)((Infantry_IMU_IsReady() != 0u) &&
                     (CAN_yaw_motor_online() != 0u) &&
                     (DM4310_IsHealthy() != 0u) && (DM4310_PositionModeReady() != 0u) &&
                     (pitch_calib_phase == PITCH_CALIB_RUNNING) &&
                     (pitch_position_in_limits() != 0u) &&
                     (pitch_target_valid != 0u) && (DM4310_GetMeasure()->error == 1u));
}

uint8_t Infantry_Shooter_IsReady(void)
{
    return CAN_shooter_motors_online();
}

#if (INFANTRY_SLIPRING_TEST != 0u)
void Infantry_YawTest_Update(const RC_ctrl_t *rc, uint8_t enable)
{
    const motor_measure_t *motor = get_yaw_gimbal_motor_measure_point();
    yaw_test_speed_set = 0.0f;
    yaw_test_command = 0;

    if ((enable == 0u) || (rc == 0) || (CAN_yaw_motor_online() == 0u) ||
        ((rc->rc.ch[0] <= INFANTRY_TEST_RC_DEADZONE) &&
         (rc->rc.ch[0] >= -INFANTRY_TEST_RC_DEADZONE)))
    {
        /* Wiring test: release the stick to stop driving, with no position return. */
        PID_clear(&yaw_speed_pid);
        CAN_cmd_gimbal(0, 0, 0, 0);
        return;
    }

    yaw_test_speed_set = INFANTRY_TEST_YAW_DIRECTION * INFANTRY_TEST_YAW_SPEED_RPM *
        clampf((float)rc->rc.ch[0] / 660.0f, -1.0f, 1.0f);
    yaw_test_command = (int16_t)PID_calc(&yaw_speed_pid,
                                        (float)motor->speed_rpm, yaw_test_speed_set);
    CAN_cmd_gimbal(yaw_test_command, 0, 0, 0);
}

float Infantry_YawTest_GetSpeedSet(void) { return yaw_test_speed_set; }
int16_t Infantry_YawTest_GetCommand(void) { return yaw_test_command; }
#endif

#if (INFANTRY_PITCH_CALIBRATION_TEST != 0u)
void Infantry_PitchCalibration_Update(const RC_ctrl_t *rc, uint8_t enable)
{
    const dm4310_measure_t *motor = DM4310_GetMeasure();
    uint32_t now = HAL_GetTick();
    float dt = clampf((float)(now - last_update_tick) * 0.001f, 0.0f, 0.05f);
    last_update_tick = now;

    if (pitch_prepare((uint8_t)((enable != 0u) && (rc != 0)),
                      DM4310_CALIB_MAX_SPEED_RAD_S) == 0u) return;

    pitch_target_valid = 1u;
    if ((rc->rc.ch[1] > DM4310_CALIB_RC_DEADZONE) ||
        (rc->rc.ch[1] < -DM4310_CALIB_RC_DEADZONE))
    {
        pitch_target_rad += DM4310_PITCH_DIRECTION *
            clampf((float)rc->rc.ch[1] / 660.0f, -1.0f, 1.0f) *
            DM4310_CALIB_RC_SPEED_RAD_S * dt;
        pitch_calib_stick_active = 1u;
    }
    else if (pitch_calib_stick_active != 0u)
    {
        /* On stick release, hold the measured position instead of chasing accumulated error. */
        pitch_target_rad = motor->position_rad;
        pitch_calib_stick_active = 0u;
    }
    /* Keep the target close to the measured position and inside the measured mechanical limits. */
    pitch_target_rad = clampf(pitch_target_rad,
        motor->position_rad - DM4310_CALIB_MAX_LEAD_RAD,
        motor->position_rad + DM4310_CALIB_MAX_LEAD_RAD);
    pitch_target_rad = clampf(pitch_target_rad,
        DM4310_PITCH_UP_LIMIT_RAD, DM4310_PITCH_DOWN_LIMIT_RAD);
    DM4310_SetPositionSpeed(pitch_target_rad, DM4310_CALIB_MAX_SPEED_RAD_S);
}

uint8_t Infantry_PitchCalibration_IsReady(void)
{
    return (uint8_t)((pitch_calib_phase == PITCH_CALIB_RUNNING) &&
                     (pitch_target_valid != 0u) && (DM4310_IsHealthy() != 0u) &&
                     (DM4310_GetMeasure()->error == 1u));
}

float Infantry_PitchCalibration_GetTarget(void) { return pitch_target_rad; }
#endif
