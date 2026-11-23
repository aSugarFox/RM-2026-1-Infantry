#include "state_machine.h"

#include "bsp_led.h"
#include "CAN_receive.h"
#include "infantry_config.h"
#include "infantry_gimbal.h"
#include "infantry_imu.h"
#include "infantry_indicator.h"
#include "usart6_remote_control.h"

void state_machine_init(state_machine_t *sm)
{
    if (sm == 0) return;
    sm->system_mode = SYSTEM_DISABLE;
    sm->launcher_mode = LAUNCHER_DISABLE;
    sm->prev_sright = RC_SW_MID;
    sm->gimbal_ready = 0u;
    sm->shoot_ready = 0u;
    sm->estop = 1u;
    sm->reverse_until_tick = 0u;
}

void state_machine_update(state_machine_t *sm, const RC_ctrl_t *rc_data)
{
    uint32_t now;
    uint32_t rc_tick = get_remote_control_update_tick();
    uint8_t rc_alive;
#if ((INFANTRY_PITCH_CALIBRATION_TEST == 0u) && (INFANTRY_SLIPRING_TEST == 0u))
    uint32_t chassis_tick = get_usart1_chassis_status_update_tick();
    uint8_t chassis_alive;
    uint8_t local_imu_ready;
    uint8_t chassis_imu_ready;
    uint8_t chassis_enabled;
    uint8_t all_imu_ready;
    uint8_t right_switch;
#endif

    if ((sm == 0) || (rc_data == 0)) return;

    /* RC and chassis ticks above must be captured before this clock read. */
    now = HAL_GetTick();
    rc_alive = (uint8_t)((rc_tick != 0u) &&
                         ((uint32_t)(now - rc_tick) <= INFANTRY_LINK_TIMEOUT_MS) &&
                         (RC_data_is_error() == 0u));
#if (INFANTRY_SLIPRING_TEST != 0u)
    sm->estop = (uint8_t)((rc_alive == 0u) ||
                          !switch_is_mid(rc_data->rc.s[RC_S_LEFT]));
    sm->system_mode = (sm->estop != 0u) ? SYSTEM_DISABLE : SYSTEM_MANUAL;
    sm->launcher_mode = LAUNCHER_DISABLE;
    sm->shoot_ready = 0u;
#if (INFANTRY_SLIPRING_YAW_TEST != 0u)
    sm->gimbal_ready = CAN_yaw_motor_online();
#else
    sm->gimbal_ready = 0u;
#endif
#elif (INFANTRY_PITCH_CALIBRATION_TEST != 0u)
    /* Left middle enables pitch calibration; both other positions disable it. */
    sm->estop = (uint8_t)((rc_alive == 0u) ||
                          !switch_is_mid(rc_data->rc.s[RC_S_LEFT]));
    sm->system_mode = (sm->estop != 0u) ? SYSTEM_DISABLE : SYSTEM_MANUAL;
    sm->launcher_mode = LAUNCHER_DISABLE;
    sm->shoot_ready = 0u;
    sm->gimbal_ready = Infantry_PitchCalibration_IsReady();
#else
    chassis_alive = (uint8_t)((chassis_tick != 0u) &&
                              ((uint32_t)(now - chassis_tick) <= INFANTRY_LINK_TIMEOUT_MS));
    local_imu_ready = Infantry_IMU_IsReady();
    chassis_imu_ready = get_usart1_chassis_imu_ready();
    chassis_enabled = get_usart1_chassis_enabled();
    all_imu_ready = (uint8_t)((local_imu_ready != 0u) && (chassis_imu_ready != 0u));

#if (INFANTRY_SHOOTER_ONLY_TEST != 0u)
    /* The test fixture has no chassis, yaw motor, pitch motor, or local IMU. */
    local_imu_ready = 1u;
    chassis_imu_ready = 1u;
    chassis_enabled = 1u;
    chassis_alive = 1u;
    all_imu_ready = 1u;
    sm->gimbal_ready = Infantry_Shooter_IsReady();
#else
    sm->gimbal_ready = Infantry_Gimbal_IsReady();
#endif
    sm->estop = (uint8_t)((rc_alive == 0u) ||
                          (chassis_alive == 0u) ||
                          switch_is_up(rc_data->rc.s[RC_S_LEFT]));

    if (sm->estop != 0u)
    {
        sm->system_mode = SYSTEM_DISABLE;
    }
    else if (switch_is_mid(rc_data->rc.s[RC_S_LEFT]) &&
             (local_imu_ready != 0u) && (chassis_enabled != 0u))
    {
        sm->system_mode = SYSTEM_MANUAL;
    }
    else if (switch_is_down(rc_data->rc.s[RC_S_LEFT]) &&
             (all_imu_ready != 0u) && (chassis_enabled != 0u) &&
             (get_usart1_small_gyro_enable() != 0u))
    {
        sm->system_mode = SYSTEM_SMALL_GYRO;
    }
    else
    {
        sm->system_mode = SYSTEM_DISABLE;
    }

#if (INFANTRY_SHOOTER_ONLY_TEST != 0u)
    sm->shoot_ready = (uint8_t)((sm->system_mode != SYSTEM_DISABLE) &&
                                (Infantry_Shooter_IsReady() != 0u));
#else
    sm->shoot_ready = (uint8_t)((sm->system_mode != SYSTEM_DISABLE) &&
                                (sm->gimbal_ready != 0u) &&
                                (Infantry_Shooter_IsReady() != 0u) &&
                                (chassis_enabled != 0u) &&
                                (all_imu_ready != 0u));
#endif

    right_switch = (uint8_t)rc_data->rc.s[RC_S_RIGHT];
    if (sm->shoot_ready == 0u)
    {
        sm->launcher_mode = LAUNCHER_DISABLE;
    }
    else if (switch_is_up(right_switch))
    {
        sm->launcher_mode = LAUNCHER_ENABLE;
    }
    else if (switch_is_down(right_switch))
    {
        if (!switch_is_down(sm->prev_sright))
        {
            sm->reverse_until_tick = now + INFANTRY_TRIGGER_REVERSE_MS;
        }
        if ((int32_t)(now - sm->reverse_until_tick) < 0)
        {
            sm->launcher_mode = LAUNCHER_REVERSE;
        }
        else
        {
            sm->launcher_mode = LAUNCHER_REVERSE_HOLD;
        }
    }
    else
    {
        sm->launcher_mode = LAUNCHER_DISABLE;
    }
    /* Entering readiness with right-down starts one reverse pulse, too. */
    sm->prev_sright = (sm->shoot_ready != 0u) ? right_switch : RC_SW_MID;
#endif
}

void state_machine_execute(state_machine_t *sm)
{
    const RC_ctrl_t *rc_data = get_remote_control_point();
    static uint8_t imu_ready_beeped;
#if ((INFANTRY_SLIPRING_TEST == 0u) && (INFANTRY_PITCH_CALIBRATION_TEST == 0u))
    infantry_shoot_mode_e shoot_mode = INFANTRY_SHOOT_OFF;
    uint8_t gimbal_enable;
#endif

    if ((sm == 0) || (rc_data == 0)) return;

    /* Local IMU completion is independent of motor readiness and the stop switch.
     * Queue once per boot; the main-loop indicator poll plays both short beeps. */
    if ((imu_ready_beeped == 0u) && (Infantry_IMU_IsReady() != 0u))
    {
        Infantry_Indicator_RequestBeeps(2u);
        imu_ready_beeped = 1u;
    }

#if (INFANTRY_SLIPRING_TEST != 0u)
#if (INFANTRY_SLIPRING_YAW_TEST != 0u)
    Infantry_YawTest_Update(rc_data, (uint8_t)(sm->estop == 0u));
    sm->gimbal_ready = CAN_yaw_motor_online();
    if ((sm->estop != 0u) || (sm->gimbal_ready == 0u))
        BSP_LED_Show(INFANTRY_LED_RED);
    else if (Infantry_YawTest_GetSpeedSet() != 0.0f)
        BSP_LED_Show(INFANTRY_LED_YELLOW);
    else
        BSP_LED_Show(INFANTRY_LED_GREEN);
#else
    /* RC forwarding runs in the DBUS ISR. No gimbal CAN commands in this mode. */
    uint32_t chassis_tick = get_usart1_chassis_status_update_tick();
    uint8_t link_ready = (uint8_t)((chassis_tick != 0u) &&
        ((uint32_t)(HAL_GetTick() - chassis_tick) <= INFANTRY_LINK_TIMEOUT_MS) &&
        ((get_usart1_chassis_status()->flags & INFANTRY_CHASSIS_FLAG_REMOTE_READY) != 0u));
    if (sm->estop != 0u)
        BSP_LED_Show(INFANTRY_LED_RED);
    else if (link_ready == 0u)
        BSP_LED_Show(INFANTRY_LED_YELLOW);
    else
        BSP_LED_Show(INFANTRY_LED_GREEN);
#endif
#elif (INFANTRY_PITCH_CALIBRATION_TEST != 0u)
    Infantry_PitchCalibration_Update(rc_data, (uint8_t)(sm->estop == 0u));
    sm->gimbal_ready = Infantry_PitchCalibration_IsReady();
    if ((sm->estop != 0u) || (sm->gimbal_ready == 0u))
        BSP_LED_Show(INFANTRY_LED_RED);
    else if ((rc_data->rc.ch[1] > DM4310_CALIB_RC_DEADZONE) ||
             (rc_data->rc.ch[1] < -DM4310_CALIB_RC_DEADZONE))
        BSP_LED_Show(INFANTRY_LED_YELLOW);
    else
        BSP_LED_Show(INFANTRY_LED_GREEN);
#else
#if (INFANTRY_SHOOTER_ONLY_TEST != 0u)
    /* CAN2 is intentionally unused while only the three shooter motors are connected. */
    gimbal_enable = 0u;
    (void)gimbal_enable;
#else
    gimbal_enable = (uint8_t)(sm->system_mode != SYSTEM_DISABLE);
    Infantry_Gimbal_Update(rc_data, gimbal_enable, INFANTRY_AUTO_AIM_REQUEST(rc_data));
#endif

    if (sm->launcher_mode == LAUNCHER_ENABLE) shoot_mode = INFANTRY_SHOOT_FIRE;
    else if (sm->launcher_mode == LAUNCHER_REVERSE) shoot_mode = INFANTRY_SHOOT_REVERSE;
    else if (sm->launcher_mode == LAUNCHER_REVERSE_HOLD) shoot_mode = INFANTRY_SHOOT_FRICTION_ONLY;
#if (INFANTRY_SHOOTER_ONLY_TEST == 0u)
    if ((shoot_mode == INFANTRY_SHOOT_FIRE) && INFANTRY_AUTO_AIM_REQUEST(rc_data) &&
        (Infantry_Vision_CanFire() == 0u)) shoot_mode = INFANTRY_SHOOT_FRICTION_ONLY;
#endif
    Infantry_Shooter_Update(shoot_mode, sm->shoot_ready);

    if ((sm->system_mode == SYSTEM_DISABLE) || (sm->gimbal_ready == 0u))
    {
        BSP_LED_Show(INFANTRY_LED_RED);
    }
    else if (sm->launcher_mode != LAUNCHER_DISABLE)
    {
        BSP_LED_Show(INFANTRY_LED_YELLOW);
    }
    else
    {
        BSP_LED_Show(INFANTRY_LED_GREEN);
    }
#endif
}

uint8_t state_machine_gimbal_ready(const state_machine_t *sm)
{
    return (sm != 0) ? sm->gimbal_ready : 0u;
}

uint8_t state_machine_shoot_ready(const state_machine_t *sm)
{
    return (sm != 0) ? sm->shoot_ready : 0u;
}

uint8_t state_machine_estop(const state_machine_t *sm)
{
    return (sm != 0) ? sm->estop : 1u;
}
