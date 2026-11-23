#include "state_machine.h"

#include "Control_Ghassis.h"
#include "bsp_led.h"
#include "infantry_config.h"
#include "infantry_imu.h"
#include "infantry_indicator.h"
#include "usart6_remote_control.h"

#if (INFANTRY_SLIPRING_TEST == 0u)
static float wrap_deg(float angle)
{
    while (angle > 180.0f) angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}
#endif

void state_machine_init(state_machine_t *sm)
{
    if (sm == 0) return;
    sm->system_mode = SYSTEM_DISABLE;
    sm->imu_ready = 0u;
    sm->enabled = 0u;
    sm->small_gyro_active = 0u;
    sm->fault = 0u;
}

void state_machine_update(state_machine_t *sm, const RC_ctrl_t *rc_data)
{
    uint8_t remote_alive;
#if (INFANTRY_SLIPRING_TEST == 0u)
    const usart1_gimbal_status_t *gimbal_status = get_usart1_gimbal_status();
    uint32_t now = HAL_GetTick();
    uint8_t gimbal_status_alive;
#endif

    if ((sm == 0) || (rc_data == 0)) return;

    sm->imu_ready = Infantry_IMU_IsReady();
    remote_alive = usart1_remote_is_online();
#if (INFANTRY_SLIPRING_TEST == 0u)
    gimbal_status_alive = (uint8_t)((gimbal_status->update_tick != 0u) &&
                                    ((uint32_t)(now - gimbal_status->update_tick) <= INFANTRY_LINK_TIMEOUT_MS));
#endif

    if ((remote_alive == 0u) || (CAN_chassis_motors_online() == 0u) ||
        switch_is_up(rc_data->rc.s[RC_S_LEFT]))
    {
        sm->system_mode = SYSTEM_DISABLE;
    }
    else if (switch_is_mid(rc_data->rc.s[RC_S_LEFT]) && (sm->imu_ready != 0u)
#if (INFANTRY_SLIPRING_TEST == 0u)
             && (gimbal_status_alive != 0u) && (get_usart1_gimbal_imu_ready() != 0u)
#endif
            )
    {
        sm->system_mode = SYSTEM_MANUAL;
    }
#if (INFANTRY_SLIPRING_TEST == 0u)
    else if (switch_is_down(rc_data->rc.s[RC_S_LEFT]) &&
             (sm->imu_ready != 0u) &&
             (gimbal_status_alive != 0u) &&
             (get_usart1_gimbal_imu_ready() != 0u))
    {
        sm->system_mode = SYSTEM_SMALL_GYRO;
    }
#endif
    else
    {
        sm->system_mode = SYSTEM_DISABLE;
    }

    sm->enabled = (uint8_t)(sm->system_mode != SYSTEM_DISABLE);
    sm->small_gyro_active = (uint8_t)(sm->system_mode == SYSTEM_SMALL_GYRO);
    sm->fault = (uint8_t)(((sm->imu_ready == 0u) ? 1u : 0u) |
                          ((CAN_chassis_motors_online() == 0u) ? 2u : 0u));
#if (INFANTRY_SLIPRING_TEST == 0u)
    if (gimbal_status_alive == 0u) sm->fault |= 4u;
    else if (get_usart1_gimbal_imu_ready() == 0u) sm->fault |= 8u;
#endif
}

void state_machine_execute(state_machine_t *sm)
{
#if (INFANTRY_SLIPRING_TEST == 0u)
    const usart1_gimbal_status_t *gimbal_status = get_usart1_gimbal_status();
    uint32_t now = HAL_GetTick();
    uint8_t gimbal_status_alive;
#endif
    static uint8_t imu_ready_beeped;

    if (sm == 0) return;

#if (INFANTRY_SLIPRING_TEST == 0u)
    gimbal_status_alive = (uint8_t)((gimbal_status->update_tick != 0u) &&
                                    ((uint32_t)(now - gimbal_status->update_tick) <= INFANTRY_LINK_TIMEOUT_MS));
#endif

    /* Announce local IMU completion once per boot, including while stopped. */
    if ((sm->imu_ready != 0u) && (imu_ready_beeped == 0u))
    {
        Infantry_Indicator_RequestBeeps(1u);
        imu_ready_beeped = 1u;
    }

#if (INFANTRY_SLIPRING_TEST == 0u)
    /* Refresh the measured angle even when stopped; MOVE must not show an old
     * drive angle after the gimbal or chassis is moved while disabled. */
    if ((sm->imu_ready != 0u) && (gimbal_status_alive != 0u) &&
        (get_usart1_gimbal_imu_ready() != 0u))
    {
        yaw_angle = wrap_deg(INFANTRY_CHASSIS_YAW_SIGN *
                             (gimbal_status->yaw_angle_deg - Infantry_IMU_GetYawDeg()) +
                             INFANTRY_GIMBAL_BOOT_OFFSET_DEG);
    }
#endif

    chassis_output_clear();
    Chassis_Set_Small_Gyro(sm->small_gyro_active);

    if (sm->enabled == 0u)
    {
        Chassis_ResetSpeedPID();
        BSP_LED_Show(INFANTRY_LED_RED);
        return;
    }

#if (INFANTRY_SLIPRING_TEST != 0u)
    RC_Control_Chassis(CHASSIS_VECTOR_NO_FOLLOW_YAW);
#else
    if ((gimbal_status_alive != 0u) && (get_usart1_gimbal_imu_ready() != 0u))
    {
        RC_Control_Chassis(CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW);
    }
    else
    {
        /* Never reinterpret the same stick input in chassis coordinates.
         * Both normal drive and spinning drive require a valid muzzle heading. */
        sm->system_mode = SYSTEM_DISABLE;
        sm->enabled = 0u;
        sm->small_gyro_active = 0u;
        Chassis_Set_Small_Gyro(0u);
        Chassis_ResetSpeedPID();
        BSP_LED_Show(INFANTRY_LED_RED);
        return;
    }
#endif
    BSP_LED_Show(INFANTRY_LED_GREEN);
}

uint8_t state_machine_is_enabled(const state_machine_t *sm)
{
    return (sm != 0) ? sm->enabled : 0u;
}

uint8_t state_machine_small_gyro_active(const state_machine_t *sm)
{
    return (sm != 0) ? sm->small_gyro_active : 0u;
}
