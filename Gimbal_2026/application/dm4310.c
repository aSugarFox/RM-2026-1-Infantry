#include "dm4310.h"

#include "can.h"
#include "infantry_config.h"
#include "main.h"
#include <string.h>

static dm4310_measure_t dm4310;
static volatile dm4310_debug_t dm_debug = {0u, 0u, 0u, DM4310_MASTER_ID, 0u};
static volatile uint8_t mode_query_pending;
static volatile uint8_t position_mode_ready;
static const uint8_t position_mode_request[8] = {
    (uint8_t)DM4310_ESC_ID, (uint8_t)(DM4310_ESC_ID >> 8),
    0x55u, 0x0Au, 2u, 0u, 0u, 0u};

static float uint_to_float(uint32_t value, float min_value, float max_value, uint8_t bits)
{
    return (float)value * (max_value - min_value) / (float)((1u << bits) - 1u) + min_value;
}

static uint8_t send_frame(uint32_t std_id, const uint8_t *data, uint8_t len)
{
    CAN_TxHeaderTypeDef header = {0};
    uint32_t mailbox;

    header.StdId = std_id;
    header.IDE = CAN_ID_STD;
    header.RTR = CAN_RTR_DATA;
    header.DLC = len;
    if (HAL_CAN_AddTxMessage(&DM4310_CAN, &header, (uint8_t *)data, &mailbox) != HAL_OK)
    {
        dm_debug.tx_failures++;
        return 0u;
    }
    return 1u;
}

static uint8_t send_special(uint32_t std_id, uint8_t command)
{
    uint8_t data[8] = {0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu, 0u};
    data[7] = command;
    return send_frame(std_id, data, 8u);
}

uint8_t DM4310_HandleFeedback(uint32_t std_id, const uint8_t *data, uint8_t len)
{
    uint16_t position;
    uint16_t velocity;
    uint16_t torque;
    uint32_t mode;

    dm_debug.rx_frames++;
    dm_debug.last_rx_id = (uint16_t)std_id;
    if ((data == 0) || (len != 8u)) return 0u;

    /* The motor's physical Master ID is not necessarily the configured 0x11.
       A reply to our read of CTRL_MODE identifies both the ID and startup mode. */
    if ((data[0] == (uint8_t)DM4310_ESC_ID) &&
        (data[1] == (uint8_t)(DM4310_ESC_ID >> 8)) &&
        (data[2] == 0x33u) && (data[3] == 0x0Au))
    {
        memcpy(&mode, &data[4], sizeof(mode));
        if ((mode >= 1u) && (mode <= 4u))
        {
            if (mode_query_pending != 0u)
            {
                dm_debug.feedback_id = (uint16_t)std_id;
                dm_debug.control_mode = (uint8_t)mode;
                mode_query_pending = 0u;
                position_mode_ready = (uint8_t)(mode == 2u);
                return 1u;
            }
            /* A duplicate parameter reply must not become fake position data. */
            return (uint8_t)(std_id == dm_debug.feedback_id);
        }
    }
    if (std_id != dm_debug.feedback_id) return 0u;
    /* Register replies share Master ID, but are not position feedback. */
    if (memcmp(data, position_mode_request, sizeof(position_mode_request)) == 0)
    {
        position_mode_ready = 1u;
        dm_debug.control_mode = 2u;
        return 1u;
    }
    if ((data[0] & 0x0Fu) != (DM4310_ESC_ID & 0x0Fu)) return 0u;

    position = (uint16_t)(((uint16_t)data[1] << 8) | data[2]);
    velocity = (uint16_t)(((uint16_t)data[3] << 4) | (data[4] >> 4));
    torque = (uint16_t)((((uint16_t)data[4] & 0x0Fu) << 8) | data[5]);

    dm4310.motor_id = data[0] & 0x0Fu;
    dm4310.error = data[0] >> 4;
    dm4310.position_rad = uint_to_float(position, -DM4310_P_MAX_RAD, DM4310_P_MAX_RAD, 16u);
    dm4310.velocity_rad_s = uint_to_float(velocity, -DM4310_V_MAX_RAD_S, DM4310_V_MAX_RAD_S, 12u);
    dm4310.torque_nm = uint_to_float(torque, -DM4310_T_MAX_NM, DM4310_T_MAX_NM, 12u);
    dm4310.mos_temperature = data[6];
    dm4310.rotor_temperature = data[7];
    dm4310.rx_tick = HAL_GetTick();
    return 1u;
}

uint8_t DM4310_Enable(void) { return send_special(DM4310_CONTROL_ID, 0xFCu); }
void DM4310_Disable(void)
{
    uint32_t std_id = DM4310_CONTROL_ID;
    /* Stop using the mode actually stored in the motor before switching modes. */
    if (dm_debug.control_mode != 0u)
        std_id = DM4310_ESC_ID + ((uint32_t)dm_debug.control_mode - 1u) * 0x100u;
    send_special(std_id, 0xFDu);
}
void DM4310_ClearError(void) { send_special(DM4310_CONTROL_ID, 0xFBu); }

void DM4310_QueryControlMode(void)
{
    /* STM32 HAL reads all eight buffer bytes even when DLC is four. */
    const uint8_t data[8] = {
        (uint8_t)DM4310_ESC_ID, (uint8_t)(DM4310_ESC_ID >> 8), 0x33u, 0x0Au};
    mode_query_pending = 1u;
    send_frame(0x7FFu, data, 4u);
}

const volatile dm4310_debug_t *DM4310_GetDebug(void) { return &dm_debug; }

void DM4310_RequestPositionMode(void)
{
    /* Volatile mode change only: do not save parameters or change zero/IDs. */
    position_mode_ready = 0u;
    send_frame(0x7FFu, position_mode_request, 8u);
}

uint8_t DM4310_PositionModeReady(void) { return position_mode_ready; }

uint8_t DM4310_SetPositionSpeed(float position_rad, float max_speed_rad_s)
{
    uint8_t data[8];
    if (max_speed_rad_s < 0.0f) max_speed_rad_s = -max_speed_rad_s;
    if (max_speed_rad_s > DM4310_PITCH_MAX_SPEED_RAD_S) max_speed_rad_s = DM4310_PITCH_MAX_SPEED_RAD_S;
    memcpy(&data[0], &position_rad, sizeof(float));
    memcpy(&data[4], &max_speed_rad_s, sizeof(float));
    return send_frame(DM4310_CONTROL_ID, data, 8u);
}

const dm4310_measure_t *DM4310_GetMeasure(void) { return &dm4310; }

uint8_t DM4310_IsOnline(void)
{
    return (uint8_t)((dm4310.rx_tick != 0u) &&
                     ((uint32_t)(HAL_GetTick() - dm4310.rx_tick) <= INFANTRY_MOTOR_TIMEOUT_MS));
}

uint8_t DM4310_IsHealthy(void)
{
    return (uint8_t)((DM4310_IsOnline() != 0u) && (dm4310.error <= 1u));
}
