#include "CAN_receive.h"

#include "can.h"
#include "dm4310.h"
#include "infantry_config.h"
#include "main.h"

static motor_measure_t yaw_motor;
static motor_measure_t fric_left_motor;
static motor_measure_t fric_right_motor;
static motor_measure_t trigger_motor;
static motor_measure_t compatibility_motors[4];
static can_yaw_debug_t yaw_debug;
static volatile can_shooter_debug_t shooter_debug;

static void decode_rm_motor(motor_measure_t *motor, const uint8_t data[8])
{
    motor->last_ecd = (int16_t)motor->ecd;
    motor->ecd = (uint16_t)(((uint16_t)data[0] << 8) | data[1]);
    motor->speed_rpm = (int16_t)(((uint16_t)data[2] << 8) | data[3]);
    motor->given_current = (int16_t)(((uint16_t)data[4] << 8) | data[5]);
    motor->temperate = data[6];
    motor->rx_tick = HAL_GetTick();
}

static uint8_t send_rm_currents(CAN_HandleTypeDef *hcan,
                             uint32_t std_id,
                             int16_t current_1,
                             int16_t current_2,
                             int16_t current_3,
                             int16_t current_4)
{
    CAN_TxHeaderTypeDef header = {0};
    uint8_t data[8];
    uint32_t mailbox;

    header.StdId = std_id;
    header.IDE = CAN_ID_STD;
    header.RTR = CAN_RTR_DATA;
    header.DLC = 8u;
    data[0] = (uint8_t)(current_1 >> 8);
    data[1] = (uint8_t)current_1;
    data[2] = (uint8_t)(current_2 >> 8);
    data[3] = (uint8_t)current_2;
    data[4] = (uint8_t)(current_3 >> 8);
    data[5] = (uint8_t)current_3;
    data[6] = (uint8_t)(current_4 >> 8);
    data[7] = (uint8_t)current_4;
    return (uint8_t)(HAL_CAN_AddTxMessage(hcan, &header, data, &mailbox) == HAL_OK);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef header;
    uint8_t data[8];

    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &header, data) != HAL_OK) return;
    if ((header.IDE != CAN_ID_STD) || (header.RTR != CAN_RTR_DATA)) return;

    if ((hcan == &DM4310_CAN) &&
        (DM4310_HandleFeedback(header.StdId, data, header.DLC) != 0u))
    {
        return;
    }

    if (header.DLC != 8u) return;
    if (hcan->Instance == CAN1)
    {
        shooter_debug.rx_frames++;
        shooter_debug.last_rx_id = (uint16_t)header.StdId;
        if ((header.StdId >= 0x201u) && (header.StdId <= 0x208u))
            shooter_debug.seen_ids |= (uint16_t)(1u << (header.StdId - 0x201u));
        if (header.StdId == INFANTRY_CAN_FRIC_LEFT_ID) decode_rm_motor(&fric_left_motor, data);
        else if (header.StdId == INFANTRY_CAN_FRIC_RIGHT_ID) decode_rm_motor(&fric_right_motor, data);
        else if (header.StdId == INFANTRY_CAN_TRIGGER_ID) decode_rm_motor(&trigger_motor, data);
    }
    else if (hcan->Instance == CAN2)
    {
        if (header.StdId == INFANTRY_CAN_YAW_FEEDBACK_ID) decode_rm_motor(&yaw_motor, data);
    }
}

void CAN_cmd_gimbal(int16_t yaw, int16_t pitch, int16_t shoot, int16_t rev)
{
    (void)pitch;
    (void)shoot;
    (void)rev;
    yaw_debug.command = yaw;
    if (send_rm_currents(&hcan2, INFANTRY_CAN_YAW_CMD_ID, yaw, 0, 0, 0) != 0u)
        yaw_debug.tx_queued++;
    else
        yaw_debug.tx_failed++;
}

const can_yaw_debug_t *CAN_yaw_get_debug(void) { return &yaw_debug; }

void CAN_cmd_shoot(int16_t fric_left, int16_t fric_right, int16_t trigger)
{
    if (send_rm_currents(&hcan1, INFANTRY_CAN_SHOOT_CMD_ID, trigger, fric_left, fric_right, 0) == 0u)
        shooter_debug.tx_failed++;
}

void CAN_cmd_fric_wheel(int16_t fric_left, int16_t fric_right, int16_t rev1, int16_t rev2)
{
    (void)rev2;
    CAN_cmd_shoot(fric_left, fric_right, rev1);
}

void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    send_rm_currents(&hcan1, INFANTRY_CAN_SHOOT_CMD_ID, motor1, motor2, motor3, motor4);
}

void CAN_cmd_chassis_reset_ID(void)
{
    send_rm_currents(&hcan1, 0x700u, 0, 0, 0, 0);
}

const motor_measure_t *get_yaw_gimbal_motor_measure_point(void) { return &yaw_motor; }
const motor_measure_t *get_pitch_gimbal_motor_measure_point(void) { return &compatibility_motors[0]; }
const motor_measure_t *get_trigger_motor_measure_point(void) { return &trigger_motor; }
const motor_measure_t *get_fric_wheel_left_motor_measure_point(void) { return &fric_left_motor; }
const motor_measure_t *get_fric_wheel_right_motor_measure_point(void) { return &fric_right_motor; }
const motor_measure_t *get_chassis_motor_measure_point(uint8_t i) { return &compatibility_motors[i & 0x03u]; }

uint8_t CAN_yaw_motor_online(void)
{
    return (uint8_t)((yaw_motor.rx_tick != 0u) &&
                     ((uint32_t)(HAL_GetTick() - yaw_motor.rx_tick) <= INFANTRY_MOTOR_TIMEOUT_MS));
}

const volatile can_shooter_debug_t *CAN_shooter_get_debug(void) { return &shooter_debug; }

uint8_t CAN_shooter_online_mask(void)
{
    uint32_t now = HAL_GetTick();
    uint8_t mask = 0u;
    if ((trigger_motor.rx_tick != 0u) &&
        ((uint32_t)(now - trigger_motor.rx_tick) <= INFANTRY_MOTOR_TIMEOUT_MS)) mask |= 1u;
    if ((fric_left_motor.rx_tick != 0u) &&
        ((uint32_t)(now - fric_left_motor.rx_tick) <= INFANTRY_MOTOR_TIMEOUT_MS)) mask |= 2u;
    if ((fric_right_motor.rx_tick != 0u) &&
        ((uint32_t)(now - fric_right_motor.rx_tick) <= INFANTRY_MOTOR_TIMEOUT_MS)) mask |= 4u;
    return mask;
}

uint8_t CAN_shooter_motors_online(void)
{
    return (uint8_t)(CAN_shooter_online_mask() == 7u);
}
