/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       can_receive.c/h
  * @brief      there is CAN interrupt function  to receive motor data,
  *             and CAN send function to send motor current to control motor.
  *             这里是CAN中断接收函数，接收电机数据,CAN发送函数发送电机电流控制电机.
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.1.0     Nov-11-2019     RM              1. support hal lib
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "CAN_receive.h"

//#include "cmsis_os.h"

#include "main.h"
#include "infantry_config.h"
//#include "bsp_rng.h"


//#include "detect_task.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
//motor data read
#define get_motor_measure(ptr, data)                                    \
    {                                                                   \
        (ptr)->last_ecd = (ptr)->ecd;                                   \
        (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);            \
        (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);      \
        (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);  \
        (ptr)->temperate = (data)[6];                                   \
    }
/*
motor data,  0:chassis motor1 3508;1:chassis motor3 3508;2:chassis motor3 3508;3:chassis motor4 3508;
4:yaw gimbal motor 6020;5:pitch gimbal motor 6020;6:trigger motor 2006;
电机数据, 0:底盘电机1 3508电机,  1:底盘电机2 3508电机,2:底盘电机3 3508电机,3:底盘电机4 3508电机;
4:yaw云台电机 6020电机; 5:pitch云台电机 6020电机; 6:拨弹电机 2006电机*/
static motor_measure_t motor_chassis[9];
static volatile uint32_t chassis_rx_tick[4];

static CAN_TxHeaderTypeDef  gimbal_tx_message;
static uint8_t              gimbal_can_send_data[8];
static CAN_TxHeaderTypeDef  chassis_tx_message;
static uint8_t              chassis_can_send_data[8];
static CAN_TxHeaderTypeDef  fric_tx_message;
static uint8_t              fric_can_send_data[8];

/**
  * @brief          hal CAN fifo call back, receive motor data
  * @param[in]      hcan, the point to CAN handle
  * @retval         none
  */
/**
  * @brief          hal库CAN回调函数,接收电机数据
  * @param[in]      hcan:CAN句柄指针
  * @retval         none
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data) != HAL_OK) return;
    if ((hcan != &CHASSIS_CAN) || (rx_header.IDE != CAN_ID_STD) ||
        (rx_header.RTR != CAN_RTR_DATA) || (rx_header.DLC != 8u)) return;

    switch (rx_header.StdId)
    {
        case CAN_3508_M1_ID:
        case CAN_3508_M2_ID:
        case CAN_3508_M3_ID:
        case CAN_3508_M4_ID:
        case CAN_YAW_MOTOR_ID:
        case CAN_PIT_MOTOR_ID:
        case CAN_TRIGGER_MOTOR_ID:
        case CAN_FRIC_LEFT_ID:
        case CAN_FRIC_RIGHT_ID:
        {
            static uint8_t i = 0;
            //get motor id
            i = rx_header.StdId - CAN_3508_M1_ID;
            get_motor_measure(&motor_chassis[i], rx_data);
            if (i < 4u) chassis_rx_tick[i] = HAL_GetTick();

//            detect_hook(CHASSIS_MOTOR1_TOE + i);   //frertos钩子函数
            break;
        }

        default:
        {
            break;
        }
    }
}



/**
  * @brief          send control current of motor (0x205, 0x206, 0x207, 0x208)
  * @param[in]      yaw: (0x205) 6020 motor control current, range [-30000,30000] 
  * @param[in]      pitch: (0x206) 6020 motor control current, range [-30000,30000]
  * @param[in]      shoot: (0x207) 2006 motor control current, range [-10000,10000]
  * @param[in]      rev: (0x208) reserve motor control current
  * @retval         none
  */
/**
  * @brief          发送电机控制电流(0x205,0x206,0x207,0x208)
  * @param[in]      yaw: (0x205) 6020电机控制电流, 范围 [-30000,30000]
  * @param[in]      pitch: (0x206) 6020电机控制电流, 范围 [-30000,30000]
  * @param[in]      shoot: (0x207) 2006电机控制电流, 范围 [-10000,10000]
  * @param[in]      rev: (0x208) 保留，电机控制电流
  * @retval         none
  */
void CAN_cmd_gimbal(int16_t yaw, int16_t pitch, int16_t shoot, int16_t rev)
{
    uint32_t send_mail_box;
    gimbal_tx_message.StdId = CAN_GIMBAL_ALL_ID;
    gimbal_tx_message.IDE = CAN_ID_STD;
    gimbal_tx_message.RTR = CAN_RTR_DATA;
    gimbal_tx_message.DLC = 0x08;
    gimbal_can_send_data[0] = (yaw >> 8);
    gimbal_can_send_data[1] = yaw;
    gimbal_can_send_data[2] = (pitch >> 8);
    gimbal_can_send_data[3] = pitch;
    gimbal_can_send_data[4] = (shoot >> 8);
    gimbal_can_send_data[5] = shoot;
    gimbal_can_send_data[6] = (rev >> 8);
    gimbal_can_send_data[7] = rev;
    HAL_CAN_AddTxMessage(&GIMBAL_CAN, &gimbal_tx_message, gimbal_can_send_data, &send_mail_box);
}

/**
 * @brief ? ? ? ? ?发送电机控制电流(0x209,0x20A,0x20B,0x20C)
 * @param[in] ? ? ?fric_left: (0x208) 3508电机控制电流, 范围 [-16384,16384]   左边负的是正转
 * @param[in] ? ? ?fric_right: (0x209) 3508电机控制电流, 范围 [-16384,16384]  右边正的是正转
 * @param[in] ? ? ?rev1: 保留，电机控制电流
 * @param[in] ? ? ?rev2: 保留，电机控制电流
 * @retval ? ? ? ? none
 */
void CAN_cmd_fric_wheel(int16_t fric_left, int16_t fric_right, int16_t rev1, int16_t rev2)
{
    uint32_t send_mail_box;
    /*
      // 使用 CAN_GIMBAL_ALL_ID = 0x1FF 对应的 CAN2 来发送，但控制的是 0x208 到 0x20B 的电机
      // 假设您希望使用 CAN 消息 ID 0x1FF 来控制 0x208, 0x209, 0x20A, 0x20B 四个电机。
      // 注意：DJI CAN协议中，0x1FF 发送给 ID 5-8 (即 0x205-0x208)。
      // 0x2FF 发送给 ID 9-12 (即 0x209-0x20C)。
      // 为了与您新增的 ID 0x208, 0x209 对应，我们需要一个新的发送 ID (例如 0x2FF)
      // 或者将摩擦轮控制逻辑放到 CAN_cmd_gimbal 的保留位 rev/rev2 中（如果 rev 对应 0x208）。
      // 基于您原有的 CAN_cmd_gimbal 发送给 CAN_GIMBAL_ALL_ID (0x1FF) 来控制 0x205-0x208 的电机，
      // 我将添加一个发送 ID 为 0x2FF 的函数来控制 0x209, 0x20A, 0x20B, 0x20C。
      // 但是您的摩擦轮 ID 是 0x208 和 0x209。
      // **最简便的兼容方式是修改 CAN_cmd_gimbal 的第四个参数 rev 对应 0x208，并新增一个 CAN 发送函数来控制 0x209 到 0x20C。**
      // 
      // **为了避免破坏原有的 CAN_cmd_gimbal 逻辑，我将新增一个发送 ID 为 0x2FF 的命令来控制 0x209-0x20C。**
      // 
      // 由于您只需要控制 0x208 和 0x209 两个电机，我将把它们分成两个 CAN 发送命令来处理：
      // 1. `CAN_cmd_gimbal` 的 `rev` 参数控制 `0x208` (左摩擦轮)
      // 2. 新增 `CAN_cmd_fric_wheel` 使用新的 CAN ID (`0x2FF`) 来控制 `0x209` (右摩擦轮) 和其他保留位。
      // 
      // **选择方案：**
      // 
      // 1. **修改 `CAN_cmd_gimbal`**：将 `rev` 参数视为控制 `0x208`（左摩擦轮），并将 `CAN_cmd_fric_wheel` 控制 `0x209` (`CAN_FRIC_RIGHT_ID`) 等电机。
      // 
      // 2. **新增 `CAN_cmd_fric_wheel` (发送 ID 0x2FF)**：控制 `0x209, 0x20A, 0x20B, 0x20C`。但这样 `0x208` (左摩擦轮) 就没有发送函数了。
      // 
      // **采纳方案：** 将 `CAN_cmd_gimbal` 中的 `rev` 明确用于控制 `0x208`。并新增 `CAN_cmd_fric_wheel` 用于控制 `0x209` 和其他电机。

      // 为了实现 0x208 和 0x209 的控制，我假设您新增了一个 CAN ID `CAN_FRIC_WHEEL_ALL_ID = 0x2FF` 来发送给 0x209, 0x20A, 0x20B, 0x20C
      // 并在 can_receive.h 的 can_msg_id_e 中添加：
      // CAN_FRIC_WHEEL_ALL_ID = 0x2FF,

      // 重新审视：
      // CAN_GIMBAL_ALL_ID = 0x1FF 控制 0x205, 0x206, 0x207, 0x208。
      // 因此，您新增的 0x208 (CAN_FRIC_LEFT_ID) 应该由 CAN_cmd_gimbal 的第四个参数 (`rev`) 控制。
      // 
      // 摩擦轮右电机 0x209 (CAN_FRIC_RIGHT_ID) 应该由新的 CAN 消息 ID (0x2FF) 控制的第一位。
      
      // **我们将 `CAN_cmd_gimbal` 的 `rev` 参数视为控制 `CAN_FRIC_LEFT_ID (0x208)`。**

      // **新增 `CAN_cmd_fric_wheel` 函数，用于控制 `0x209, 0x20A, 0x20B, 0x20C`。**
      
      // 确保 `can_receive.h` 中有 `CAN_FRIC_WHEEL_ALL_ID = 0x2FF` 这一行（我会在最终代码中添加）。
    */
    
    gimbal_tx_message.StdId = CAN_CHASSIS_ALL_ID; 
    gimbal_tx_message.IDE = CAN_ID_STD;
    gimbal_tx_message.RTR = CAN_RTR_DATA;
    gimbal_tx_message.DLC = 0x08;
    gimbal_can_send_data[0] = (fric_left >> 8);   
    gimbal_can_send_data[1] = fric_left;
    gimbal_can_send_data[2] = (fric_right >> 8); 
    gimbal_can_send_data[3] = fric_right;
    gimbal_can_send_data[4] = (rev1 >> 8); 
    gimbal_can_send_data[5] = rev1;
    gimbal_can_send_data[6] = (rev2 >> 8); 
    gimbal_can_send_data[7] = rev2;
    // 使用 CAN2 来发送摩擦轮控制电流
    HAL_CAN_AddTxMessage(&GIMBAL_CAN, &gimbal_tx_message, gimbal_can_send_data, &send_mail_box);
}

/**
  * @brief          send CAN packet of ID 0x700, it will set chassis motor 3508 to quick ID setting
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          发送ID为0x700的CAN包,它会设置3508电机进入快速设置ID
  * @param[in]      none
  * @retval         none
  */
void CAN_cmd_chassis_reset_ID(void)
{
    uint32_t send_mail_box;
    chassis_tx_message.StdId = 0x700;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
    chassis_can_send_data[0] = 0;
    chassis_can_send_data[1] = 0;
    chassis_can_send_data[2] = 0;
    chassis_can_send_data[3] = 0;
    chassis_can_send_data[4] = 0;
    chassis_can_send_data[5] = 0;
    chassis_can_send_data[6] = 0;
    chassis_can_send_data[7] = 0;

    HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}


/**
  * @brief          send control current of motor (0x201, 0x202, 0x203, 0x204)
  * @param[in]      motor1: (0x201) 3508 motor control current, range [-16384,16384] 
  * @param[in]      motor2: (0x202) 3508 motor control current, range [-16384,16384] 
  * @param[in]      motor3: (0x203) 3508 motor control current, range [-16384,16384] 
  * @param[in]      motor4: (0x204) 3508 motor control current, range [-16384,16384] 
  * @retval         none
  */
/**
  * @brief          发送电机控制电流(0x201,0x202,0x203,0x204)
  * @param[in]      motor1: (0x201) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor2: (0x202) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor3: (0x203) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor4: (0x204) 3508电机控制电流, 范围 [-16384,16384]
  * @retval         none
  */
void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    uint32_t send_mail_box;
    chassis_tx_message.StdId = CAN_CHASSIS_ALL_ID;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
    chassis_can_send_data[0] = motor1 >> 8;
    chassis_can_send_data[1] = motor1;
    chassis_can_send_data[2] = motor2 >> 8;
    chassis_can_send_data[3] = motor2;
    chassis_can_send_data[4] = motor3 >> 8;
    chassis_can_send_data[5] = motor3;
    chassis_can_send_data[6] = motor4 >> 8;
    chassis_can_send_data[7] = motor4;

    HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}

/**
  * @brief          return the yaw 6020 motor data point
  * @param[in]      none
  * @retval         motor data point
  */
/**
  * @brief          返回yaw 6020电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
const motor_measure_t *get_yaw_gimbal_motor_measure_point(void)
{
    return &motor_chassis[4];
}

/**
  * @brief          return the pitch 6020 motor data point
  * @param[in]      none
  * @retval         motor data point
  */
/**
  * @brief          返回pitch 6020电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
const motor_measure_t *get_pitch_gimbal_motor_measure_point(void)
{
    return &motor_chassis[5];
}

/**
  * @brief          return the trigger 2006 motor data point
  * @param[in]      none
  * @retval         motor data point
  */
/**
  * @brief          返回拨弹电机 2006电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
const motor_measure_t *get_trigger_motor_measure_point(void)
{
    return &motor_chassis[6];
}

/**
 * @brief ? ? ? ? ?返回左摩擦轮 3508电机数据指针
 * @param[in] ? ? ?none
 * @retval ? ? ? ? 电机数据指针
 */
const motor_measure_t *get_fric_wheel_left_motor_measure_point(void)
{
  return &motor_chassis[0]; 
}

/**
 * @brief ? ? ? ? ?return the right friction wheel 3508 motor data point
 * @param[in] ? ? ?none
 * @retval ? ? ? ? motor data point
 */
/**
 * @brief ? ? ? ? ?返回右摩擦轮 3508电机数据指针
 * @param[in] ? ? ?none
 * @retval ? ? ? ? 电机数据指针
 */
const motor_measure_t *get_fric_wheel_right_motor_measure_point(void)
{
  return &motor_chassis[1]; 
}

/**
  * @brief          return the chassis 3508 motor data point
  * @param[in]      i: motor number,range [0,3]
  * @retval         motor data point
  */
/**
  * @brief          返回底盘电机 3508电机数据指针
  * @param[in]      i: 电机编号,范围[0,3]
  * @retval         电机数据指针
  */
const motor_measure_t *get_chassis_motor_measure_point(uint8_t i)
{
    return &motor_chassis[(i & 0x03)];
}

uint8_t CAN_chassis_motors_online(void)
{
    uint8_t i;
    uint32_t now = HAL_GetTick();
    for (i = 0u; i < 4u; i++)
        if ((chassis_rx_tick[i] == 0u) ||
            ((uint32_t)(now - chassis_rx_tick[i]) > INFANTRY_MOTOR_TIMEOUT_MS)) return 0u;
    return 1u;
}
