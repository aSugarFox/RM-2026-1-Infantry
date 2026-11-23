/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       remote_control.c/h
  * @brief      遥控器处理，遥控器是通过类似SBUS的协议传输，利用DMA传输方式节约CPU
  *             资源，利用串口空闲中断来拉起处理函数，同时提供一些掉线重启DMA，串口
  *             的方式保证热插拔的稳定性。
  * @note       该任务是通过串口中断启动，不是freeRTOS任务
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.0.0     Nov-11-2019     RM              1. support development board tpye c
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "remote_control.h"

#include "main.h"
#include "infantry_config.h"
#include "bsp_buzzer.h"
#include "bsp_usart.h"
#include "string.h"
#include "WIT_Gyro.h"
//#include "detect_task.h"



//遥控器出错数据上限
#define RC_CHANNAL_ERROR_VALUE 700

extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart6;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;

#define USART1_FRAME_HEADER_WIT       0xA5u
#define USART1_FRAME_HEADER_RC        0xA6u
#define USART1_FRAME_HEADER_KEYBOARD  0xA7u

extern uint16_t get_usart6_tuchuan_keyboard_value(void);
extern float get_tuchuan_gimbal_gyro_comp_speed(void);
extern uint8_t get_tuchuan_gimbal_gyro_enabled(void);

static void forward_to_usart1_by_mode(uint8_t *sbus);
static uint8_t usart1_tx_busy(void);
float angle[3];
char buffer[64];

//取正函数
static int16_t RC_abs(int16_t value);

/**
  * @brief          遥控器协议解析
  * @param[in]      sbus_buf: 原生数据指针
  * @param[out]     rc_ctrl: 遥控器数据指
  * @retval         none
  */
static void sbus_to_rc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl);

//remote control data 
//遥控器控制变量
RC_ctrl_t rc_ctrl;
static volatile uint32_t rc_update_tick;
static uint8_t remote_tx_latest[RC_FRAME_LENGTH];
static volatile uint8_t remote_tx_pending;
static uint32_t remote_tx_tick;
//接收原始数据，为18个字节，给了36个字节长度，防止DMA传输越界
static uint8_t sbus_rx_buf[2][SBUS_RX_BUF_NUM];



/**
  * @brief          遥控器初始化
  * @param[in]      none
  * @retval         none
  */
void remote_control_init(void)
{
    rc_update_tick = 0u;
    RC_Init(sbus_rx_buf[0], sbus_rx_buf[1], SBUS_RX_BUF_NUM);
}

/**
  * @brief          获取遥控器数据指针
  * @param[in]      none
  * @retval         遥控器数据指针
  */
const RC_ctrl_t *get_remote_control_point(void)
{
    return &rc_ctrl;
}

uint32_t get_remote_control_update_tick(void)
{
    return rc_update_tick;
}

//判断遥控器数据是否出错，
uint8_t RC_data_is_error(void)
{
    //使用了go to语句 方便出错统一处理遥控器变量数据归零
    if (RC_abs(rc_ctrl.rc.ch[0]) > RC_CHANNAL_ERROR_VALUE)
    {
        goto error;
    }
    if (RC_abs(rc_ctrl.rc.ch[1]) > RC_CHANNAL_ERROR_VALUE)
    {
        goto error;
    }
    if (RC_abs(rc_ctrl.rc.ch[2]) > RC_CHANNAL_ERROR_VALUE)
    {
        goto error;
    }
    if (RC_abs(rc_ctrl.rc.ch[3]) > RC_CHANNAL_ERROR_VALUE)
    {
        goto error;
    }
    if (rc_ctrl.rc.s[0] == 0)
    {
        goto error;
    }
    if (rc_ctrl.rc.s[1] == 0)
    {
        goto error;
    }
    return 0;

error:
    rc_ctrl.rc.ch[0] = 0;
    rc_ctrl.rc.ch[1] = 0;
    rc_ctrl.rc.ch[2] = 0;
    rc_ctrl.rc.ch[3] = 0;
    rc_ctrl.rc.ch[4] = 0;
    rc_ctrl.rc.s[0] = RC_SW_DOWN;
    rc_ctrl.rc.s[1] = RC_SW_DOWN;
    rc_ctrl.mouse.x = 0;
    rc_ctrl.mouse.y = 0;
    rc_ctrl.mouse.z = 0;
    rc_ctrl.mouse.press_l = 0;
    rc_ctrl.mouse.press_r = 0;
    rc_ctrl.key.v = 0;
    return 1;
}

void slove_RC_lost(void)
{
    RC_restart(SBUS_RX_BUF_NUM);
}
void slove_data_error(void)
{
    RC_restart(SBUS_RX_BUF_NUM);
}
float Wit_Ecd_angle(float alpha)
{
    (void)alpha;
    WIT_IIC_ReadAngle(angle);
    return angle[2];
}
//串口中断
void USART3_IRQHandler(void)
{
    if(huart3.Instance->SR & UART_FLAG_RXNE)//接收到数据
    {
        __HAL_UART_CLEAR_PEFLAG(&huart3);
    }
    else if(USART3->SR & UART_FLAG_IDLE)
    {
        static uint16_t this_time_rx_len = 0;

        __HAL_UART_CLEAR_PEFLAG(&huart3);

        if ((hdma_usart3_rx.Instance->CR & DMA_SxCR_CT) == RESET)
        {
            /* Current memory buffer used is Memory 0 */

            //disable DMA
            //失效DMA
            __HAL_DMA_DISABLE(&hdma_usart3_rx);

            //get receive data length, length = set_data_length - remain_length
            //获取接收数据长度,长度 = 设定长度 - 剩余长度
            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;

            //reset set_data_lenght
            //重新设定数据长度
            hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

            //set memory buffer 1
            //设定缓冲区1
            hdma_usart3_rx.Instance->CR |= DMA_SxCR_CT;
            
            //enable DMA
            //使能DMA
            __HAL_DMA_ENABLE(&hdma_usart3_rx);

            if(this_time_rx_len == RC_FRAME_LENGTH)
            {
                sbus_to_rc(sbus_rx_buf[0], &rc_ctrl);
                rc_update_tick = HAL_GetTick();
                //记录数据接收时间
//                detect_hook(DBUS_TOE);
                forward_to_usart1_by_mode(sbus_rx_buf[0]);                
            }
        }
        else
        {
            /* Current memory buffer used is Memory 1 */
            //disable DMA
            //失效DMA
            __HAL_DMA_DISABLE(&hdma_usart3_rx);

            //get receive data length, length = set_data_length - remain_length
            //获取接收数据长度,长度 = 设定长度 - 剩余长度
            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;

            //reset set_data_lenght
            //重新设定数据长度
            hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

            //set memory buffer 0
            //设定缓冲区0
            DMA1_Stream1->CR &= ~(DMA_SxCR_CT);
            
            //enable DMA
            //使能DMA
            __HAL_DMA_ENABLE(&hdma_usart3_rx);

            if(this_time_rx_len == RC_FRAME_LENGTH)
            {
                //处理遥控器数据
                sbus_to_rc(sbus_rx_buf[1], &rc_ctrl);
                rc_update_tick = HAL_GetTick();
                //记录数据接收时间
//                detect_hook(DBUS_TOE);
                forward_to_usart1_by_mode(sbus_rx_buf[1]);
            }
        }
    }
}

//取正函数
static int16_t RC_abs(int16_t value)
{
    if (value > 0)
    {
        return value;
    }
    else
    {
        return -value;
    }
}
/**
  * @brief          遥控器协议解析
  * @param[in]      sbus_buf: 原生数据指针
  * @param[out]     rc_ctrl: 遥控器数据指
  * @retval         none
  */
static void sbus_to_rc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl)
{
    if (sbus_buf == NULL || rc_ctrl == NULL)
    {
        return;
    }

    rc_ctrl->rc.ch[0] = (sbus_buf[0] | (sbus_buf[1] << 8)) & 0x07ff;        //!< Channel 0
    rc_ctrl->rc.ch[1] = ((sbus_buf[1] >> 3) | (sbus_buf[2] << 5)) & 0x07ff; //!< Channel 1
    rc_ctrl->rc.ch[2] = ((sbus_buf[2] >> 6) | (sbus_buf[3] << 2) |          //!< Channel 2
                         (sbus_buf[4] << 10)) &0x07ff;
    rc_ctrl->rc.ch[3] = ((sbus_buf[4] >> 1) | (sbus_buf[5] << 7)) & 0x07ff; //!< Channel 3
    rc_ctrl->rc.s[0] = ((sbus_buf[5] >> 4) & 0x0003);                  //!< Switch left
    rc_ctrl->rc.s[1] = ((sbus_buf[5] >> 4) & 0x000C) >> 2;                       //!< Switch right
    rc_ctrl->mouse.x = sbus_buf[6] | (sbus_buf[7] << 8);                    //!< Mouse X axis
    rc_ctrl->mouse.y = sbus_buf[8] | (sbus_buf[9] << 8);                    //!< Mouse Y axis
    rc_ctrl->mouse.z = sbus_buf[10] | (sbus_buf[11] << 8);                  //!< Mouse Z axis
    rc_ctrl->mouse.press_l = sbus_buf[12];                                  //!< Mouse Left Is Press ?
    rc_ctrl->mouse.press_r = sbus_buf[13];                                  //!< Mouse Right Is Press ?
    rc_ctrl->key.v = sbus_buf[14] | (sbus_buf[15] << 8);                    //!< KeyBoard value
    rc_ctrl->rc.ch[4] = sbus_buf[16] | (sbus_buf[17] << 8);                 //NULL

    rc_ctrl->rc.ch[0] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[1] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[2] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[3] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[4] -= RC_CH_VALUE_OFFSET;
}

/**
  * @brief          send sbus data by usart1, called in usart3_IRQHandle
  * @param[in]      sbus: sbus data, 18 bytes
  * @retval         none
  */
/**
  * @brief          通过usart1发送sbus数据,在usart3_IRQHandle调用
  * @param[in]      sbus: sbus数据, 18字节
  * @retval         none
  */
static uint8_t usart1_tx_busy(void)
{
    if (hdma_usart1_tx.Instance->NDTR != 0u)
    {
        return 1u;
    }

    if ((USART1->SR & USART_SR_TC) == 0u)
    {
        return 1u;
    }

    return 0u;
}

void remote_control_forward_poll(void)
{
    uint32_t irq_mask = __get_PRIMASK();
    __disable_irq();
    if ((remote_tx_pending != 0u) &&
        ((uint32_t)(HAL_GetTick() - remote_tx_tick) > INFANTRY_LINK_TIMEOUT_MS))
        remote_tx_pending = 0u; /* Never refresh chassis watchdog with old RC data. */
    if ((remote_tx_pending != 0u) && (usart1_tx_busy() == 0u))
    {
        sbus_to_usart1(remote_tx_latest);
        remote_tx_pending = 0u;
    }
    __set_PRIMASK(irq_mask);
}

static void forward_to_usart1_by_mode(uint8_t *sbus)
{
    if (sbus == NULL) return;
    memcpy(remote_tx_latest, sbus, RC_FRAME_LENGTH);
    remote_tx_tick = HAL_GetTick();
    remote_tx_pending = 1u;
    remote_control_forward_poll();
}
// USART1 0xA7帧: [1..2]=键盘值(uint16_t)，其他字节为0，校验位为前19字节之和
void keyboard_to_usart1(uint16_t keyboard_value)
{
    static uint8_t usart_tx_buf[20];
    static uint8_t i;

    memset(usart_tx_buf, 0, sizeof(usart_tx_buf));
    usart_tx_buf[0] = USART1_FRAME_HEADER_KEYBOARD;
    usart_tx_buf[1] = (uint8_t)(keyboard_value & 0x00FFu);
    usart_tx_buf[2] = (uint8_t)((keyboard_value >> 8) & 0x00FFu);

    usart_tx_buf[19] = 0u;
    for (i = 0; i < 19; i++)
    {
        usart_tx_buf[19] += usart_tx_buf[i];
    }

    usart1_tx_dma_enable(usart_tx_buf, 20);
}
// USART1 0xA6帧: [1..18]=原始SBUS数据(18字节)
void sbus_to_usart1(uint8_t *sbus)
{
    static uint8_t usart_tx_buf[20];
    static uint8_t i =0;
    usart_tx_buf[0] = USART1_FRAME_HEADER_RC;
    memcpy(usart_tx_buf + 1, sbus, 18);
    for(i = 0, usart_tx_buf[19] = 0; i < 19; i++)
    {
        usart_tx_buf[19] += usart_tx_buf[i];
    }
    
    usart1_tx_dma_enable(usart_tx_buf, 20);
}

/* USART1 0xA5帧: [1..4]=陀螺仪角度(float), [5..8]=云台Yaw电机转速RPM(float), [9]=小陀螺使能 */
void data_to_usart1(float data)
{
    static uint8_t usart_tx_buf[20];
    static uint8_t i;
    float yaw_speed_rpm = get_tuchuan_gimbal_gyro_comp_speed();
    uint8_t gyro_enable = get_tuchuan_gimbal_gyro_enabled();

    memset(usart_tx_buf, 0, sizeof(usart_tx_buf));
    usart_tx_buf[0] = USART1_FRAME_HEADER_WIT;
    memcpy(&usart_tx_buf[1], &data, sizeof(float));
    memcpy(&usart_tx_buf[5], &yaw_speed_rpm, sizeof(float));
    usart_tx_buf[9] = gyro_enable;

    usart_tx_buf[19] = 0;
    for(i = 0; i < 19; i++)
    {
        usart_tx_buf[19] += usart_tx_buf[i];
    }

    usart1_tx_dma_enable(usart_tx_buf, 20);
}
/*@brief          USART1非中断发送任务，定期发送键盘数据和陀螺仪数据，陀螺仪数据供底盘运动解算使用，键盘数据供底盘键盘控制使用
  * @param[in]      none
  * @retval         none
  */
void usart1_non_irq_tx_task(void)
{
    static uint32_t last_keyboard_tx_tick = 0u;
    static uint32_t last_gyro_tx_tick = 0u;
    static uint8_t alternate_pick_keyboard = 0u;
    uint32_t now_tick = HAL_GetTick();
    uint8_t keyboard_mode_on = (uint8_t)((rc_ctrl.rc.s[RC_S_LEFT] == RC_SW_DOWN) ? 1u : 0u);
    uint8_t keyboard_due = 0u;
    uint8_t gyro_due = 0u;
    uint8_t send_keyboard = 0u;
    uint8_t send_gyro = 0u;

    if ((uint32_t)(now_tick - last_gyro_tx_tick) >= 10u)
    {
        gyro_due = 1u;
    }

    if ((keyboard_mode_on != 0u) && ((uint32_t)(now_tick - last_keyboard_tx_tick) >= 20u))
    {
        keyboard_due = 1u;
    }

    if ((keyboard_due != 0u) && (gyro_due != 0u))
    {
        /* 两路都到发送周期时，严格交替，防止键盘帧被陀螺帧长期挤占 */
        if (alternate_pick_keyboard != 0u)
        {
            send_keyboard = 1u;
        }
        else
        {
            send_gyro = 1u;
        }
    }
    else if (keyboard_due != 0u)
    {
        send_keyboard = 1u;
    }
    else if (gyro_due != 0u)
    {
        send_gyro = 1u;
    }

    if (send_gyro != 0u)
    {
        //读取陀螺仪数据，供给底盘运动解算使用
        WIT_IIC_ReadAngle(angle);
    }

    __disable_irq();
    if (usart1_tx_busy() == 0u)
    {
        if (send_keyboard != 0u)
        {
            //串口给底盘发送键盘数据，底盘根据这个数据进行键盘控制
            keyboard_to_usart1(get_usart6_tuchuan_keyboard_value());
            last_keyboard_tx_tick = now_tick;
            alternate_pick_keyboard = 0u;
        }
        else if (send_gyro != 0u)
        {
            //串口给底盘发送陀螺仪数据，底盘根据这个数据进行陀螺仪补偿
            data_to_usart1(angle[2]);
            last_gyro_tx_tick = now_tick;
            alternate_pick_keyboard = 1u;
        }
    }
    __enable_irq();
}











