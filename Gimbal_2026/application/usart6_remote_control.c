/**
  ****************************(C) COPYRIGHT 2024****************************
  * @file       usart1_remote_control.c
  * @brief      串口1接收转发的遥控器数据处理
  * @note       该模块用于接收通过其他串口转发的SBUS数据，并进行解码
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-18-2024     Assistant       1. 初始版本
  ****************************(C) COPYRIGHT 2024****************************
  */

   /*
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     2026-4-12       zxy             串口收发任务
  *   这里有串口一和底盘通信，发送云台陀螺仪Yaw角，接收底盘旋转角速度
  *   串口六和图传通信，接收图传数据，例如鼠标键盘
  *   板子上丝印是错的串口一实际是串口六，串口二实际是串口一具体看C板说明手册
  ****************************(C) COPYRIGHT 2024****************************        
  */
#include "usart6_remote_control.h"
#include "bsp_usart.h"
#include "string.h"
#include <stdarg.h>
#include <stdio.h>
#include "bsp_led.h" // Include the header file where BSP_LED_SHOW is declared
#include "usbd_cdc_if.h"
  
extern UART_HandleTypeDef huart6; // 声明外部UART句柄
extern DMA_HandleTypeDef hdma_usart1_tx;

float chassis_rad = 0.0f; // 全局变量存储从底盘接收的角度值

/* Main-loop-only staging, separate from the persistent interrupt TX buffer. */
static char debug_pending[1536]; /* Include the shooter speed/current diagnostic line. */
static uint16_t debug_pending_len;
static uint8_t debug_active[1536];

void usart6_debug_printf(const char *format, ...)
{
    va_list args;
    int length;
    uint16_t available = (uint16_t)(sizeof(debug_pending) - debug_pending_len);
    if ((format == NULL) || (available <= 1u)) return;
    va_start(args, format);
    length = vsnprintf(&debug_pending[debug_pending_len], available, format, args);
    va_end(args);
    if (length > 0)
        debug_pending_len += (uint16_t)((length < available) ? length : available - 1u);
}

void usart6_debug_poll(void)
{
    if ((debug_pending_len == 0u) || (huart6.gState != HAL_UART_STATE_READY)) return;
    memcpy(debug_active, debug_pending, debug_pending_len);
    if (HAL_UART_Transmit_IT(&huart6, debug_active, debug_pending_len) == HAL_OK)
        debug_pending_len = 0u;
}

/*
 * 底盘状态帧缓存（由USART1接收中断写入，控制环读取）：
 * - 字段定义与 usart1_chassis_status_t 保持一致。
 * - update_tick 用于上层判断数据新鲜度（避免串口丢包时继续用旧值）。
 */
static volatile usart1_chassis_status_t g_usart1_chassis_status = {0};
static volatile usart1_chassis_motion_t g_chassis_motion = {0};
static volatile usart1_chassis_imu_t g_chassis_imu = {0};
static volatile uint32_t g_usart1_chassis_status_update_tick = 0u;

/*
 * USART1流解析器内部状态（支持任意len输入，如20/40/60字节）：
 * - parse_buf: 正在拼接的候选帧缓存。
 * - parse_idx: 当前已缓存字节数。
 *
 * 说明：
 * - 该状态在多次调用 usart1_parse_chassis_status_stream() 之间保留，
 *   用于处理“半帧跨IDLE边界”的场景。
 */
static uint8_t g_usart1_status_parse_buf[USART1_FRAME_LENGTH] = {0};
static uint16_t g_usart1_status_parse_idx = 0u;

// 串口1接收缓冲（双缓冲模式）
static uint8_t usart1_rx_buf[2][USART1_RX_BUF_NUM];

// 串口1解码后的遥控器数据
static RC_ctrl_t usart1_rc_ctrl;

// 临时SBUS数据缓冲（用于提取SBUS原始数据）
static uint8_t sbus_temp_buf[18];

// 取正函数
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
  
// SBUS解码函数（复制自 remote_control.c）
static void sbus_to_rc_local(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl)
{
    if (sbus_buf == NULL || rc_ctrl == NULL)
    {
        return;
    }

    rc_ctrl->rc.ch[0] = (sbus_buf[0] | (sbus_buf[1] << 8)) & 0x07ff;
    rc_ctrl->rc.ch[1] = ((sbus_buf[1] >> 3) | (sbus_buf[2] << 5)) & 0x07ff;
    rc_ctrl->rc.ch[2] = ((sbus_buf[2] >> 6) | (sbus_buf[3] << 2) | 
                        (sbus_buf[4] << 10)) & 0x07ff;
    rc_ctrl->rc.ch[3] = ((sbus_buf[4] >> 1) | (sbus_buf[5] << 7)) & 0x07ff;
    rc_ctrl->rc.s[0] = ((sbus_buf[5] >> 4) & 0x0003);
    rc_ctrl->rc.s[1] = ((sbus_buf[5] >> 4) & 0x000C) >> 2;
    rc_ctrl->mouse.x = sbus_buf[6] | (sbus_buf[7] << 8);
    rc_ctrl->mouse.y = sbus_buf[8] | (sbus_buf[9] << 8);
    rc_ctrl->mouse.z = sbus_buf[10] | (sbus_buf[11] << 8);
    rc_ctrl->mouse.press_l = sbus_buf[12];
    rc_ctrl->mouse.press_r = sbus_buf[13];
    rc_ctrl->key.v = sbus_buf[14] | (sbus_buf[15] << 8);
    rc_ctrl->rc.ch[4] = sbus_buf[16] | (sbus_buf[17] << 8);

    rc_ctrl->rc.ch[0] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[1] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[2] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[3] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[4] -= RC_CH_VALUE_OFFSET;
}
  
/**
* @brief          校验接收到的数据帧
* @param[in]      buf: 接收到的数据缓冲区
* @param[in]      len: 数据长度
* @retval         0: 校验成功, 1: 校验失败
*/
static uint8_t usart1_data_check(uint8_t *buf, uint16_t len)
{
    if (len != USART1_FRAME_LENGTH)
        return 1;
    
    // 检查帧头
    if (buf[0] != 0xA6)
        return 1;
    
    // 校验和检查
    uint8_t checksum = 0;
    for (int i = 0; i < 19; i++)
    {
        checksum += buf[i];
    }
    
    if (checksum != buf[19])
        return 1;
    
    return 0;
}

/**
* @brief          从转发帧中提取SBUS数据
* @param[in]      rx_buf: 接收缓冲区
* @param[out]     sbus_buf: SBUS数据输出缓冲区
* @retval         0: 成功, 1: 失败
*/
static uint8_t extract_sbus_from_frame(uint8_t *rx_buf, uint8_t *sbus_buf)
{
    if (rx_buf[0] != 0xA6)
        return 1;
    
    // 提取18字节的SBUS数据
    memcpy(sbus_buf, rx_buf + 1, 18);
    return 0;
}

/**
* @brief          串口1遥控器初始化
* @param[in]      none
* @retval         none
*/
void usart1_remote_control_init(void)
{
    // 初始化DMA双缓冲接收
//    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
//    HAL_UART_Receive_DMA(&huart1, usart1_rx_buf[0], USART1_RX_BUF_NUM);
    usart1_tx_dma_init();
    // 初始化遥控器数据结构
    memset(&usart1_rc_ctrl, 0, sizeof(RC_ctrl_t));

    /* 初始化USART1底盘状态帧缓存，避免上电误用历史值。 */
    memset((void *)&g_usart1_chassis_status, 0, sizeof(g_usart1_chassis_status));
    memset((void *)&g_chassis_imu, 0, sizeof(g_chassis_imu));
    g_usart1_chassis_status_update_tick = 0u;

    /* 重置流解析器状态，避免上电或复位后残留半帧。 */
    memset(g_usart1_status_parse_buf, 0, sizeof(g_usart1_status_parse_buf));
    g_usart1_status_parse_idx = 0u;
    // BSP_LED_Show(0xFF00ffff);
}

/**
* @brief          获取串口1的遥控器数据指针
* @param[in]      none
* @retval         遥控器数据指针
*/
const RC_ctrl_t *get_usart1_remote_control_point(void)
{
    return &usart1_rc_ctrl;
}

/**
* @brief          判断串口1遥控器数据是否出错
* @param[in]      none
* @retval         0: 正常, 1: 出错
*/
uint8_t usart1_RC_data_is_error(void)
{
    // 使用了go to语句 方便出错统一处理遥控器变量数据归零
    if (RC_abs(usart1_rc_ctrl.rc.ch[0]) > RC_CHANNAL_ERROR_VALUE)
    {
        goto error;
    }
    if (RC_abs(usart1_rc_ctrl.rc.ch[1]) > RC_CHANNAL_ERROR_VALUE)
    {
        goto error;
    }
    if (RC_abs(usart1_rc_ctrl.rc.ch[2]) > RC_CHANNAL_ERROR_VALUE)
    {
        goto error;
    }
    if (RC_abs(usart1_rc_ctrl.rc.ch[3]) > RC_CHANNAL_ERROR_VALUE)
    {
        goto error;
    }
    if (usart1_rc_ctrl.rc.s[0] == 0)
    {
        goto error;
    }
    if (usart1_rc_ctrl.rc.s[1] == 0)
    {
        goto error;
    }
    return 0;

error:
    usart1_rc_ctrl.rc.ch[0] = 0;
    usart1_rc_ctrl.rc.ch[1] = 0;
    usart1_rc_ctrl.rc.ch[2] = 0;
    usart1_rc_ctrl.rc.ch[3] = 0;
    usart1_rc_ctrl.rc.ch[4] = 0;
    usart1_rc_ctrl.rc.s[0] = RC_SW_DOWN;
    usart1_rc_ctrl.rc.s[1] = RC_SW_DOWN;
    usart1_rc_ctrl.mouse.x = 0;
    usart1_rc_ctrl.mouse.y = 0;
    usart1_rc_ctrl.mouse.z = 0;
    usart1_rc_ctrl.mouse.press_l = 0;
    usart1_rc_ctrl.mouse.press_r = 0;
    usart1_rc_ctrl.key.v = 0;
    return 1;
}

/**
* @brief          重启串口1接收
* @param[in]      none
* @retval         none
*/
void usart1_slove_RC_lost(void)
{
    // 停止DMA
    __HAL_DMA_DISABLE(&hdma_usart1_rx);
    
    // 重新设置DMA
    hdma_usart1_rx.Instance->NDTR = USART1_RX_BUF_NUM;
    
    // 重新启动接收
    if ((hdma_usart1_rx.Instance->CR & DMA_SxCR_CT) == RESET)
    {
        hdma_usart1_rx.Instance->CR |= DMA_SxCR_CT;
    }
    else
    {
        hdma_usart1_rx.Instance->CR &= ~(DMA_SxCR_CT);
    }
    
    __HAL_DMA_ENABLE(&hdma_usart1_rx);
}

void usart1_slove_data_error(void)
{
    usart1_slove_RC_lost();
}

float get_rad_from_chassis(uint8_t* data) 
{
    float rad;
    if(data == NULL) {
        return 0.0f;
    }
    memcpy(&rad, data, sizeof(float));
    chassis_rad = rad; // 将角度值存储到全局变量中，供其他模块使用
    return rad;
}

uint8_t usart1_parse_chassis_status_frame(const uint8_t *frame, uint16_t len)
{
    uint8_t checksum = 0u;
    uint16_t i;
    float chassis_speed_rpm = 0.0f;
    uint16_t shooter_heat = 0u;
    uint16_t shooter_heat_limit = 0u;
    uint16_t shooter_cooling_value = 0u;
    int16_t remote_ch2;
    int16_t remote_ch3;

    if ((frame == NULL) || (len != USART1_FRAME_LENGTH))
    {
        return 0u;
    }

    if ((frame[0] != USART1_FRAME_HEADER_WIT) &&
        (frame[0] != INFANTRY_FRAME_CHASSIS_MOTION) &&
        (frame[0] != INFANTRY_FRAME_CHASSIS_IMU))
    {
        return 0u;
    }

    for (i = 0u; i < (USART1_FRAME_LENGTH - 1u); i++)
    {
        checksum = (uint8_t)(checksum + frame[i]);
    }
    if (checksum != frame[USART1_FRAME_LENGTH - 1u])
    {
        return 0u;
    }

    if (frame[0] == INFANTRY_FRAME_CHASSIS_IMU)
    {
        int16_t angles_cdeg[3];
        float yaw_rate_rpm;
        uint32_t source_tick;
        memcpy(angles_cdeg, &frame[4], sizeof(angles_cdeg));
        memcpy(&yaw_rate_rpm, &frame[10], sizeof(yaw_rate_rpm));
        memcpy(&source_tick, &frame[14], sizeof(source_tick));
        g_chassis_imu.ready = frame[1];
        g_chassis_imu.error = frame[2];
        g_chassis_imu.sequence = frame[3];
        g_chassis_imu.yaw_angle_deg = (float)angles_cdeg[0] * 0.01f;
        g_chassis_imu.pitch_angle_deg = (float)angles_cdeg[1] * 0.01f;
        g_chassis_imu.roll_angle_deg = (float)angles_cdeg[2] * 0.01f;
        g_chassis_imu.yaw_rate_rpm = yaw_rate_rpm;
        g_chassis_imu.source_tick = source_tick;
        g_chassis_imu.update_tick = HAL_GetTick();
        /* Receiving telemetry must not keep a stale A5 safety status alive. */
        return 1u;
    }

    if (frame[0] == INFANTRY_FRAME_CHASSIS_MOTION)
    {
        int16_t relative_cdeg;
        memcpy(&relative_cdeg, &frame[1], sizeof(relative_cdeg));
        g_chassis_motion.relative_yaw_deg = (float)relative_cdeg * 0.01f;
        memcpy((void *)g_chassis_motion.target_rpm, &frame[3], 8u);
        memcpy((void *)g_chassis_motion.actual_rpm, &frame[11], 8u);
        g_chassis_motion.update_tick = HAL_GetTick();
        /* Diagnostics must not refresh safety status or change its flags. */
        return 1u;
    }

    memcpy(&chassis_speed_rpm, &frame[4], sizeof(float));

    shooter_heat = (uint16_t)(frame[8] | ((uint16_t)frame[9] << 8));
    shooter_heat_limit = (uint16_t)(frame[10] | ((uint16_t)frame[11] << 8));
    shooter_cooling_value = (uint16_t)(frame[12] | ((uint16_t)frame[13] << 8));

    g_usart1_chassis_status.flags = frame[1];
    g_usart1_chassis_status.fault = frame[2];
    g_usart1_chassis_status.sequence = frame[3];
    g_usart1_chassis_status.chassis_gyro_speed_rpm = chassis_speed_rpm;
    g_usart1_chassis_status.shooter_17mm_heat = shooter_heat;
    g_usart1_chassis_status.shooter_17mm_heat_limit = shooter_heat_limit;
    g_usart1_chassis_status.shooter_barrel_cooling_value = shooter_cooling_value;
    memcpy(&remote_ch2, &frame[14], sizeof(remote_ch2));
    memcpy(&remote_ch3, &frame[16], sizeof(remote_ch3));
    g_usart1_chassis_status.remote_ch2 = remote_ch2;
    g_usart1_chassis_status.remote_ch3 = remote_ch3;
    g_usart1_chassis_status.remote_left_switch = frame[18];
    g_usart1_chassis_status_update_tick = HAL_GetTick();

    /* 旧接口 chassis_rad 保留给 legacy 数据链路，不在此处覆盖。 */

    return 1u;
}

uint16_t usart1_parse_chassis_status_stream(const uint8_t *data, uint16_t len)
{
    uint16_t i;
    uint16_t parsed_count = 0u;

    if ((data == NULL) || (len == 0u))
    {
        return 0u;
    }

    for (i = 0u; i < len; i++)
    {
        uint8_t byte = data[i];

        if (g_usart1_status_parse_idx == 0u)
        {
            /*
             * 流同步起点：仅在帧头0xA5处开始拼帧。
             * 这样即使IDLE一次收到40/60字节，也能在流中自动对齐到正确边界。
             */
            if ((byte == USART1_FRAME_HEADER_WIT) ||
                (byte == INFANTRY_FRAME_CHASSIS_MOTION) ||
                (byte == INFANTRY_FRAME_CHASSIS_IMU))
            {
                g_usart1_status_parse_buf[0] = byte;
                g_usart1_status_parse_idx = 1u;
            }
            continue;
        }

        if (g_usart1_status_parse_idx < USART1_FRAME_LENGTH)
        {
            g_usart1_status_parse_buf[g_usart1_status_parse_idx] = byte;
            g_usart1_status_parse_idx++;
        }

        if (g_usart1_status_parse_idx >= USART1_FRAME_LENGTH)
        {
            uint8_t frame_ok = usart1_parse_chassis_status_frame(g_usart1_status_parse_buf, USART1_FRAME_LENGTH);

            if (frame_ok != 0u)
            {
                parsed_count++;
                g_usart1_status_parse_idx = 0u;
            }
            else
            {
                uint16_t resync_pos = USART1_FRAME_LENGTH;
                uint16_t j;

                /*
                 * 校验失败时做“滑窗重同步”：
                 * 在当前20字节窗口内寻找下一个0xA5，避免整包丢弃导致长期错位。
                 */
                for (j = 1u; j < USART1_FRAME_LENGTH; j++)
                {
                    if ((g_usart1_status_parse_buf[j] == USART1_FRAME_HEADER_WIT) ||
                        (g_usart1_status_parse_buf[j] == INFANTRY_FRAME_CHASSIS_MOTION) ||
                        (g_usart1_status_parse_buf[j] == INFANTRY_FRAME_CHASSIS_IMU))
                    {
                        resync_pos = j;
                        break;
                    }
                }

                if (resync_pos < USART1_FRAME_LENGTH)
                {
                    uint16_t remain = (uint16_t)(USART1_FRAME_LENGTH - resync_pos);
                    memmove(g_usart1_status_parse_buf,
                            &g_usart1_status_parse_buf[resync_pos],
                            remain);
                    g_usart1_status_parse_idx = remain;
                }
                else
                {
                    g_usart1_status_parse_idx = 0u;
                }
            }
        }
    }

    return parsed_count;
}

void usart1_reset_chassis_status_stream_parser(void)
{
    /*
     * 复位流解析状态：
     * - 在串口错误/重启接收时调用，避免残留半帧影响后续校验。
     */
    memset(g_usart1_status_parse_buf, 0, sizeof(g_usart1_status_parse_buf));
    g_usart1_status_parse_idx = 0u;
}

float get_usart1_chassis_gyro_speed_rpm(void)
{
    return g_usart1_chassis_status.chassis_gyro_speed_rpm;
}

uint8_t get_usart1_small_gyro_enable(void)
{
    return (uint8_t)((g_usart1_chassis_status.flags & INFANTRY_CHASSIS_FLAG_SMALL_GYRO) != 0u);
}

uint8_t get_usart1_chassis_imu_ready(void)
{
    return (uint8_t)((g_usart1_chassis_status.flags & INFANTRY_CHASSIS_FLAG_IMU_READY) != 0u);
}

uint8_t get_usart1_chassis_enabled(void)
{
    return (uint8_t)((g_usart1_chassis_status.flags & INFANTRY_CHASSIS_FLAG_ENABLED) != 0u);
}

uint16_t get_usart1_shooter_17mm_heat(void)
{
    return g_usart1_chassis_status.shooter_17mm_heat;
}

uint16_t get_usart1_shooter_17mm_heat_limit(void)
{
    return g_usart1_chassis_status.shooter_17mm_heat_limit;
}

uint16_t get_usart1_shooter_barrel_cooling_value(void)
{
    return g_usart1_chassis_status.shooter_barrel_cooling_value;
}

uint32_t get_usart1_chassis_status_update_tick(void)
{
    return g_usart1_chassis_status_update_tick;
}

const volatile usart1_chassis_imu_t *get_usart1_chassis_imu(void)
{
    return &g_chassis_imu;
}

const volatile usart1_chassis_motion_t *get_usart1_chassis_motion(void)
{
    return &g_chassis_motion;
}

const volatile usart1_chassis_status_t *get_usart1_chassis_status(void)
{
    return &g_usart1_chassis_status;
}

void usart1_send_gimbal_status(const usart1_gimbal_tx_status_t *status)
{
    static uint8_t tx_buffer[USART1_FRAME_LENGTH];
    static uint32_t last_tx_tick;
    uint32_t now = HAL_GetTick();
    uint8_t i;
    uint32_t irq_mask;

    if (status == NULL) return;
    if ((uint32_t)(now - last_tx_tick) < INFANTRY_STATUS_PERIOD_MS) return;

    remote_control_forward_poll(); /* Pending RC always precedes periodic status. */
    irq_mask = __get_PRIMASK();
    __disable_irq();
    if (((hdma_usart1_tx.Instance->CR & DMA_SxCR_EN) != 0u) ||
        ((USART1->SR & USART_SR_TC) == 0u))
    {
        __set_PRIMASK(irq_mask);
        return;
    }

    memset(tx_buffer, 0, sizeof(tx_buffer));
    tx_buffer[0] = USART1_FRAME_HEADER_KEYBOARD;
    tx_buffer[1] = status->flags;
    tx_buffer[2] = status->fault;
    tx_buffer[3] = status->sequence;
    memcpy(&tx_buffer[4], &status->yaw_angle_deg, sizeof(float));
    memcpy(&tx_buffer[8], &status->yaw_speed_rpm, sizeof(float));
    memcpy(&tx_buffer[12], &status->yaw_encoder, sizeof(status->yaw_encoder));
    for (i = 0u; i < (USART1_FRAME_LENGTH - 1u); i++)
    {
        tx_buffer[USART1_FRAME_LENGTH - 1u] += tx_buffer[i];
    }
    usart1_tx_dma_enable(tx_buffer, sizeof(tx_buffer));
    last_tx_tick = now;
    __set_PRIMASK(irq_mask);
}




/* 全局变量 */
uint8_t usart1_rx_buffer[USART1_FRAME_LENGTH];  // 接收缓冲区
uint8_t usart1_rx_complete = 0;                  // 接收完成标志

/* 在初始化完成后启动一次接收 */
void USART1_Start_Receive(void)
{
    // 启动中断接收，每次接收 USART1_FRAME_LENGTH 字节
    HAL_UART_Receive_IT(&huart1, usart1_rx_buffer, USART1_FRAME_LENGTH);
}

/* ================= USART6 图传链路接收 ================= */
#define TUCHUAN_CRC16_INIT 0xFFFFu

#define TUCHUAN_VTM_HEADER_1   0xA9u
#define TUCHUAN_VTM_HEADER_2   0x53u
#define TUCHUAN_VTM_FRAME_LEN  21u
#define TUCHUAN_VTM_SHORT_FRAME_LEN 20u
#define TUCHUAN_RX_CHUNK_SIZE  1u

#define TUCHUAN_DEBUG_RAW_BUF_SIZE 1024u

static volatile uint16_t usart6_debug_head = 0u;
static volatile uint16_t usart6_debug_tail = 0u;
static uint8_t usart6_debug_raw_buf[TUCHUAN_DEBUG_RAW_BUF_SIZE];
static uint8_t usart6_debug_enable_flag = 1u;

static void usart6_debug_push_raw_byte(uint8_t byte)
{
    uint16_t next_head = (uint16_t)(usart6_debug_head + 1u);
    if (next_head >= TUCHUAN_DEBUG_RAW_BUF_SIZE)
    {
        next_head = 0u;
    }

    if (next_head != usart6_debug_tail)
    {
        usart6_debug_raw_buf[usart6_debug_head] = byte;
        usart6_debug_head = next_head;
    }
}

uint8_t usart6_tuchuan_rx_buffer[TUCHUAN_RX_CHUNK_SIZE];
static uint8_t usart6_vtm_frame_buf[TUCHUAN_VTM_FRAME_LEN];
static uint8_t usart6_vtm_frame_index = 0u;
static usart6_tuchuan_remote_control_t usart6_tuchuan_rc_data;
static usart6_tuchuan_custom_data_t usart6_tuchuan_custom_data;
static uint16_t usart6_tuchuan_last_cmd_id = 0;
static usart6_tuchuan_debug_stat_t usart6_tuchuan_debug_stat;

static uint16_t usart6_crc16_calculate(const uint8_t *message, uint16_t length, uint16_t crc)
{
    uint16_t i;
    uint8_t bit;

    if (message == NULL)
    {
        return crc;
    }

    for (i = 0; i < length; i++)
    {
        crc ^= message[i];
        for (bit = 0; bit < 8; bit++)
        {
            if (crc & 0x0001u)
            {
                crc = (uint16_t)((crc >> 1) ^ 0x8408u);
            }
            else
            {
                crc >>= 1;
            }
        }
    }

    return crc;
}

static uint16_t usart6_crc16_ccitt_false_calculate(const uint8_t *message, uint16_t length, uint16_t crc)
{
    uint16_t i;
    uint8_t bit;

    if (message == NULL)
    {
        return crc;
    }

    for (i = 0; i < length; i++)
    {
        crc ^= (uint16_t)(message[i] << 8);
        for (bit = 0; bit < 8; bit++)
        {
            if (crc & 0x8000u)
            {
                crc = (uint16_t)((crc << 1) ^ 0x1021u);
            }
            else
            {
                crc <<= 1;
            }
        }
    }

    return crc;
}

static uint16_t usart6_read_u16_le(const uint8_t *data)
{
    return (uint16_t)(data[0] | ((uint16_t)data[1] << 8));
}

static int16_t usart6_read_i16_le(const uint8_t *data)
{
    return (int16_t)usart6_read_u16_le(data);
}

static uint8_t usart6_verify_vtm_frame_crc(const uint8_t *frame)
{
    uint16_t crc_tail_le;
    uint16_t crc_reflected;
    uint16_t crc_ccitt;

    if (frame == NULL)
    {
        return 0u;
    }

    crc_tail_le = usart6_read_u16_le(frame + (TUCHUAN_VTM_FRAME_LEN - 2u));

    crc_reflected = usart6_crc16_calculate(frame, (uint16_t)(TUCHUAN_VTM_FRAME_LEN - 2u), TUCHUAN_CRC16_INIT);
    if (crc_tail_le == crc_reflected)
    {
        return 1u;
    }

    crc_ccitt = usart6_crc16_ccitt_false_calculate(frame, (uint16_t)(TUCHUAN_VTM_FRAME_LEN - 2u), TUCHUAN_CRC16_INIT);
    if ((crc_tail_le == crc_ccitt) || (crc_tail_le == (uint16_t)((crc_ccitt >> 8) | (crc_ccitt << 8))))
    {
        return 1u;
    }

    return 0u;
}

static void usart6_decode_vtm_frame(const uint8_t *frame);

static uint8_t usart6_verify_vtm_short_frame_crc(const uint8_t *frame)
{
    uint8_t frame_expand[TUCHUAN_VTM_FRAME_LEN];

    if (frame == NULL)
    {
        return 0u;
    }

    memcpy(frame_expand, frame, (TUCHUAN_VTM_SHORT_FRAME_LEN - 2u));
    frame_expand[TUCHUAN_VTM_FRAME_LEN - 3u] = 0u;
    frame_expand[TUCHUAN_VTM_FRAME_LEN - 2u] = frame[TUCHUAN_VTM_SHORT_FRAME_LEN - 2u];
    frame_expand[TUCHUAN_VTM_FRAME_LEN - 1u] = frame[TUCHUAN_VTM_SHORT_FRAME_LEN - 1u];

    return usart6_verify_vtm_frame_crc(frame_expand);
}

static void usart6_decode_vtm_short_frame(const uint8_t *frame)
{
    uint8_t frame_expand[TUCHUAN_VTM_FRAME_LEN];

    if (frame == NULL)
    {
        return;
    }

    memcpy(frame_expand, frame, (TUCHUAN_VTM_SHORT_FRAME_LEN - 2u));
    frame_expand[TUCHUAN_VTM_FRAME_LEN - 3u] = 0u;
    frame_expand[TUCHUAN_VTM_FRAME_LEN - 2u] = frame[TUCHUAN_VTM_SHORT_FRAME_LEN - 2u];
    frame_expand[TUCHUAN_VTM_FRAME_LEN - 1u] = frame[TUCHUAN_VTM_SHORT_FRAME_LEN - 1u];

    usart6_decode_vtm_frame(frame_expand);
}

static void usart6_decode_vtm_frame(const uint8_t *frame)
{
    const uint8_t *p;

    if (frame == NULL)
    {
        return;
    }

    p = frame + 2u;

    usart6_tuchuan_rc_data.channel_0 = (uint16_t)(((uint16_t)p[0] | ((uint16_t)p[1] << 8)) & 0x07FFu);
    usart6_tuchuan_rc_data.channel_1 = (uint16_t)((((uint16_t)p[1] >> 3) | ((uint16_t)p[2] << 5)) & 0x07FFu);
    usart6_tuchuan_rc_data.channel_2 = (uint16_t)((((uint16_t)p[2] >> 6) | ((uint16_t)p[3] << 2) | ((uint16_t)p[4] << 10)) & 0x07FFu);
    usart6_tuchuan_rc_data.channel_3 = (uint16_t)((((uint16_t)p[4] >> 1) | ((uint16_t)p[5] << 7)) & 0x07FFu);

    usart6_tuchuan_rc_data.mode_switch = (uint8_t)((p[5] >> 4) & 0x03u);
    usart6_tuchuan_rc_data.pause_button = (uint8_t)((p[5] >> 6) & 0x01u);
    usart6_tuchuan_rc_data.custom_button_l = (uint8_t)((p[5] >> 7) & 0x01u);
    usart6_tuchuan_rc_data.custom_button_r = (uint8_t)((p[6] >> 0) & 0x01u);

    usart6_tuchuan_rc_data.dial = (uint16_t)((((uint16_t)p[6] >> 1) | ((uint16_t)p[7] << 7)) & 0x07FFu);
    usart6_tuchuan_rc_data.trigger_down = (uint8_t)((p[7] >> 4) & 0x01u);

    usart6_tuchuan_rc_data.mouse_x = usart6_read_i16_le(p + 8u);
    usart6_tuchuan_rc_data.mouse_y = usart6_read_i16_le(p + 10u);
    usart6_tuchuan_rc_data.mouse_z = usart6_read_i16_le(p + 12u);

    usart6_tuchuan_rc_data.left_button_down = (uint8_t)((p[14] >> 0) & 0x03u);
    usart6_tuchuan_rc_data.right_button_down = (uint8_t)((p[14] >> 2) & 0x03u);
    usart6_tuchuan_rc_data.middle_button_down = (uint8_t)((p[14] >> 4) & 0x03u);

    usart6_tuchuan_rc_data.keyboard_value = usart6_read_u16_le(p + 15u);
    usart6_tuchuan_rc_data.reserved = 0u;
    usart6_tuchuan_rc_data.update_tick = HAL_GetTick();
    usart6_tuchuan_rc_data.updated = 1u;

    usart6_tuchuan_last_cmd_id = 0xA953u;

    memset(usart6_tuchuan_custom_data.data, 0, sizeof(usart6_tuchuan_custom_data.data));
    memcpy(usart6_tuchuan_custom_data.data, frame, TUCHUAN_VTM_FRAME_LEN);
    usart6_tuchuan_custom_data.data_len = TUCHUAN_VTM_FRAME_LEN;
    usart6_tuchuan_custom_data.update_tick = usart6_tuchuan_rc_data.update_tick;
    usart6_tuchuan_custom_data.updated = 1u;
}

static void usart6_tuchuan_unpack_byte(uint8_t byte)
{
    if (usart6_vtm_frame_index == 0u)
    {
        if (byte == TUCHUAN_VTM_HEADER_1)
        {
            usart6_tuchuan_debug_stat.hdr1_cnt++;
            usart6_vtm_frame_buf[0] = byte;
            usart6_vtm_frame_index = 1u;
        }
        return;
    }

    if (usart6_vtm_frame_index == 1u)
    {
        if (byte == TUCHUAN_VTM_HEADER_2)
        {
            usart6_tuchuan_debug_stat.hdr2_cnt++;
            usart6_vtm_frame_buf[1] = byte;
            usart6_vtm_frame_index = 2u;
        }
        else if (byte == TUCHUAN_VTM_HEADER_1)
        {
            usart6_tuchuan_debug_stat.hdr1_cnt++;
            usart6_vtm_frame_buf[0] = byte;
            usart6_vtm_frame_index = 1u;
        }
        else
        {
            usart6_vtm_frame_index = 0u;
        }
        return;
    }

    usart6_vtm_frame_buf[usart6_vtm_frame_index++] = byte;

    if (usart6_vtm_frame_index < TUCHUAN_VTM_FRAME_LEN)
    {
        return;
    }

    if (usart6_verify_vtm_frame_crc(usart6_vtm_frame_buf))
    {
        usart6_tuchuan_debug_stat.frame_ok_cnt++;
        usart6_decode_vtm_frame(usart6_vtm_frame_buf);
    }
    else if ((byte == TUCHUAN_VTM_HEADER_1) && usart6_verify_vtm_short_frame_crc(usart6_vtm_frame_buf))
    {
        usart6_tuchuan_debug_stat.frame_short_ok_cnt++;
        usart6_decode_vtm_short_frame(usart6_vtm_frame_buf);
    }
    else
    {
        usart6_tuchuan_debug_stat.frame_fail_cnt++;
    }

    if (byte == TUCHUAN_VTM_HEADER_1)
    {
        usart6_tuchuan_debug_stat.hdr1_cnt++;
        usart6_vtm_frame_buf[0] = byte;
        usart6_vtm_frame_index = 1u;
    }
    else
    {
        usart6_vtm_frame_index = 0u;
    }
}
uint8_t flag=0;
uint8_t usart6_rx_buffer[21];
int cnt=0;
// 该函数为简化的字节流解析示例，实际使用中建议使用状态机方式进行更健壮的帧同步和错误处理。
void usart6_tuchuan_byte(uint8_t byte)
{
    if(flag==0)
    {
        if(byte==0xA9)
        {
            flag=1;
        }
    }
    else if(flag==1)
    {
        if(byte==0x53)
        {
            flag=2;
        }
        else if(byte==0xA9)
        {
            flag=1;
        }
        else
        {
            flag=0;
        }
    }
    else if(flag==2)
    {
        usart6_rx_buffer[cnt++]=byte;
    }
    if(cnt==21)
    {
        cnt=0;
        flag=0;
    }
}

void usb_debug_push_byte(void)
{
    // snprintf();
    USB_Transmit(usart6_rx_buffer, 21);
}
void usart6_tuchuan_init(void)
{
    memset(usart6_vtm_frame_buf, 0, sizeof(usart6_vtm_frame_buf));
    usart6_vtm_frame_index = 0u;

    memset(&usart6_tuchuan_rc_data, 0, sizeof(usart6_tuchuan_rc_data));
    memset(&usart6_tuchuan_custom_data, 0, sizeof(usart6_tuchuan_custom_data));
    memset(&usart6_tuchuan_debug_stat, 0, sizeof(usart6_tuchuan_debug_stat));
    usart6_tuchuan_last_cmd_id = 0u;

    if (HAL_UART_Receive_IT(&huart6, usart6_tuchuan_rx_buffer, TUCHUAN_RX_CHUNK_SIZE) == HAL_OK)
    {
        usart6_tuchuan_debug_stat.rx_start_ok_cnt++;
    }
    else
    {
        usart6_tuchuan_debug_stat.rx_start_fail_cnt++;
    }
}


void usart6_tuchuan_error_callback(uint32_t error_code)
{
    usart6_tuchuan_debug_stat.uart_err_cnt++;
    usart6_tuchuan_debug_stat.uart_last_err_code = error_code;
    __HAL_UART_CLEAR_PEFLAG(&huart6);

    if (HAL_UART_Receive_IT(&huart6, usart6_tuchuan_rx_buffer, TUCHUAN_RX_CHUNK_SIZE) == HAL_OK)
    {
        usart6_tuchuan_debug_stat.rx_start_ok_cnt++;
    }
    else
    {
        usart6_tuchuan_debug_stat.rx_start_fail_cnt++;
    }
}
void usart6_tuchuan_rx_callback(void)
{
    usart6_tuchuan_debug_stat.rx_irq_cnt++;

    if (HAL_UART_Receive_IT(&huart6, usart6_tuchuan_rx_buffer, TUCHUAN_RX_CHUNK_SIZE) == HAL_OK)
    {
        usart6_tuchuan_debug_stat.rx_start_ok_cnt++;
    }
    else
    {
        usart6_tuchuan_debug_stat.rx_start_fail_cnt++;
    }

    usart6_debug_push_raw_byte(usart6_tuchuan_rx_buffer[0]);
    usart6_tuchuan_unpack_byte(usart6_tuchuan_rx_buffer[0]);
}
const usart6_tuchuan_remote_control_t *get_usart6_tuchuan_remote_control_point(void)
{
    return &usart6_tuchuan_rc_data;
}

const usart6_tuchuan_custom_data_t *get_usart6_tuchuan_custom_data_point(void)
{
    return &usart6_tuchuan_custom_data;
}

uint16_t get_usart6_tuchuan_last_cmd_id(void)
{
    return usart6_tuchuan_last_cmd_id;
}

uint16_t get_usart6_tuchuan_keyboard_value(void)
{
    return usart6_tuchuan_rc_data.keyboard_value;
}
void clear_usart6_tuchuan_remote_control_update_flag(void)
{
    usart6_tuchuan_rc_data.updated = 0u;
}

void clear_usart6_tuchuan_custom_data_update_flag(void)
{
    usart6_tuchuan_custom_data.updated = 0u;
}
const usart6_tuchuan_debug_stat_t *get_usart6_tuchuan_debug_stat_point(void)
{
    return &usart6_tuchuan_debug_stat;
}

void usart6_tuchuan_debug_enable(uint8_t enable)
{
    usart6_debug_enable_flag = (uint8_t)(enable ? 1u : 0u);
}

void usart6_tuchuan_usb_debug_flush(void)
{
    uint8_t local_buf[24];
    uint16_t cnt = 0u;
    uint16_t i;
    char usb_line[96];
    uint16_t pos = 0u;
    static const char hex_table[] = "0123456789ABCDEF";

    if (usart6_debug_enable_flag == 0u)
    {
        return;
    }

    while (cnt < (uint16_t)sizeof(local_buf))
    {
        uint8_t has_data = 0u;

        __disable_irq();
        if (usart6_debug_head != usart6_debug_tail)
        {
            local_buf[cnt] = usart6_debug_raw_buf[usart6_debug_tail];
            usart6_debug_tail++;
            if (usart6_debug_tail >= TUCHUAN_DEBUG_RAW_BUF_SIZE)
            {
                usart6_debug_tail = 0u;
            }
            has_data = 1u;
        }
        __enable_irq();

        if (has_data == 0u)
        {
            break;
        }

        cnt++;
    }

    if (cnt == 0u)
    {
        return;
    }

    usb_line[pos++] = 'R';
    usb_line[pos++] = 'X';
    usb_line[pos++] = ':';
    usb_line[pos++] = ' ';

    for (i = 0u; i < cnt; i++)
    {
        usb_line[pos++] = hex_table[(local_buf[i] >> 4) & 0x0Fu];
        usb_line[pos++] = hex_table[local_buf[i] & 0x0Fu];
        if (i + 1u < cnt)
        {
            usb_line[pos++] = ' ';
        }
    }

    usb_line[pos++] = '\r';
    usb_line[pos++] = '\n';

    USB_Transmit((uint8_t *)usb_line, pos);
}






























