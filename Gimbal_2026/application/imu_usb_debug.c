   /*
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     2026-4-12       zxy             Usb给视觉发送陀螺仪数据任务
  *  这里是通过USB串口向视觉系统发送陀螺仪数据，同时监控陀螺仪的状态和温度，并进行温度控制。
  ****************************(C) COPYRIGHT 2024****************************        
  */
#include "imu_usb_debug.h"

#include "main.h"
#include "i2c.h"
#include "bsp_delay.h"
#include "bsp_imu_pwm.h"
#include "pid.h"
#include "BMI088driver.h"
#include "BMI088reg.h"
#include "BMI088Middleware.h"
#include "ist8310driver.h"
#include "ist8310driver_middleware.h"
#include "usbd_cdc_if.h"

#include <stdio.h>
#include <string.h>

#define IMU_USB_DEBUG_TX_PERIOD_MS        50u
#define IMU_USB_DEBUG_INIT_RETRY_MS       300u
#define IMU_USB_DEBUG_TEMP_PID_KP         1600.0f
#define IMU_USB_DEBUG_TEMP_PID_KI         0.2f
#define IMU_USB_DEBUG_TEMP_PID_KD         0.0f
#define IMU_USB_DEBUG_TEMP_PID_MAX_OUT    4500.0f
#define IMU_USB_DEBUG_TEMP_PID_MAX_IOUT   4400.0f
#define IMU_USB_DEBUG_TEMP_PWM_MAX        5000u
#define IMU_USB_DEBUG_TEMP_TARGET_C       45.0f

typedef struct
{
    uint32_t last_tx_tick;
    uint32_t last_init_tick;
    uint8_t bmi_ready;
    uint8_t ist_ready;
    uint8_t bmi_error;
    uint8_t bmi_accel_init_error;
    uint8_t bmi_gyro_init_error;
    uint8_t bmi_accel_chip_id;
    uint8_t bmi_gyro_chip_id;
    uint8_t bmi_accel_pwr_ctrl_readback;
    uint8_t bmi_accel_pwr_conf_readback;
    uint8_t bmi_accel_conf_readback;
    uint8_t bmi_accel_range_readback;
    uint8_t ist_error;
    uint8_t ist_chip_id;
    uint8_t temp_pid_inited;
    uint8_t temp_first_reached;
    uint8_t temp_constant_time;
    fp32 gyro[3];
    fp32 accel[3];
    fp32 mag[3];
    fp32 temp;
    pid_type_def temp_pid;
    char tx_buffer[256];
} imu_usb_debug_ctx_t;

static imu_usb_debug_ctx_t g_imu_usb_debug;
static const fp32 g_imu_usb_debug_temp_pid_param[3] =
{
    IMU_USB_DEBUG_TEMP_PID_KP,
    IMU_USB_DEBUG_TEMP_PID_KI,
    IMU_USB_DEBUG_TEMP_PID_KD
};

static uint8_t imu_usb_debug_bmi088_read_accel_chip_id_once(void);
static uint8_t imu_usb_debug_bmi088_read_accel_chip_id(void);
static void imu_usb_debug_bmi088_accel_write_single_reg(uint8_t reg, uint8_t value);
static uint8_t imu_usb_debug_bmi088_accel_read_single_reg(uint8_t reg);
static uint8_t imu_usb_debug_bmi088_read_gyro_chip_id(void);
static uint8_t imu_usb_debug_bmi088_accel_init(imu_usb_debug_ctx_t *ctx);
static uint8_t imu_usb_debug_ist8310_read_single_reg(uint8_t reg);
static void imu_usb_debug_ist8310_write_single_reg(uint8_t reg, uint8_t value);
static uint8_t imu_usb_debug_ist8310_init(void);
static void imu_usb_debug_ist8310_read_mag(fp32 mag[3]);
static void imu_usb_debug_try_init(imu_usb_debug_ctx_t *ctx);
static void imu_usb_debug_temp_control(imu_usb_debug_ctx_t *ctx, fp32 temp);
static void imu_usb_debug_send_frame(imu_usb_debug_ctx_t *ctx);

void imu_usb_debug_init(void)
{
    memset(&g_imu_usb_debug, 0, sizeof(g_imu_usb_debug));

    g_imu_usb_debug.bmi_error = 0xFFu;
    g_imu_usb_debug.bmi_accel_init_error = 0xFFu;
    g_imu_usb_debug.bmi_gyro_init_error = 0xFFu;
    g_imu_usb_debug.bmi_accel_chip_id = 0xFFu;
    g_imu_usb_debug.bmi_gyro_chip_id = 0xFFu;
    g_imu_usb_debug.bmi_accel_pwr_ctrl_readback = 0xFFu;
    g_imu_usb_debug.bmi_accel_pwr_conf_readback = 0xFFu;
    g_imu_usb_debug.bmi_accel_conf_readback = 0xFFu;
    g_imu_usb_debug.bmi_accel_range_readback = 0xFFu;
    g_imu_usb_debug.ist_error = 0xFFu;
    g_imu_usb_debug.ist_chip_id = 0xFFu;

    imu_pwm_set(0u);
}

void imu_usb_debug_poll(void)
{
    imu_usb_debug_ctx_t *ctx = &g_imu_usb_debug;
    uint32_t now = HAL_GetTick();

    if (((ctx->bmi_ready == 0u) || (ctx->ist_ready == 0u)) &&
        ((now - ctx->last_init_tick) >= IMU_USB_DEBUG_INIT_RETRY_MS))
    {
        imu_usb_debug_try_init(ctx);
        ctx->last_init_tick = now;
    }

    if ((now - ctx->last_tx_tick) < IMU_USB_DEBUG_TX_PERIOD_MS)
    {
        return;
    }

    if (ctx->bmi_ready != 0u)
    {
        BMI088_read(ctx->gyro, ctx->accel, &ctx->temp);
        imu_usb_debug_temp_control(ctx, ctx->temp);

        if (ctx->ist_ready != 0u)
        {
            imu_usb_debug_ist8310_read_mag(ctx->mag);
        }
        else
        {
            ctx->mag[0] = 0.0f;
            ctx->mag[1] = 0.0f;
            ctx->mag[2] = 0.0f;
        }
    }

    imu_usb_debug_send_frame(ctx);
    ctx->last_tx_tick = now;
}

static uint8_t imu_usb_debug_bmi088_read_accel_chip_id_once(void)
{
    uint8_t chip_id = 0xFFu;

    BMI088_GPIO_init();
    BMI088_com_init();
    BMI088_ACCEL_NS_L();
    BMI088_read_write_byte(BMI088_ACC_CHIP_ID | 0x80u);
    BMI088_read_write_byte(0x55u);
    chip_id = BMI088_read_write_byte(0x55u);
    BMI088_ACCEL_NS_H();
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    return chip_id;
}

static uint8_t imu_usb_debug_bmi088_read_accel_chip_id(void)
{
    (void)imu_usb_debug_bmi088_read_accel_chip_id_once();
    return imu_usb_debug_bmi088_read_accel_chip_id_once();
}

static void imu_usb_debug_bmi088_accel_write_single_reg(uint8_t reg, uint8_t value)
{
    BMI088_ACCEL_NS_L();
    BMI088_read_write_byte(reg);
    BMI088_read_write_byte(value);
    BMI088_ACCEL_NS_H();
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
}

static uint8_t imu_usb_debug_bmi088_accel_read_single_reg(uint8_t reg)
{
    uint8_t value = 0xFFu;

    BMI088_ACCEL_NS_L();
    BMI088_read_write_byte(reg | 0x80u);
    BMI088_read_write_byte(0x55u);
    value = BMI088_read_write_byte(0x55u);
    BMI088_ACCEL_NS_H();
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    return value;
}

static uint8_t imu_usb_debug_bmi088_read_gyro_chip_id(void)
{
    uint8_t chip_id = 0xFFu;

    BMI088_GPIO_init();
    BMI088_com_init();
    BMI088_GYRO_NS_L();
    BMI088_read_write_byte(BMI088_GYRO_CHIP_ID | 0x80u);
    chip_id = BMI088_read_write_byte(0x55u);
    BMI088_GYRO_NS_H();
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    return chip_id;
}

static uint8_t imu_usb_debug_bmi088_accel_init(imu_usb_debug_ctx_t *ctx)
{
    uint8_t readback = 0xFFu;

    BMI088_GPIO_init();
    BMI088_com_init();

    imu_usb_debug_bmi088_accel_write_single_reg(BMI088_ACC_PWR_CONF, BMI088_ACC_PWR_ACTIVE_MODE);
    delay_ms(5u);
    readback = imu_usb_debug_bmi088_accel_read_single_reg(BMI088_ACC_PWR_CONF);
    ctx->bmi_accel_pwr_conf_readback = readback;
    if (readback != BMI088_ACC_PWR_ACTIVE_MODE)
    {
        return BMI088_ACC_PWR_CONF_ERROR;
    }

    imu_usb_debug_bmi088_accel_write_single_reg(BMI088_ACC_PWR_CTRL, BMI088_ACC_ENABLE_ACC_ON);
    delay_ms(1u);
    readback = imu_usb_debug_bmi088_accel_read_single_reg(BMI088_ACC_PWR_CTRL);
    ctx->bmi_accel_pwr_ctrl_readback = readback;

    imu_usb_debug_bmi088_accel_write_single_reg(BMI088_ACC_CONF, BMI088_ACC_NORMAL | BMI088_ACC_800_HZ | BMI088_ACC_CONF_MUST_Set);
    delay_ms(1u);
    readback = imu_usb_debug_bmi088_accel_read_single_reg(BMI088_ACC_CONF);
    ctx->bmi_accel_conf_readback = readback;
    if (readback != (BMI088_ACC_NORMAL | BMI088_ACC_800_HZ | BMI088_ACC_CONF_MUST_Set))
    {
        return BMI088_ACC_CONF_ERROR;
    }

    imu_usb_debug_bmi088_accel_write_single_reg(BMI088_ACC_RANGE, BMI088_ACC_RANGE_3G);
    delay_ms(1u);
    readback = imu_usb_debug_bmi088_accel_read_single_reg(BMI088_ACC_RANGE);
    ctx->bmi_accel_range_readback = readback;
    if (readback != BMI088_ACC_RANGE_3G)
    {
        return BMI088_ACC_RANGE_ERROR;
    }

    return BMI088_NO_ERROR;
}

static uint8_t imu_usb_debug_ist8310_read_single_reg(uint8_t reg)
{
    uint8_t value = 0xFFu;

    if (HAL_I2C_Mem_Read(&hi2c3, IST8310_IIC_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, &value, 1, 100u) != HAL_OK)
    {
        return 0xFFu;
    }

    return value;
}

static void imu_usb_debug_ist8310_write_single_reg(uint8_t reg, uint8_t value)
{
    (void)HAL_I2C_Mem_Write(&hi2c3, IST8310_IIC_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, &value, 1, 100u);
}

static uint8_t imu_usb_debug_ist8310_init(void)
{
    static const uint8_t reg_cfg[4][2] =
    {
        {0x0Bu, 0x08u},
        {0x41u, 0x09u},
        {0x42u, 0xC0u},
        {0x0Au, 0x0Bu}
    };
    uint8_t readback = 0u;
    uint8_t idx = 0u;

    ist8310_GPIO_init();
    ist8310_com_init();

    ist8310_RST_L();
    delay_ms(50u);
    ist8310_RST_H();
    delay_ms(50u);

    readback = imu_usb_debug_ist8310_read_single_reg(0x00u);
    if (readback != 0x10u)
    {
        return IST8310_NO_SENSOR;
    }

    delay_ms(1u);

    for (idx = 0u; idx < 4u; idx++)
    {
        imu_usb_debug_ist8310_write_single_reg(reg_cfg[idx][0], reg_cfg[idx][1]);
        delay_ms(1u);
        readback = imu_usb_debug_ist8310_read_single_reg(reg_cfg[idx][0]);
        delay_ms(1u);
        if (readback != reg_cfg[idx][1])
        {
            return (uint8_t)(idx + 1u);
        }
    }

    return IST8310_NO_ERROR;
}

static void imu_usb_debug_ist8310_read_mag(fp32 mag[3])
{
    uint8_t raw[6] = {0u};
    int16_t value = 0;

    if (HAL_I2C_Mem_Read(&hi2c3, IST8310_IIC_ADDRESS, 0x03u, I2C_MEMADD_SIZE_8BIT, raw, 6u, 100u) != HAL_OK)
    {
        mag[0] = 0.0f;
        mag[1] = 0.0f;
        mag[2] = 0.0f;
        return;
    }

    value = (int16_t)(((uint16_t)raw[1] << 8) | raw[0]);
    mag[0] = 0.3f * value;
    value = (int16_t)(((uint16_t)raw[3] << 8) | raw[2]);
    mag[1] = 0.3f * value;
    value = (int16_t)(((uint16_t)raw[5] << 8) | raw[4]);
    mag[2] = 0.3f * value;
}

static void imu_usb_debug_try_init(imu_usb_debug_ctx_t *ctx)
{
    if (ctx->bmi_ready == 0u)
    {
        ctx->bmi_accel_chip_id = imu_usb_debug_bmi088_read_accel_chip_id();
        ctx->bmi_gyro_chip_id = imu_usb_debug_bmi088_read_gyro_chip_id();

        if ((ctx->bmi_accel_chip_id == BMI088_ACC_CHIP_ID_VALUE) &&
            (ctx->bmi_gyro_chip_id == BMI088_GYRO_CHIP_ID_VALUE))
        {
            ctx->bmi_accel_init_error = imu_usb_debug_bmi088_accel_init(ctx);
            ctx->bmi_gyro_init_error = bmi088_gyro_init();

            if ((ctx->bmi_accel_init_error == BMI088_NO_ERROR) &&
                (ctx->bmi_gyro_init_error == BMI088_NO_ERROR))
            {
                ctx->bmi_ready = 1u;
                ctx->bmi_error = BMI088_NO_ERROR;
            }
            else if (ctx->bmi_accel_init_error != BMI088_NO_ERROR)
            {
                ctx->bmi_error = ctx->bmi_accel_init_error;
            }
            else
            {
                ctx->bmi_error = ctx->bmi_gyro_init_error;
            }
        }
        else
        {
            ctx->bmi_accel_init_error = (uint8_t)((ctx->bmi_accel_chip_id == BMI088_ACC_CHIP_ID_VALUE) ? BMI088_NO_ERROR : BMI088_NO_SENSOR);
            ctx->bmi_gyro_init_error = (uint8_t)((ctx->bmi_gyro_chip_id == BMI088_GYRO_CHIP_ID_VALUE) ? BMI088_NO_ERROR : BMI088_NO_SENSOR);
            ctx->bmi_error = BMI088_NO_SENSOR;
        }
    }

    if (ctx->ist_ready == 0u)
    {
        ctx->ist_chip_id = imu_usb_debug_ist8310_read_single_reg(0x00u);
        ctx->ist_error = imu_usb_debug_ist8310_init();
        ctx->ist_ready = (uint8_t)((ctx->ist_error == IST8310_NO_ERROR) ? 1u : 0u);
    }
}

static void imu_usb_debug_temp_control(imu_usb_debug_ctx_t *ctx, fp32 temp)
{
    uint16_t pwm = 0u;

    if (ctx->temp_pid_inited == 0u)
    {
        PID_init(&ctx->temp_pid,
                 PID_POSITION,
                 g_imu_usb_debug_temp_pid_param,
                 IMU_USB_DEBUG_TEMP_PID_MAX_OUT,
                 IMU_USB_DEBUG_TEMP_PID_MAX_IOUT);
        ctx->temp_pid_inited = 1u;
    }

    if (ctx->temp_first_reached != 0u)
    {
        PID_calc(&ctx->temp_pid, temp, IMU_USB_DEBUG_TEMP_TARGET_C);
        if (ctx->temp_pid.out < 0.0f)
        {
            ctx->temp_pid.out = 0.0f;
        }
        if (ctx->temp_pid.out > (fp32)(IMU_USB_DEBUG_TEMP_PWM_MAX - 1u))
        {
            ctx->temp_pid.out = (fp32)(IMU_USB_DEBUG_TEMP_PWM_MAX - 1u);
        }
        pwm = (uint16_t)ctx->temp_pid.out;
    }
    else
    {
        if (temp > IMU_USB_DEBUG_TEMP_TARGET_C)
        {
            if (ctx->temp_constant_time < 255u)
            {
                ctx->temp_constant_time++;
            }
            if (ctx->temp_constant_time > 200u)
            {
                ctx->temp_first_reached = 1u;
                ctx->temp_pid.Iout = IMU_USB_DEBUG_TEMP_PWM_MAX / 2.0f;
            }
        }
        else
        {
            ctx->temp_constant_time = 0u;
        }

        pwm = IMU_USB_DEBUG_TEMP_PWM_MAX - 1u;
    }

    imu_pwm_set(pwm);
}

static void imu_usb_debug_send_frame(imu_usb_debug_ctx_t *ctx)
{
    int msg_len = 0;

    if (ctx->bmi_ready != 0u)
    {
        msg_len = snprintf(ctx->tx_buffer,
                           sizeof(ctx->tx_buffer),
                           "%.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f\r\n",
                           ctx->gyro[0],
                           ctx->gyro[1],
                           ctx->gyro[2],
                           ctx->accel[0],
                           ctx->accel[1],
                           ctx->accel[2],
                           ctx->mag[0],
                           ctx->mag[1],
                           ctx->mag[2]);
    }
    else
    {
        msg_len = snprintf(ctx->tx_buffer,
                           sizeof(ctx->tx_buffer),
                           "imu_init bmi_ready=%u bmi_err=%u acc_id=0x%02X gyro_id=0x%02X acc_init=%u gyro_init=%u pwr_ctrl=0x%02X pwr_conf=0x%02X acc_conf=0x%02X acc_range=0x%02X ist_ready=%u ist_err=%u ist_id=0x%02X\r\n",
                           ctx->bmi_ready,
                           ctx->bmi_error,
                           ctx->bmi_accel_chip_id,
                           ctx->bmi_gyro_chip_id,
                           ctx->bmi_accel_init_error,
                           ctx->bmi_gyro_init_error,
                           ctx->bmi_accel_pwr_ctrl_readback,
                           ctx->bmi_accel_pwr_conf_readback,
                           ctx->bmi_accel_conf_readback,
                           ctx->bmi_accel_range_readback,
                           ctx->ist_ready,
                           ctx->ist_error,
                           ctx->ist_chip_id);
    }

    if (msg_len > 0)
    {
        USB_Transmit((uint8_t *)ctx->tx_buffer, (uint16_t)msg_len);
    }
}
