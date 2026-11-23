#include "infantry_imu.h"

#include "BMI088driver.h"
#include "imu_dynamic_ekf.h"
#include "infantry_config.h"
#include "main.h"
#include <math.h>

#define RAD_TO_DEG 57.29577951308232f
#define RAD_S_TO_RPM 9.54929658551372f

typedef struct
{
    uint8_t ready;
    uint8_t error;
    uint32_t last_tick;
    float gyro_offset[3];
    float yaw_deg;
    float pitch_deg;
    float roll_deg;
    float gyro_z_rpm;
} infantry_imu_t;

static infantry_imu_t imu;
static imu_dynamic_ekf_t infantry_ekf;
static uint8_t imu_initialized_once;

static float wrap_deg(float angle)
{
    while (angle > 180.0f) angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}

uint8_t Infantry_IMU_Init(void)
{
    float gyro_raw[3];
    float accel_raw[3];
    float temperature;
    float gyro_sum[3] = {0.0f, 0.0f, 0.0f};
    uint16_t i;

    /* Do not silently re-zero one board after a runtime fault. Reboot both aligned. */
    if (imu_initialized_once != 0u) return Infantry_IMU_IsReady();
    imu.ready = 0u;
    imu.error = BMI088_init();
    if (imu.error != BMI088_NO_ERROR)
    {
        return 0u;
    }

    for (i = 0u; i < INFANTRY_IMU_CALIBRATION_SAMPLES; i++)
    {
        BMI088_read(gyro_raw, accel_raw, &temperature);
        if (BMI088_LastReadValid() == 0u) { imu.error = 0x80u; return 0u; }
        if ((fabsf(gyro_raw[0]) > 0.10f) || (fabsf(gyro_raw[1]) > 0.10f) ||
            (fabsf(gyro_raw[2]) > 0.10f)) { imu.error = 0x81u; return 0u; }
        gyro_sum[0] += INFANTRY_IMU_GYRO_X(gyro_raw);
        gyro_sum[1] += INFANTRY_IMU_GYRO_Y(gyro_raw);
        gyro_sum[2] += INFANTRY_IMU_GYRO_Z(gyro_raw);
        HAL_Delay(INFANTRY_IMU_CALIBRATION_DELAY_MS);
    }

    imu.gyro_offset[0] = gyro_sum[0] / (float)INFANTRY_IMU_CALIBRATION_SAMPLES;
    imu.gyro_offset[1] = gyro_sum[1] / (float)INFANTRY_IMU_CALIBRATION_SAMPLES;
    imu.gyro_offset[2] = gyro_sum[2] / (float)INFANTRY_IMU_CALIBRATION_SAMPLES;

    BMI088_read(gyro_raw, accel_raw, &temperature);
    if (BMI088_LastReadValid() == 0u) { imu.error = 0x80u; return 0u; }
    {
        float ax = INFANTRY_IMU_ACCEL_X(accel_raw);
        float ay = INFANTRY_IMU_ACCEL_Y(accel_raw);
        float az = INFANTRY_IMU_ACCEL_Z(accel_raw);
        float pitch = atan2f(-ax, sqrtf(ay * ay + az * az));
        float roll = atan2f(ay, az);
        float cp = cosf(0.5f * pitch), sp = sinf(0.5f * pitch);
        float cr = cosf(0.5f * roll), sr = sinf(0.5f * roll);
        /* Start with measured tilt and zero horizontal heading, not identity. */
        float init_q[4] = {cr * cp, sr * cp, cr * sp, -sr * sp};
        IMU_DynamicEKF_Init(&infantry_ekf, init_q, 0,
            INFANTRY_IMU_EKF_Q_NOISE, INFANTRY_IMU_EKF_BIAS_NOISE,
            INFANTRY_IMU_EKF_ACCEL_NOISE, INFANTRY_IMU_EKF_CHI_THRESHOLD);
        imu.pitch_deg = pitch * RAD_TO_DEG;
        imu.roll_deg = roll * RAD_TO_DEG;
    }
    imu.yaw_deg = 0.0f;
    imu.gyro_z_rpm = 0.0f;
    imu.last_tick = HAL_GetTick();
    imu.ready = 1u;
    imu_initialized_once = 1u;
    return 1u;
}

void Infantry_IMU_Poll(void)
{
    float gyro_raw[3];
    float accel_raw[3];
    float temperature;
    float gx;
    float gy;
    float gz;
    float ax;
    float ay;
    float az;
    float gyro[3], accel[3];
    float roll, pitch_sin, cos_pitch;
    const float *q;
    float dt;
    uint32_t now;

    if (imu.ready == 0u) return;

    now = HAL_GetTick();
    if ((uint32_t)(now - imu.last_tick) < INFANTRY_CONTROL_PERIOD_MS) return;

    if ((uint32_t)(now - imu.last_tick) > INFANTRY_IMU_STALE_MS)
    {
        imu.ready = 0u; imu.error = 0x82u; return;
    }
    dt = (float)(now - imu.last_tick) * 0.001f;
    if (dt > 0.05f) dt = (float)INFANTRY_CONTROL_PERIOD_MS * 0.001f;
    imu.last_tick = now;

    BMI088_read(gyro_raw, accel_raw, &temperature);
    if (BMI088_LastReadValid() == 0u)
    {
        imu.ready = 0u; imu.error = 0x80u; return;
    }
    gx = INFANTRY_IMU_GYRO_X(gyro_raw) - imu.gyro_offset[0];
    gy = INFANTRY_IMU_GYRO_Y(gyro_raw) - imu.gyro_offset[1];
    gz = INFANTRY_IMU_GYRO_Z(gyro_raw) - imu.gyro_offset[2];
    ax = INFANTRY_IMU_ACCEL_X(accel_raw);
    ay = INFANTRY_IMU_ACCEL_Y(accel_raw);
    az = INFANTRY_IMU_ACCEL_Z(accel_raw);

    gyro[0] = gx; gyro[1] = gy; gyro[2] = gz;
    accel[0] = ax; accel[1] = ay; accel[2] = az;
    IMU_DynamicEKF_Update(&infantry_ekf, gyro, accel, dt);
    q = infantry_ekf.q;
    roll = atan2f(2.0f * (q[0] * q[1] + q[2] * q[3]),
                  1.0f - 2.0f * (q[1] * q[1] + q[2] * q[2]));
    pitch_sin = 2.0f * (q[0] * q[2] - q[3] * q[1]);
    if (pitch_sin > 1.0f) pitch_sin = 1.0f;
    if (pitch_sin < -1.0f) pitch_sin = -1.0f;
    cos_pitch = sqrtf(1.0f - pitch_sin * pitch_sin);
    /* Horizontal heading is undefined if the board's forward axis is vertical. */
    if (cos_pitch < 0.1f) { imu.ready = 0u; imu.error = 0x83u; return; }
    imu.yaw_deg = wrap_deg(infantry_ekf.yaw * RAD_TO_DEG);
    imu.pitch_deg = asinf(pitch_sin) * RAD_TO_DEG;
    imu.roll_deg = roll * RAD_TO_DEG;
    /* Preserve the public API: this now means horizontal yaw rate, not body Z.
     * The gimbal board pitches with the barrel, so raw gz is insufficient. */
    imu.gyro_z_rpm = (((gy - infantry_ekf.gyro_bias[1]) * sinf(roll) +
                       gz * cosf(roll)) / cos_pitch) * RAD_S_TO_RPM;
}

uint8_t Infantry_IMU_IsReady(void)
{
    return (uint8_t)((imu.ready != 0u) &&
                     ((uint32_t)(HAL_GetTick() - imu.last_tick) <= INFANTRY_IMU_STALE_MS));
}

uint8_t Infantry_IMU_GetError(void) { return imu.error; }
float Infantry_IMU_GetYawDeg(void) { return imu.yaw_deg; }
float Infantry_IMU_GetPitchDeg(void) { return imu.pitch_deg; }
float Infantry_IMU_GetRollDeg(void) { return imu.roll_deg; }
float Infantry_IMU_GetGyroZRpm(void) { return imu.gyro_z_rpm; }
