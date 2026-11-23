#ifndef INFANTRY_CONFIG_H
#define INFANTRY_CONFIG_H

/* Shared safety and link timing; change UART1 baud on BOTH boards together. */
#define INFANTRY_LINK_BAUDRATE                921600u
#define INFANTRY_STATUS_PERIOD_MS             2u /* Target status interval; RC forwarding has priority. */
#define INFANTRY_RC_DEADZONE                  40
#define INFANTRY_LINK_TIMEOUT_MS              100u
#define INFANTRY_MOTOR_TIMEOUT_MS             100u
#define INFANTRY_CONTROL_PERIOD_MS            2u

/* Slip-ring test: left stick only, in chassis coordinates; no rotation. */
#define INFANTRY_SLIPRING_TEST                 0u
#define INFANTRY_TEST_CHASSIS_RC_SCALE         0.5f
#define INFANTRY_TEST_CHASSIS_MAX_CURRENT      3000.0f
#define INFANTRY_TEST_RC_DEADZONE              40

/* Reuse the working old chassis quaternion EKF; gyro and accel use the same axes. */
#define INFANTRY_IMU_EKF_Q_NOISE              10.0f
#define INFANTRY_IMU_EKF_BIAS_NOISE           0.001f
#define INFANTRY_IMU_EKF_ACCEL_NOISE          1000000.0f
#define INFANTRY_IMU_EKF_CHI_THRESHOLD        1.0e-8f

/* BMI088 startup and board-axis mapping. */
#define INFANTRY_IMU_CALIBRATION_SAMPLES      300u
#define INFANTRY_IMU_CALIBRATION_DELAY_MS     2u
#define INFANTRY_IMU_STALE_MS                 100u
#define INFANTRY_IMU_GYRO_X(raw)              ((raw)[1])
#define INFANTRY_IMU_GYRO_Y(raw)              (-(raw)[0])
#define INFANTRY_IMU_GYRO_Z(raw)              ((raw)[2])
#define INFANTRY_IMU_ACCEL_X(raw)             ((raw)[1])
#define INFANTRY_IMU_ACCEL_Y(raw)             (-(raw)[0])
#define INFANTRY_IMU_ACCEL_Z(raw)             ((raw)[2])

/* Low initial speeds. Tune these only after the direction test passes. */
#define INFANTRY_CHASSIS_VX_SCALE             1.5f
#define INFANTRY_CHASSIS_VY_SCALE             1.5f
/* RC translation polarity: independent of speed magnitude and spin direction. */
#define INFANTRY_CHASSIS_VX_DIRECTION         (-1.0f)
#define INFANTRY_CHASSIS_VY_DIRECTION         (-1.0f)
#define INFANTRY_CHASSIS_SMALL_GYRO_SPEED     8000.0f /* Old kinematics scale: each wheel adds 0.25 * this rpm. */

/* Left-middle dial (ch[4]): same direction as the old chassis, lower speed. */
#define INFANTRY_CHASSIS_DIAL_SCALE         2.0f /* Full dial: wheel target 165 rpm. */

/* Restore the reference chassis speed-loop proportional gains.
 * Keep the existing 2 ms loop, integral gains, speed and current limits. */
#define INFANTRY_CHASSIS_SPEED_KP             9.5f
#define INFANTRY_CHASSIS_SPEED_KP_M2          9.0f
#define INFANTRY_CHASSIS_MAX_CURRENT          3000.0f
/* Same IMU-to-wheel heading convention as old CHASSIS_GYRO_YAW_SIGN.
 * Apply to BOTH IMU headings before taking gimbal minus chassis. */
#define INFANTRY_CHASSIS_YAW_SIGN             (-1.0f)
/* Both IMUs zero at boot: align muzzle with chassis front and keep still. */
#define INFANTRY_GIMBAL_BOOT_OFFSET_DEG        0.0f

/* Status indication. */
#define INFANTRY_LED_RED                      0xFFFF0000u
#define INFANTRY_LED_YELLOW                   0xFFFFFF00u
#define INFANTRY_LED_GREEN                    0xFF00FF00u
#define INFANTRY_BUZZER_ENABLE                1u /* Local IMU initialized: one short beep. */
#define INFANTRY_BUZZER_FREQUENCY_HZ          4000u
#define INFANTRY_BUZZER_ON_MS                 80u
#define INFANTRY_BUZZER_GAP_MS                100u
#define INFANTRY_BUZZER_PATTERN_GAP_MS        350u

/* USART1 fixed frame protocol. */
/* 20-byte diagnostic frame: angle in centidegrees, four targets, four feedback RPMs. */
#define INFANTRY_FRAME_CHASSIS_MOTION          0xA8u
#define INFANTRY_FRAME_CHASSIS_IMU             0xA9u
#define INFANTRY_CHASSIS_IMU_PERIOD_MS         100u
#define INFANTRY_CHASSIS_MOTION_PERIOD_MS      100u
#define INFANTRY_FRAME_LENGTH                 20u
#define INFANTRY_FRAME_CHASSIS_STATUS         0xA5u
#define INFANTRY_FRAME_REMOTE                 0xA6u
#define INFANTRY_FRAME_GIMBAL_STATUS          0xA7u

#define INFANTRY_CHASSIS_FLAG_IMU_READY       (1u << 0)
#define INFANTRY_CHASSIS_FLAG_ENABLED         (1u << 1)
#define INFANTRY_CHASSIS_FLAG_SMALL_GYRO      (1u << 2)
#define INFANTRY_CHASSIS_FLAG_REMOTE_READY    (1u << 3)

#define INFANTRY_GIMBAL_FLAG_IMU_READY        (1u << 0)
#define INFANTRY_GIMBAL_FLAG_READY            (1u << 1)
#define INFANTRY_GIMBAL_FLAG_SHOOT_READY      (1u << 2)
#define INFANTRY_GIMBAL_FLAG_ESTOP            (1u << 3)

#endif
