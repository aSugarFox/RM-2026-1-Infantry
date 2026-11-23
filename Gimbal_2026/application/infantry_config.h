#ifndef INFANTRY_CONFIG_H
#define INFANTRY_CONFIG_H

/* Shared safety and link timing; change UART1 baud on BOTH boards together. */
#define INFANTRY_LINK_BAUDRATE                921600u
#define INFANTRY_STATUS_PERIOD_MS             2u /* Target status interval; RC forwarding has priority. */
#define INFANTRY_RC_DEADZONE                  40
#define INFANTRY_LINK_TIMEOUT_MS              100u
#define INFANTRY_MOTOR_TIMEOUT_MS             100u
#define INFANTRY_CONTROL_PERIOD_MS            2u

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

/* CAN layout. */
#define INFANTRY_SHOOT_CAN                    hcan1
#define INFANTRY_GIMBAL_CAN                   hcan2
#define INFANTRY_CAN_SHOOT_CMD_ID             0x200u
#define INFANTRY_CAN_FRIC_LEFT_ID             0x202u
#define INFANTRY_CAN_FRIC_RIGHT_ID            0x203u
#define INFANTRY_CAN_TRIGGER_ID               0x201u
#define INFANTRY_CAN_YAW_CMD_ID               0x1FFu
#define INFANTRY_CAN_YAW_FEEDBACK_ID          0x205u

/* Standalone shooter test: only CAN1 M2006 + two M3508 are connected. */
#define INFANTRY_SHOOTER_ONLY_TEST            0u

/* Full infantry integration: UART1 remote link + gimbal motors. */
#define INFANTRY_SLIPRING_TEST                0u
#define INFANTRY_SLIPRING_YAW_TEST            0u /* 0: forward RC only; 1: also test CAN2 GM6020 ID 1. */
#define INFANTRY_TEST_YAW_SPEED_RPM           3.0f /* Full stick: 18 deg/s. */
#define INFANTRY_TEST_YAW_MAX_COMMAND         3000.0f /* Raw GM6020 voltage command, not amperes. */
#define INFANTRY_TEST_YAW_DIRECTION           -1.0f
#define INFANTRY_TEST_RC_DEADZONE             40
#define INFANTRY_TEST_DEBUG_PERIOD_MS         100u

/* Set to 1 only when running the standalone DM4310 limit calibration. */
#define INFANTRY_PITCH_CALIBRATION_TEST       0u
#define DM4310_CALIB_RC_SPEED_RAD_S           0.30f /* 17.19 deg/s at full stick. */
#define DM4310_CALIB_MAX_SPEED_RAD_S          0.60f /* 34.38 deg/s ceiling. */
#define DM4310_CALIB_MAX_LEAD_RAD             0.30f /* Max tracking error, not a travel limit; too small limits speed. */
#define DM4310_CALIB_RC_DEADZONE              40
#define DM4310_CALIB_COMMAND_PERIOD_MS        50u
#define DM4310_CALIB_DEBUG_PERIOD_MS          100u

/* Absolute encoder endpoints measured on this gimbal. Do not reset motor zero. */
#define DM4310_PITCH_MEASURED_UP_RAD           1.8042f
#define DM4310_PITCH_MEASURED_DOWN_RAD         2.7678f
#define DM4310_PITCH_LIMIT_MARGIN_RAD         0.02f
/* A small startup overshoot may only recover inward; normal travel stays unchanged. */
#define DM4310_PITCH_RECOVERY_MARGIN_RAD      0.05f
#define DM4310_PITCH_RECOVERY_SPEED_RAD_S     0.10f
#define DM4310_PITCH_POSITION_TOLERANCE_RAD   0.002f
/* DM-J4310 pitch motor, position-speed mode. */
#define DM4310_CAN                            hcan2 /* 4310 is now connected to CAN2. */
#define DM4310_ESC_ID                         0x001u
#define DM4310_MASTER_ID                      0x000u /* Measured Master ID; CTRL_MODE query also detects it. */
#define DM4310_CONTROL_ID                     (0x100u + DM4310_ESC_ID)
#define DM4310_P_MAX_RAD                      12.5f
#define DM4310_V_MAX_RAD_S                    30.0f
#define DM4310_T_MAX_NM                       10.0f
#define DM4310_PITCH_UP_LIMIT_RAD             (DM4310_PITCH_MEASURED_UP_RAD + DM4310_PITCH_LIMIT_MARGIN_RAD)
#define DM4310_PITCH_DOWN_LIMIT_RAD           (DM4310_PITCH_MEASURED_DOWN_RAD - DM4310_PITCH_LIMIT_MARGIN_RAD)
#define DM4310_PITCH_MAX_SPEED_RAD_S          1.60f
#define DM4310_PITCH_RC_SPEED_RAD_S           0.70f /* Full stick: 17.19 deg/s. */
#define DM4310_PITCH_DIRECTION                -1.0f /* Stick up raises pitch: encoder radians decrease. */

#define DM4310_PITCH_MAX_LEAD_RAD              0.10f

/* Conservative first-run gimbal and shooter parameters. */
#define INFANTRY_YAW_MOTOR_DIRECTION           1.0f /* Positive motor output -> positive IMU yaw. */
#define INFANTRY_YAW_MAX_LEAD_DEG              5.0f /* Manual target lead and angle-loop error limit; idle target stays fixed. */
#define INFANTRY_YAW_RC_SPEED_DPS             186.0f /* Full stick: 36 deg/s. */
#define INFANTRY_YAW_ANGLE_KP                 8.0f
#define INFANTRY_YAW_ANGLE_KI                 0.0f
#define INFANTRY_YAW_ANGLE_KD                 0.2f
#define INFANTRY_YAW_MAX_SPEED_RPM            30.0f /* Absolute gimbal rate ceiling: 60 deg/s. */
#define INFANTRY_YAW_SPEED_KP                 500.0f /* Stronger proportional response at low RPM. */
#define INFANTRY_YAW_SPEED_KI                 0.55f
#define INFANTRY_YAW_MAX_INTEGRAL             3000.0f /* Limit residual drive when changing direction. */
#define INFANTRY_YAW_SPEED_KD                 0.0f
#define INFANTRY_YAW_MAX_CURRENT              16000.0f /* Raw GM6020 voltage command, including feedforward. */
/* Initial estimate from spin logs: command 10000 gives about 40 motor RPM. */
#define INFANTRY_YAW_CHASSIS_FF_PER_RPM       250.0f

#define INFANTRY_FRIC_SPEED_RPM               8500.0f
#define INFANTRY_FRIC_LEFT_DIRECTION          -1.0f
#define INFANTRY_FRIC_RIGHT_DIRECTION         1.0f
#define INFANTRY_TRIGGER_FORWARD_RPM          3000.0f
#define INFANTRY_TRIGGER_REVERSE_RPM          800.0f
#define INFANTRY_TRIGGER_DIRECTION            1.0f
#define INFANTRY_TRIGGER_REVERSE_MS           1000u
#define INFANTRY_FRIC_MAX_CURRENT             4000.0f
#define INFANTRY_TRIGGER_MAX_CURRENT          2500.0f

/* Hold mouse-right to use USB aim offsets; right switch still authorizes firing. */
#define INFANTRY_AUTO_AIM_REQUEST(rc)          ((rc)->mouse.press_r != 0u)
#define INFANTRY_DEBUG_PERIOD_MS              200u
/* 1: only dual-IMU telemetry; 0: restore LINK/MOVE/SYS/YAW/SHOT/DM. */
#define INFANTRY_DEBUG_IMU_ONLY               0u
/* Temporarily append chassis direction feedback to the IMU log. */
#define INFANTRY_DEBUG_DRIVE_DIRECTION        1u
/* Vision offsets are signed centi-degrees. */
#define INFANTRY_VISION_TIMEOUT_MS             120u
#define INFANTRY_VISION_YAW_GAIN               0.01f
#define INFANTRY_VISION_PITCH_GAIN             0.000174532925f
#define INFANTRY_VISION_YAW_DIRECTION          -1.0f
#define INFANTRY_VISION_PITCH_DIRECTION        -1.0f

/* Status indication. */
#define INFANTRY_LED_RED                       0xFFFF0000u
#define INFANTRY_LED_YELLOW                    0xFFFFFF00u
#define INFANTRY_LED_GREEN                     0xFF00FF00u
#define INFANTRY_BUZZER_ENABLE                 1u /* Local IMU initialized: two short beeps. */
#define INFANTRY_BUZZER_FREQUENCY_HZ           4000u
#define INFANTRY_BUZZER_ON_MS                  80u
#define INFANTRY_BUZZER_GAP_MS                 100u
#define INFANTRY_BUZZER_PATTERN_GAP_MS         350u

/* USART1 fixed frame protocol. */
/* 20-byte diagnostic frame: angle in centidegrees, four targets, four feedback RPMs. */
#define INFANTRY_FRAME_CHASSIS_MOTION          0xA8u
#define INFANTRY_FRAME_CHASSIS_IMU             0xA9u
#define INFANTRY_CHASSIS_IMU_PERIOD_MS         100u
#define INFANTRY_CHASSIS_MOTION_PERIOD_MS      100u
#define INFANTRY_FRAME_LENGTH                  20u
#define INFANTRY_FRAME_CHASSIS_STATUS          0xA5u
#define INFANTRY_FRAME_REMOTE                  0xA6u
#define INFANTRY_FRAME_GIMBAL_STATUS           0xA7u

#define INFANTRY_CHASSIS_FLAG_IMU_READY        (1u << 0)
#define INFANTRY_CHASSIS_FLAG_ENABLED          (1u << 1)
#define INFANTRY_CHASSIS_FLAG_SMALL_GYRO       (1u << 2)
#define INFANTRY_CHASSIS_FLAG_REMOTE_READY     (1u << 3)

#define INFANTRY_GIMBAL_FLAG_IMU_READY         (1u << 0)
#define INFANTRY_GIMBAL_FLAG_READY             (1u << 1)
#define INFANTRY_GIMBAL_FLAG_SHOOT_READY       (1u << 2)
#define INFANTRY_GIMBAL_FLAG_ESTOP             (1u << 3)
#define INFANTRY_GIMBAL_FLAG_ENCODER_READY    (1u << 4) /* A7 bytes 12..13 valid. */

#endif
