/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "can.h"
#include "dma.h"
#include "i2c.h"
#include "rtc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bsp_delay.h"
#include "bsp_led.h"
#include "remote_control.h"
#include "Control_Gimbal.h"
#include "Control_Ghassis.h"
#include "state_machine.h"
#include "Control_Shoot.h"
#include "Vision_Control.h"
#include "WIT_Gyro.h"
#include "usart6_remote_control.h"
#include "Control_Shoot_MouseSM.h"
#include "imu_usb_debug.h"
#include "string.h"
#include <stdio.h>
#include "bsp_can.h"
#include "CAN_receive.h"
#include "infantry_config.h"
#include "infantry_gimbal.h"
#include "infantry_imu.h"
#include "infantry_indicator.h"
#include "dm4310.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
state_machine_t g_state_machine;//状态机变量
extern uint8_t usart1_rx_buffer[USART1_FRAME_LENGTH];
/*
 * USART1接收诊断变量：
 * - g_usart1_rx_start_status：初始化时首次HAL_UART_Receive_IT返回值（0通常表示HAL_OK）。
 * - g_usart1_rx_irq_cnt：USART1接收完成回调进入次数，用于确认中断是否在跑。
 * - g_usart1_rx_restart_fail_cnt：回调内重启接收失败次数，用于发现串口状态卡死/忙状态。
 */
static volatile uint8_t g_usart1_rx_start_status = 0xFFu;
static volatile uint32_t g_usart1_rx_irq_cnt = 0u;
static volatile uint32_t g_usart1_rx_restart_fail_cnt = 0u;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  delay_init(); /* Initialize delays after switching to 168 MHz. */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM5_Init();
  MX_TIM4_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_USART6_UART_Init();
  MX_ADC1_Init();
  MX_ADC3_Init();
  MX_RTC_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM8_Init();
  MX_TIM10_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_I2C3_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_TIM14_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
  BSP_LED_Init();                             //LED初始化
  Infantry_Indicator_Init();
  BSP_CAN_Init();
  Infantry_Gimbal_Init();
  state_machine_init(&g_state_machine);       //状态机初始化
  usart1_remote_control_init();               //USART1遥控器数据接收相关初始化        
  remote_control_init(); /* UART1 DMA must be ready before the DBUS ISR forwards data. */
  /*
   * 启动USART1中断接收：
   * 1) 这里先按20字节块启动一次接收（底盘状态帧固定20字节）。
   * 2) 在HAL_UART_RxCpltCallback中会持续重启，形成持续接收循环。
   * 3) 回调里使用“流解析器”处理，即使出现错位/粘包也能自动重同步。
   */
  g_usart1_rx_start_status = (uint8_t)HAL_UART_Receive_IT(&huart1, usart1_rx_buffer, USART1_FRAME_LENGTH);
#if ((INFANTRY_PITCH_CALIBRATION_TEST == 0u) && (INFANTRY_SLIPRING_TEST == 0u))
  (void)Infantry_IMU_Init();
#endif
  // ui_auto_init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    uint32_t control_cycle_tick = HAL_GetTick();
#if ((INFANTRY_PITCH_CALIBRATION_TEST == 0u) && (INFANTRY_SLIPRING_TEST == 0u))
    static uint32_t imu_retry_tick = 0u;
#endif
    static uint32_t debug_tick = 0u;
#if ((INFANTRY_SLIPRING_TEST == 0u) || (INFANTRY_SLIPRING_YAW_TEST != 0u))
    static uint8_t status_sequence = 0u;
    usart1_gimbal_tx_status_t link_status;
#endif

#if ((INFANTRY_PITCH_CALIBRATION_TEST == 0u) && (INFANTRY_SLIPRING_TEST == 0u))
    remote_control_forward_poll();
    Infantry_IMU_Poll();
    if ((Infantry_IMU_IsReady() == 0u) &&
        ((uint32_t)(HAL_GetTick() - imu_retry_tick) >= 1000u))
    {
      Infantry_Gimbal_Update(get_remote_control_point(), 0u, 0u);
      Infantry_Shooter_Update(INFANTRY_SHOOT_OFF, 0u);
      imu_retry_tick = HAL_GetTick();
      (void)Infantry_IMU_Init();
    }
#endif
  /* --------------------------------------------------------------------------------------------------------------------------------------------------*/ 
    //这些指针都是电机或遥控器数据的指针，放在循环里每次都要获取最新的数据用于调试输出
   const RC_ctrl_t *rc_data = get_remote_control_point();     
  //  const RC_ctrl_t *rc_ctrl = get_usart1_remote_control_point();
  //  const motor_measure_t *fric_1 = get_fric_wheel_left_motor_measure_point();
  //  const motor_measure_t *fric_2 = get_fric_wheel_right_motor_measure_point();  
  //  const motor_measure_t *bodan_motor = get_trigger_motor_measure_point();
   const motor_measure_t *pitch_motor = get_pitch_gimbal_motor_measure_point();
#if (INFANTRY_DEBUG_IMU_ONLY == 0u)
   const motor_measure_t *yaw_motor = get_yaw_gimbal_motor_measure_point();
#endif
   const vision_coordinate_t*vision_data =  get_vision_data_point();// ?????????????
   const usart6_tuchuan_remote_control_t *tuchuan_rc_data = get_usart6_tuchuan_remote_control_point();// ???????????��?????��????????
  /* --------------------------------------------------------------------------------------------------------------------------------------------------*/  
  //这个是给视觉发板载陀螺仪数据，视觉自瞄需要云台陀螺仪数据
  // if(vision_data->x!=0 || vision_data->y!=0)
  // {
  //   imu_usb_debug_poll();
  // }
         
    //状态机更新，传入遥控器数据用于状态转换判断
    state_machine_update(&g_state_machine, rc_data);

    //状态机执行，根据当前状态输出控制指令到云台和底盘
    state_machine_execute(&g_state_machine);

#if (INFANTRY_DEBUG_IMU_ONLY != 0u)
    if ((uint32_t)(HAL_GetTick() - debug_tick) >= INFANTRY_DEBUG_PERIOD_MS)
    {
      usart1_chassis_imu_t chassis_imu;
      uint32_t primask = __get_PRIMASK();
      uint32_t now;
      uint32_t age;
      unsigned chassis_valid;
      /* The RX ISR owns this sample; copy it before formatting text. */
      __disable_irq();
      chassis_imu = *get_usart1_chassis_imu();
      if (primask == 0u) __enable_irq();
      now = HAL_GetTick();
      age = (uint32_t)(now - chassis_imu.update_tick);
      chassis_valid = (unsigned)((chassis_imu.ready != 0u) &&
          (chassis_imu.update_tick != 0u) &&
          (age <= 3u * INFANTRY_CHASSIS_IMU_PERIOD_MS));
      debug_tick = now;
      /* Degrees and degrees/s (RPM * 6). Board clocks are independent. */
      usart6_debug_printf("IMU:t=%lu Gok=%u Gyaw=%.2f Gpitch=%.2f Groll=%.2f Gdps=%.3f Cok=%u Cyaw=%.2f Cpitch=%.2f Croll=%.2f Cdps=%.3f Ct=%lu Cage=%ld\r\n",
          (unsigned long)now, (unsigned)Infantry_IMU_IsReady(),
          (double)Infantry_IMU_GetYawDeg(), (double)Infantry_IMU_GetPitchDeg(),
          (double)Infantry_IMU_GetRollDeg(), (double)(Infantry_IMU_GetGyroZRpm() * 6.0f),
          chassis_valid, (double)chassis_imu.yaw_angle_deg,
          (double)chassis_imu.pitch_angle_deg, (double)chassis_imu.roll_angle_deg,
          (double)(chassis_imu.yaw_rate_rpm * 6.0f),
          (unsigned long)chassis_imu.source_tick,
          (chassis_imu.update_tick != 0u) ? (long)age : -1L);
    }
#else
#if (INFANTRY_SLIPRING_TEST != 0u)
    if ((uint32_t)(HAL_GetTick() - debug_tick) >= INFANTRY_TEST_DEBUG_PERIOD_MS)
    {
      const volatile usart1_chassis_status_t *chassis = get_usart1_chassis_status();
      uint32_t now = HAL_GetTick();
      uint32_t chassis_tick = get_usart1_chassis_status_update_tick();
      uint32_t rc_tick = get_remote_control_update_tick();
      unsigned chassis_alive = (unsigned)((chassis_tick != 0u) &&
          ((uint32_t)(now - chassis_tick) <= INFANTRY_LINK_TIMEOUT_MS));
      unsigned remote_alive = (unsigned)((rc_tick != 0u) &&
          ((uint32_t)(now - rc_tick) <= INFANTRY_LINK_TIMEOUT_MS));
      debug_tick = now;
      usart6_debug_printf("LINK:local=%u L=%u dial=%d ch2=%d ch3=%d back=%u rxrc=%u echo2=%d echo3=%d echoL=%u imu=%u enabled=%u\r\n",
          remote_alive, (unsigned)rc_data->rc.s[RC_S_LEFT], (int)rc_data->rc.ch[4],
          (int)rc_data->rc.ch[2], (int)rc_data->rc.ch[3], chassis_alive,
          chassis_alive && ((chassis->flags & INFANTRY_CHASSIS_FLAG_REMOTE_READY) != 0u),
          (int)chassis->remote_ch2, (int)chassis->remote_ch3, (unsigned)chassis->remote_left_switch,
          chassis_alive && ((chassis->flags & INFANTRY_CHASSIS_FLAG_IMU_READY) != 0u),
          chassis_alive && ((chassis->flags & INFANTRY_CHASSIS_FLAG_ENABLED) != 0u));
#if (INFANTRY_SLIPRING_YAW_TEST != 0u)
      usart6_debug_printf("YAW:can=2 id=1 on=%u rc=%d set=%.1f rpm=%d cmd=%d ecd=%u stop=%u\r\n",
          (unsigned)CAN_yaw_motor_online(), (int)rc_data->rc.ch[0],
          (double)Infantry_YawTest_GetSpeedSet(), (int)yaw_motor->speed_rpm,
          (int)Infantry_YawTest_GetCommand(), (unsigned)yaw_motor->ecd,
          (unsigned)g_state_machine.estop);
#endif
    }
#elif (INFANTRY_PITCH_CALIBRATION_TEST != 0u)
    if ((uint32_t)(HAL_GetTick() - debug_tick) >= DM4310_CALIB_DEBUG_PERIOD_MS)
    {
      const dm4310_measure_t *dm = DM4310_GetMeasure();
      const volatile dm4310_debug_t *dm_debug = DM4310_GetDebug();
      debug_tick = HAL_GetTick();
      usart6_debug_printf("P:on=%u en=%u mode=%u err=%u rc=%d L=%u rad=%.4f deg=%.2f target=%.4f vel=%.4f\r\n",
                          (unsigned)DM4310_IsOnline(), (unsigned)(dm->error == 1u),
                          (unsigned)DM4310_PositionModeReady(), (unsigned)dm->error,
                          (int)rc_data->rc.ch[1], (unsigned)rc_data->rc.s[RC_S_LEFT],
                          (double)dm->position_rad, (double)(dm->position_rad * 57.2957795f),
                          (double)Infantry_PitchCalibration_GetTarget(),
                          (double)dm->velocity_rad_s);
      usart6_debug_printf("CAN:rx=%lu last=0x%03X master=0x%03X ctrl=%u txfail=%lu esr=0x%08lX stop=%u\r\n",
                          (unsigned long)dm_debug->rx_frames, (unsigned)dm_debug->last_rx_id,
                          (unsigned)dm_debug->feedback_id, (unsigned)dm_debug->control_mode,
                          (unsigned long)dm_debug->tx_failures,
                          (unsigned long)DM4310_CAN.Instance->ESR,
                          (unsigned)g_state_machine.estop);
      usart6_debug_printf("CAL:stick=%.3f vmax=%.3f lead=%.3f\r\n",
                          (double)DM4310_CALIB_RC_SPEED_RAD_S,
                          (double)((DM4310_CALIB_MAX_SPEED_RAD_S < DM4310_PITCH_MAX_SPEED_RAD_S) ?
                              DM4310_CALIB_MAX_SPEED_RAD_S : DM4310_PITCH_MAX_SPEED_RAD_S),
                          (double)DM4310_CALIB_MAX_LEAD_RAD);
    }
#elif (INFANTRY_SHOOTER_ONLY_TEST != 0u)
    if ((uint32_t)(HAL_GetTick() - debug_tick) >= INFANTRY_DEBUG_PERIOD_MS)
    {
      const motor_measure_t *trigger = get_trigger_motor_measure_point();
      const motor_measure_t *fric_left = get_fric_wheel_left_motor_measure_point();
      const motor_measure_t *fric_right = get_fric_wheel_right_motor_measure_point();
      debug_tick = HAL_GetTick();
      usart6_debug_printf("shoot=%u ready=%u T:%d/%d L:%d/%d R:%d/%d\r\n",
                          (unsigned)g_state_machine.launcher_mode,
                          (unsigned)g_state_machine.shoot_ready,
                          (int)trigger->speed_rpm, (int)trigger->given_current,
                          (int)fric_left->speed_rpm, (int)fric_left->given_current,
                          (int)fric_right->speed_rpm, (int)fric_right->given_current);
    }
#else
    if ((uint32_t)(HAL_GetTick() - debug_tick) >= INFANTRY_DEBUG_PERIOD_MS)
    {
      const volatile usart1_chassis_status_t *chassis = get_usart1_chassis_status();
      const dm4310_measure_t *dm = DM4310_GetMeasure();
      const volatile dm4310_debug_t *dm_debug = DM4310_GetDebug();
      uint32_t now = HAL_GetTick();
      uint32_t chassis_tick = get_usart1_chassis_status_update_tick();
      uint32_t rc_tick = get_remote_control_update_tick();
      unsigned chassis_alive = (unsigned)((chassis_tick != 0u) &&
          ((uint32_t)(now - chassis_tick) <= INFANTRY_LINK_TIMEOUT_MS));
      unsigned remote_alive = (unsigned)((rc_tick != 0u) &&
          ((uint32_t)(now - rc_tick) <= INFANTRY_LINK_TIMEOUT_MS));
      debug_tick = now;
      usart6_debug_printf("LINK:local=%u L=%u dial=%d ch2=%d ch3=%d back=%u rxrc=%u echo2=%d echo3=%d echoL=%u imu=%u enabled=%u\r\n",
          remote_alive, (unsigned)rc_data->rc.s[RC_S_LEFT], (int)rc_data->rc.ch[4],
          (int)rc_data->rc.ch[2], (int)rc_data->rc.ch[3], chassis_alive,
          chassis_alive && ((chassis->flags & INFANTRY_CHASSIS_FLAG_REMOTE_READY) != 0u),
          (int)chassis->remote_ch2, (int)chassis->remote_ch3,
          (unsigned)chassis->remote_left_switch,
          chassis_alive && ((chassis->flags & INFANTRY_CHASSIS_FLAG_IMU_READY) != 0u),
          chassis_alive && ((chassis->flags & INFANTRY_CHASSIS_FLAG_ENABLED) != 0u));
      {
        const volatile usart1_chassis_motion_t *motion = get_usart1_chassis_motion();
        unsigned fresh = (unsigned)((motion->update_tick != 0u) &&
            ((uint32_t)(now - motion->update_tick) <= 3u * INFANTRY_CHASSIS_MOTION_PERIOD_MS));
        usart6_debug_printf("MOVE:ok=%u rel=%.2f set=%d,%d,%d,%d rpm=%d,%d,%d,%d\r\n",
            fresh, (double)motion->relative_yaw_deg,
            (int)motion->target_rpm[0], (int)motion->target_rpm[1],
            (int)motion->target_rpm[2], (int)motion->target_rpm[3],
            (int)motion->actual_rpm[0], (int)motion->actual_rpm[1],
            (int)motion->actual_rpm[2], (int)motion->actual_rpm[3]);
      }
      usart6_debug_printf("SYS:mode=%u estop=%u gimbal=%u shoot=%u YAW:on=%u rpm=%d P:on=%u err=%u rad=%.4f target=%.4f imu=%u/%u cf=%u\r\n",
          (unsigned)g_state_machine.system_mode, (unsigned)g_state_machine.estop,
          (unsigned)state_machine_gimbal_ready(&g_state_machine),
          (unsigned)state_machine_shoot_ready(&g_state_machine),
          (unsigned)CAN_yaw_motor_online(), (int)yaw_motor->speed_rpm,
          (unsigned)DM4310_IsOnline(), (unsigned)dm->error,
          (double)dm->position_rad, (double)Infantry_Gimbal_GetPitchTarget(),
          (unsigned)Infantry_IMU_IsReady(), (unsigned)Infantry_IMU_GetError(),
          (unsigned)chassis->fault);
      {
        const can_yaw_debug_t *yaw_debug = CAN_yaw_get_debug();
        usart6_debug_printf("YAW:rc=%d angle=%.2f target=%.2f set=%.2f gyro=%.2f rpm=%d base=%.2f ff=%.0f follow=%u cmd=%d q=%lu fail=%lu esr=0x%08lX\r\n",
            (int)rc_data->rc.ch[0], (double)Infantry_IMU_GetYawDeg(),
            (double)Infantry_Gimbal_GetYawTarget(),
            (double)Infantry_Gimbal_GetYawSpeedSet(), (double)Infantry_IMU_GetGyroZRpm(),
            (int)yaw_motor->speed_rpm, (double)Infantry_Gimbal_GetChassisYawRate(),
            (double)Infantry_Gimbal_GetYawFeedforward(),
            (unsigned)Infantry_Gimbal_IsDialFollowing(), (int)yaw_debug->command,
            (unsigned long)yaw_debug->tx_queued, (unsigned long)yaw_debug->tx_failed,
            (unsigned long)hcan2.Instance->ESR);
      }
      {
        const volatile can_shooter_debug_t *shot = CAN_shooter_get_debug();
        uint8_t online = CAN_shooter_online_mask();
        usart6_debug_printf("SHOT:Rsw=%u mode=%u T=%u L=%u R=%u rx=%lu ids=0x%02X last=0x%03X fail=%lu esr=0x%08lX\r\n",
            (unsigned)rc_data->rc.s[RC_S_RIGHT], (unsigned)g_state_machine.launcher_mode,
            (unsigned)((online & 1u) != 0u), (unsigned)((online & 2u) != 0u),
            (unsigned)((online & 4u) != 0u), (unsigned long)shot->rx_frames,
            (unsigned)shot->seen_ids, (unsigned)shot->last_rx_id,
            (unsigned long)shot->tx_failed, (unsigned long)hcan1.Instance->ESR);
      }
      usart6_debug_printf("DM:rx=%lu last=0x%03X master=0x%03X ctrl=%u txfail=%lu stop=%u phase=%u limits=%.4f..%.4f\r\n",
          (unsigned long)dm_debug->rx_frames, (unsigned)dm_debug->last_rx_id,
          (unsigned)dm_debug->feedback_id, (unsigned)dm_debug->control_mode,
          (unsigned long)dm_debug->tx_failures, (unsigned)g_state_machine.estop,
          (unsigned)Infantry_Gimbal_GetPitchPhase(),
          (double)DM4310_PITCH_UP_LIMIT_RAD,
          (double)DM4310_PITCH_DOWN_LIMIT_RAD);
    }
#endif
    //串口1发送上云台陀螺仪数据，供给底盘运动解算使用，建议后续改成DMA发送，减少CPU占用
#endif /* INFANTRY_DEBUG_IMU_ONLY */
#if ((INFANTRY_SLIPRING_TEST == 0u) || (INFANTRY_SLIPRING_YAW_TEST != 0u))
    memset(&link_status, 0, sizeof(link_status));
    if (Infantry_IMU_IsReady() != 0u) link_status.flags |= INFANTRY_GIMBAL_FLAG_IMU_READY;
    if (state_machine_gimbal_ready(&g_state_machine) != 0u) link_status.flags |= INFANTRY_GIMBAL_FLAG_READY;
    if (state_machine_shoot_ready(&g_state_machine) != 0u) link_status.flags |= INFANTRY_GIMBAL_FLAG_SHOOT_READY;
    if (state_machine_estop(&g_state_machine) != 0u) link_status.flags |= INFANTRY_GIMBAL_FLAG_ESTOP;
    link_status.fault = Infantry_IMU_GetError();
    if ((DM4310_GetMeasure()->error > 1u) && (DM4310_GetMeasure()->error < 0x0Fu))
    {
      link_status.fault |= 0x02u;
    }
    link_status.sequence = status_sequence++;
    link_status.yaw_angle_deg = Infantry_IMU_GetYawDeg();
    link_status.yaw_speed_rpm = Infantry_IMU_GetGyroZRpm();
    usart1_send_gimbal_status(&link_status);
#endif

    remote_control_forward_poll();
    usart6_debug_poll();
    Infantry_Indicator_Poll();
    /* Include work time in the period; interrupts still receive RC/CAN while idle. */
    while ((uint32_t)(HAL_GetTick() - control_cycle_tick) < INFANTRY_CONTROL_PERIOD_MS)
        __WFI();
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */


//这些中断函数其他地方有定义，可以在CubeMx生成的stm32f4xx_it.c里找到，弱定义在那里，用户可以在这里重定义覆盖掉它们实现自己的功能，注意函数名和参数要完全一样，否则就变成了新的函数，原来的中断处理函数还是会被调用，而不是我写的这个函数
/*  
__weak void DMA1_Stream7_IRQHandler(void)
__weak void USART3_IRQHandler(void)
__weak void USART1_IRQHandler(void)
__weak void USART6_IRQHandler(void)
*/
//USART1和USART6的接收完成回调函数，分别处理底盘状态帧和图传遥控器数据
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if(huart->Instance == USART6)
  {
    usart6_tuchuan_rx_callback();
    return;
  }

  if (huart->Instance == USART1)
  {
    /*
     * USART1接收完成回调（每次20字节）：
     * 1) 交给流解析器处理，支持跨包重组与错位重同步。
     * 2) 无论本次是否解析出有效帧，都立即重启下一次20字节接收。
     * 3) 这样可以避免一次异常后接收链路停住，导致上层长期读到0。
     */
    g_usart1_rx_irq_cnt++;
    (void)usart1_parse_chassis_status_stream(usart1_rx_buffer, USART1_FRAME_LENGTH);
    if (HAL_UART_Receive_IT(huart, usart1_rx_buffer, USART1_FRAME_LENGTH) != HAL_OK)
    {
      g_usart1_rx_restart_fail_cnt++;
    }
    return;
  }
}
//USART1和USART6的错误回调函数，分别处理底盘状态帧和图传遥控器数据的异常情况
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if(huart->Instance == USART6)
  {
    usart6_tuchuan_error_callback(huart->ErrorCode);
    return;
  }

  if (huart->Instance == USART1)
  {
    /*
     * USART1错误回调恢复：
     * 1) 清除ORE标志，防止串口因溢出卡死。
     * 2) 清空流解析器内部半帧缓存，避免残留脏数据影响后续校验。
     * 3) 立刻重启20字节接收，确保链路自动恢复。
     */
    __HAL_UART_CLEAR_OREFLAG(huart);
    usart1_reset_chassis_status_stream_parser();
    if (HAL_UART_Receive_IT(huart, usart1_rx_buffer, USART1_FRAME_LENGTH) != HAL_OK)
    {
      g_usart1_rx_restart_fail_cnt++;
    }
    return;
  }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
