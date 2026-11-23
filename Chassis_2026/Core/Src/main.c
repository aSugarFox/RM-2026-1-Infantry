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
#include "bsp_led.h"
#include "bsp_delay.h"
#include "bsp_usart.h"
#include "remote_control.h"
#include "string.h"
#include <stdio.h> 
 #include "Control_Ghassis.h"
#include <math.h>
#include "arm_math.h"
#include "BMI088driver.h"
#include "state_machine.h"
#include "uart6_rmul.h"
#include "usart6_remote_control.h"
#include "WIT_Gyro.h"
#include "usbd_cdc_if.h"
#include "bsp_can.h"
#include "infantry_config.h"
#include "infantry_imu.h"
#include "infantry_indicator.h"
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
state_machine_t g_state_machine;
extern float yaw_angle; 
float angle[3];
extern float yaw_zero; // 用于存储初始Yaw角的全局变量
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void uart6_data_ready_callback(void);
void ui_auto_init(void);
void ui_auto_task(void);

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
  delay_init(); /* Use the configured system clock for BMI088 microsecond delays. */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM5_Init();
  MX_TIM4_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_USART1_UART_Init();
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
  BSP_LED_Init();                                       // 初始化LED
  Infantry_Indicator_Init();
  BSP_CAN_Init();
  chassis_init(&chassis_move);                          // 初始化底盘控制  

  usart1_tx_dma_init();                                 // 初始化USART1的DMA传输，用于发送数据到上位机
  usart1_remote_control_init();                         // 初始化USART1接收遥控器数据 
  // char buffer[100];                                     // 用于存储调试信息的缓冲区
  uint32_t referee_dbg_last_tick = 0;                   // 上次发送调试信息的时间戳  
  
  state_machine_init(&g_state_machine);                 // 初始化状态机
  
  uart6_rmul_init(&huart6);                             // 初始化UART6接收Referee系统数据
  uart6_rmul_set_callback(uart6_data_ready_callback);   // 设置UART6数据接收完成后的回调函数
  (void)Infantry_IMU_Init();
  ui_auto_init();                                       // 初始化UI自动化测试模块，用开源自动生成的UI
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    uint32_t control_cycle_tick = HAL_GetTick();
    static uint32_t imu_retry_tick = 0u;
    static uint8_t status_sequence = 0u;

    Infantry_IMU_Poll();
    if ((Infantry_IMU_IsReady() == 0u) &&
        ((uint32_t)(HAL_GetTick() - imu_retry_tick) >= 1000u))
    {
      chassis_output_clear();
      Chassis_ResetSpeedPID();
      chassis_output_send(); /* Stop motors before blocking for IMU initialization. */
      imu_retry_tick = HAL_GetTick();
      (void)Infantry_IMU_Init();
    }

    //用于调试的代码，获取底盘电机数据和遥控器数据
    // const motor_measure_t *motor1 = get_chassis_motor_measure_point(0);
    // const motor_measure_t *motor2 = get_chassis_motor_measure_point(1);
    // const motor_measure_t *motor3 = get_chassis_motor_measure_point(2);
    // const motor_measure_t *motor4 = get_chassis_motor_measure_point(3);
    // const RC_ctrl_t *rc_data = get_remote_control_point();     
    const RC_ctrl_t *rc_ctrl = get_usart1_remote_control_point();
    // shoot_data_t* shoot = uart6_get_shoot_data();
    robot_performance_t* perf = uart6_get_robot_perf();
    power_heat_data_t* power = uart6_get_power_heat();
    usart1_tx_status_t usart1_status;
    uint16_t current_hp = perf->current_HP;
    // // uart6_rmul_poll();

    // if ((HAL_GetTick() - referee_dbg_last_tick) >= 100U) {
    //   int msg_len = snprintf(buffer, sizeof(buffer),"606:%.2f 931:%.2f err:%.2f\r\n",
    //                            yaw_angle,
    //                            Get_Yaw_Angle_Data(),
    //                             yaw_zero);
    //   if (msg_len > 0) {
    //     (void)CDC_Transmit_FS((uint8_t *)buffer, (uint16_t)msg_len);// 通过USB CDC发送调试信息到PC
    //   }
    //   referee_dbg_last_tick = HAL_GetTick();
    // }

    // 根据当前HP值调整底盘速度，HP较低时增加速度提升以增强机动性实际没用上，因为没用超级电容回超功率，使用我直接改了速度上限的值
    if ((current_hp != 0u) && (current_hp <= 80u))
    {
      Chassis_Set_Low_Hp_Speed_Boost(1u);
    }
    else
    {
      Chassis_Set_Low_Hp_Speed_Boost(0u);
    }
    
    state_machine_update(&g_state_machine, rc_ctrl);
    
      // 执行状态机逻辑，根据当前状态和输入控制底盘
    state_machine_execute(&g_state_machine);
    //发送底盘数据到电机
    chassis_output_send();
    // 发送数据到云台，包括陀螺仪数据和射击系统的热量数据，供上位机显示和UI自动化测试使用
    memset(&usart1_status, 0, sizeof(usart1_status));
    usart1_status.flags = 0u;
    if (Infantry_IMU_IsReady() != 0u) usart1_status.flags |= INFANTRY_CHASSIS_FLAG_IMU_READY;
    if (state_machine_is_enabled(&g_state_machine) != 0u) usart1_status.flags |= INFANTRY_CHASSIS_FLAG_ENABLED;
    if (state_machine_small_gyro_active(&g_state_machine) != 0u) usart1_status.flags |= INFANTRY_CHASSIS_FLAG_SMALL_GYRO;
    if (usart1_remote_is_online() != 0u) usart1_status.flags |= INFANTRY_CHASSIS_FLAG_REMOTE_READY;
    usart1_status.fault = g_state_machine.fault;
    usart1_status.sequence = status_sequence++;
    usart1_status.chassis_gyro_speed_rpm = Infantry_IMU_GetGyroZRpm();
    usart1_status.shooter_17mm_heat = power->shooter_17mm_heat;
    usart1_status.shooter_17mm_heat_limit = perf->shooter_barrel_heat_limit;
    usart1_status.shooter_barrel_cooling_value = perf->shooter_barrel_cooling_value;
    usart1_status.remote_ch2 = rc_ctrl->rc.ch[2];
    usart1_status.remote_ch3 = rc_ctrl->rc.ch[3];
    usart1_status.remote_left_switch = (uint8_t)rc_ctrl->rc.s[RC_S_LEFT];
    data_to_usart1(&usart1_status);
    //UI发送
    ui_auto_task(); 
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

/* ���ݸ��»ص� */
void uart6_data_ready_callback(void)
{
    /* ������?���ݸ���??*/
    if (uart6_is_robot_hp_updated()) {
        robot_hp_t *hp = uart6_get_robot_hp();
        /* ����Ѫ���仯�������ж��Ƿ���Ҫ���� */
        if (hp->ally_3_robot_HP < 500) {
            // Ѫ�����ͣ��������˲���
        }
    }
    
    if (uart6_is_power_heat_updated()) {
        power_heat_data_t *ph = uart6_get_power_heat();
        robot_performance_t *perf = uart6_get_robot_perf();
        /* Use heat limit from 0x0201 and current heat from 0x0202 */
        if ((perf->shooter_barrel_heat_limit > 0U) &&
            ((float)ph->shooter_17mm_heat > ((float)perf->shooter_barrel_heat_limit * 0.8f))) {
            // Heat is high, reduce fire rate
        }
    }
    
    if (uart6_is_robot_pos_updated()) {
        robot_pos_t *pos = uart6_get_robot_pos();
        /* ����λ����Ϣ�������㷨 */
    }
    
    /* ���������� */
    custom_client_data_t *key = uart6_get_custom_data();
    if (key->key_value & KEY_W) {
        // ǰ��ָ��
    }
    if (key->key_value & KEY_SHIFT) {
        // С����ģʽ
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








