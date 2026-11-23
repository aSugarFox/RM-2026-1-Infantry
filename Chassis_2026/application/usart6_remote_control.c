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

  #include "usart6_remote_control.h"
#include "infantry_imu.h"
  #include "bsp_usart.h"
  #include "string.h"
  #include "bsp_led.h" // Include the header file where BSP_LED_SHOW is declared
  #include "WIT_Gyro.h"
  #include "Control_Ghassis.h"
  float yaw_angle = 0.0f;
  float last_angle = 0.0f;
  extern float angle[3];
  extern UART_HandleTypeDef huart6;
  static uint8_t usart1_rx_buf[2][USART1_RX_BUF_NUM];
  static uint8_t usart1_tx_buf[USART1_FRAME_LENGTH];
  extern DMA_HandleTypeDef hdma_usart1_tx;
  
  static RC_ctrl_t usart1_rc_ctrl;
  static volatile uint16_t usart1_keyboard_value = 0u;
  static volatile usart1_input_mode_e usart1_input_mode = USART1_INPUT_NONE;
  static volatile uint32_t usart1_input_mode_update_tick = 0u;
  static volatile uint32_t usart1_remote_update_tick = 0u;
  static volatile uint32_t usart1_keyboard_update_tick = 0u;
  static volatile float usart1_gimbal_gyro_comp_speed_rpm = 0.0f;
  static volatile uint8_t usart1_gimbal_gyro_enable = 0u;
  static volatile uint32_t usart1_gimbal_gyro_update_tick = 0u;
  static volatile usart1_gimbal_status_t usart1_gimbal_status = {0};
  static uint8_t *current_buf;
  static uint8_t usart1_stream_frame[USART1_FRAME_LENGTH];
  static uint16_t usart1_stream_length;
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
      rc_ctrl->rc.ch[4] = sbus_buf[16] | (sbus_buf[17] << 8);
  
      rc_ctrl->rc.ch[0] -= RC_CH_VALUE_OFFSET;
      rc_ctrl->rc.ch[1] -= RC_CH_VALUE_OFFSET;
      rc_ctrl->rc.ch[2] -= RC_CH_VALUE_OFFSET;
      rc_ctrl->rc.ch[3] -= RC_CH_VALUE_OFFSET;
      rc_ctrl->rc.ch[4] -= RC_CH_VALUE_OFFSET;
  }
  // 从陀螺仪数据中提取角度信息
  float get_angle_from_gyro(uint8_t* data) 
  {
      float angle;
      if(data == NULL) {
          return 0.0f;
      }
  
      memcpy(&angle, data, sizeof(float));
      
      // if(angle > 180.0f || angle < -180.0f || RC_abs(angle) < 0.01f) // 角度值异常，可能是数据错误 abs
      // {
      //     // yaw_angle = 999.0f; // 设置一个特殊值表示数据异常，供其他模块检测
      //     return last_angle; // 返回默认值，或根据需要进行错误处理
      // }
      // else 
      // {
      //     // yaw_angle = angle; // 将角度值存储到全局变量中，供其他模块使用
      //     last_angle = angle; // 更新上一次的角度值
      //     return angle;
      // }
      return angle;
  }
  static void usart1_parse_wit_frame(const uint8_t *frame)
  {
      if (frame == NULL)
      {
          return;
      }

      yaw_angle = get_angle_from_gyro((uint8_t *)&frame[1]);
      memcpy(&usart1_gimbal_gyro_comp_speed_rpm, &frame[5], sizeof(float));
      usart1_gimbal_gyro_enable = frame[9];
      usart1_gimbal_gyro_update_tick = HAL_GetTick();
  }

  static void usart1_parse_gimbal_status_frame(const uint8_t *frame)
  {
      if (frame == NULL)
      {
          return;
      }

      usart1_gimbal_status.flags = frame[1];
      usart1_gimbal_status.fault = frame[2];
      usart1_gimbal_status.sequence = frame[3];
      memcpy((void *)&usart1_gimbal_status.yaw_angle_deg, &frame[4], sizeof(float));
      memcpy((void *)&usart1_gimbal_status.yaw_speed_rpm, &frame[8], sizeof(float));
      memcpy((void *)&usart1_gimbal_status.yaw_encoder, &frame[12], sizeof(uint16_t));
      usart1_gimbal_status.update_tick = HAL_GetTick();

      /* yaw_angle is the relative angle owned by the control loop, not this ISR. */
      usart1_gimbal_gyro_comp_speed_rpm = usart1_gimbal_status.yaw_speed_rpm;
      usart1_gimbal_gyro_enable =
          (uint8_t)((usart1_gimbal_status.flags & INFANTRY_GIMBAL_FLAG_READY) != 0u);
      usart1_gimbal_gyro_update_tick = usart1_gimbal_status.update_tick;
  }
    
  /**
  * @brief          校验接收到的数据帧
  * @param[in]      buf: 接收到的数据缓冲区
  * @param[in]      len: 数据长度
  * @retval         0: 校验成功, 1: 校验失败
  */
  static uint8_t usart1_data_check(const uint8_t *buf, uint16_t len, uint8_t header)
  {
      uint8_t checksum = 0;
      uint16_t i;

      if (buf == NULL || len != USART1_FRAME_LENGTH)
      {
          return 1;
      }

      // 检查帧头
      if (buf[0] != header)
      {
          return 1;
      }

      // 校验和检查
      for (i = 0; i < 19u; i++)
      {
          checksum += buf[i];
      }

      if (checksum != buf[19])
      {
          return 1;
      }

      return 0;
  }
  
  static uint8_t usart1_packet_checksum_ok(const uint8_t *buf, uint16_t len)
  {
      uint8_t checksum = 0;
      uint16_t i;
  
      if (buf == NULL || len != USART1_FRAME_LENGTH)
      {
          return 0;
      }
  
      for (i = 0; i < (USART1_FRAME_LENGTH - 1U); i++)
      {
          checksum += buf[i];
      }
  
      return (checksum == buf[USART1_FRAME_LENGTH - 1U]) ? 1U : 0U;
  }
  
  static void usart1_parse_frame(uint8_t *frame)
  {
      if (frame == NULL)
      {
          return;
      }

      if ((frame[0] == USART1_FRAME_HEADER_RC) &&
          (usart1_data_check(frame, USART1_FRAME_LENGTH, USART1_FRAME_HEADER_RC) == 0))
      {
          sbus_to_rc_local(&frame[1], &usart1_rc_ctrl);
          usart1_rc_ctrl.key.v = 0u;
          usart1_input_mode = USART1_INPUT_RC;
          usart1_input_mode_update_tick = HAL_GetTick();
          usart1_remote_update_tick = usart1_input_mode_update_tick;
      }
      else if ((frame[0] == USART1_FRAME_HEADER_KEYBOARD) &&
               (usart1_data_check(frame, USART1_FRAME_LENGTH, USART1_FRAME_HEADER_KEYBOARD) == 0))
      {
          usart1_parse_gimbal_status_frame(frame);
      }
      else if ((frame[0] == USART1_FRAME_HEADER_WIT) &&
               usart1_packet_checksum_ok(frame, USART1_FRAME_LENGTH))
      {
          usart1_parse_wit_frame(frame);
      }
  }
  
  static void usart1_parse_frames(uint8_t *buf, uint16_t len)
  {
      uint16_t i;
      if (buf == NULL) return;

      /* Keep partial frames across UART IDLE boundaries; resync after noise. */
      for (i = 0u; i < len; i++)
      {
          if ((usart1_stream_length == 0u) &&
              (buf[i] != USART1_FRAME_HEADER_RC) &&
              (buf[i] != USART1_FRAME_HEADER_KEYBOARD) &&
              (buf[i] != USART1_FRAME_HEADER_WIT)) continue;
          usart1_stream_frame[usart1_stream_length++] = buf[i];
          if (usart1_stream_length < USART1_FRAME_LENGTH) continue;

          if (((usart1_stream_frame[0] == USART1_FRAME_HEADER_RC) ||
               (usart1_stream_frame[0] == USART1_FRAME_HEADER_KEYBOARD) ||
               (usart1_stream_frame[0] == USART1_FRAME_HEADER_WIT)) &&
              usart1_packet_checksum_ok(usart1_stream_frame, USART1_FRAME_LENGTH))
          {
              usart1_parse_frame(usart1_stream_frame);
              usart1_stream_length = 0u;
          }
          else
          {
              memmove(usart1_stream_frame, &usart1_stream_frame[1], USART1_FRAME_LENGTH - 1u);
              usart1_stream_length = USART1_FRAME_LENGTH - 1u;
          }
      }
  }
  
  /**
  * @brief          从转发帧中提取SBUS数据
  * @param[in]      rx_buf: 接收缓冲区
  * @param[out]     sbus_buf: SBUS数据输出缓冲区
  * @retval         0: 成功, 1: 失败
  */
  static uint8_t extract_sbus_from_frame(uint8_t *rx_buf, uint8_t *sbus_buf)
  {
      if (rx_buf[0] != USART1_FRAME_HEADER_RC)
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
      usart1_stream_length = 0u;
      usart1_tx_dma_init();
      USART1_RC_Init(usart1_rx_buf[0],usart1_rx_buf[1],USART1_RX_BUF_NUM);
      usart1_keyboard_value = 0u;
      usart1_input_mode = USART1_INPUT_NONE;
      usart1_input_mode_update_tick = HAL_GetTick();
      usart1_remote_update_tick = 0u;
      usart1_keyboard_update_tick = 0u;
      usart1_gimbal_gyro_comp_speed_rpm = 0.0f;
      usart1_gimbal_gyro_enable = 0u;
      usart1_gimbal_gyro_update_tick = HAL_GetTick();
      memset((void *)&usart1_gimbal_status, 0, sizeof(usart1_gimbal_status));
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

  uint16_t get_usart1_keyboard_value(void)
  {
      return usart1_keyboard_value;
  }

  usart1_input_mode_e get_usart1_input_mode(void)
  {
      return usart1_input_mode;
  }

  uint32_t get_usart1_input_mode_update_tick(void)
  {
      return usart1_input_mode_update_tick;
  }

  uint32_t get_usart1_remote_update_tick(void)
  {
      return usart1_remote_update_tick;
  }

  uint8_t usart1_remote_is_online(void)
  {
      return (uint8_t)((usart1_remote_update_tick != 0u) &&
          ((uint32_t)(HAL_GetTick() - usart1_remote_update_tick) <= INFANTRY_LINK_TIMEOUT_MS) &&
          (usart1_RC_data_is_error() == 0u));
  }

  uint32_t get_usart1_keyboard_update_tick(void)
  {
      return usart1_keyboard_update_tick;
  }

  float get_usart1_gimbal_yaw_angle_deg(void)
  {
      return yaw_angle;
  }

  float get_usart1_gimbal_gyro_comp_speed_rpm(void)
  {
      return usart1_gimbal_gyro_comp_speed_rpm;
  }

  uint8_t get_usart1_gimbal_gyro_enable(void)
  {
      return usart1_gimbal_gyro_enable;
  }

  uint32_t get_usart1_gimbal_gyro_update_tick(void)
  {
      return usart1_gimbal_gyro_update_tick;
  }

  void get_usart1_gimbal_status_snapshot(usart1_gimbal_status_t *status)
  {
      /* Keep encoder, online flags and reception time from the same RX frame. */
      uint32_t irq_mask = __get_PRIMASK();
      __disable_irq();
      *status = usart1_gimbal_status;
      __set_PRIMASK(irq_mask);
  }

  const usart1_gimbal_status_t *get_usart1_gimbal_status(void)
  {
      return (const usart1_gimbal_status_t *)&usart1_gimbal_status;
  }

  uint8_t get_usart1_gimbal_imu_ready(void)
  {
      return (uint8_t)((usart1_gimbal_status.flags & INFANTRY_GIMBAL_FLAG_IMU_READY) != 0u);
  }

  uint8_t get_usart1_gimbal_ready(void)
  {
      return (uint8_t)((usart1_gimbal_status.flags & INFANTRY_GIMBAL_FLAG_READY) != 0u);
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
          __HAL_UART_DISABLE(&huart1);
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
          __HAL_UART_ENABLE(&huart1);
  }
  
  void usart1_slove_data_error(void)
  {
      usart1_slove_RC_lost();
  }
  
  
  
  /**
  * @brief          串口1中断处理函数
  * @param[in]      none
  * @retval         none
  */
  // void USART1_IRQHandler(void)
  // {
  //     if(huart1.Instance->SR & UART_FLAG_RXNE)//接收到数据
  //     {
  //         __HAL_UART_CLEAR_PEFLAG(&huart1);
  //     }
  //     else if(USART1->SR & UART_FLAG_IDLE)
  //     {
  //         static uint16_t this_time_rx_len = 0;
      
  //         __HAL_UART_CLEAR_PEFLAG(&huart1);
      
  //         if ((hdma_usart1_rx.Instance->CR & DMA_SxCR_CT) == RESET)
  //         {
  //             /* 当前使用缓冲区0 */
      
  //             // 失效DMA
  //             __HAL_DMA_DISABLE(&hdma_usart1_rx);
      
  //             // 获取接收数据长度
  //             this_time_rx_len = USART1_RX_BUF_NUM - hdma_usart1_rx.Instance->NDTR;
      
  //             // 重新设定数据长度
  //             hdma_usart1_rx.Instance->NDTR = USART1_RX_BUF_NUM;
      
  //             // 设定缓冲区1
  //             hdma_usart1_rx.Instance->CR |= DMA_SxCR_CT;
              
  //             // 使能DMA
  //             __HAL_DMA_ENABLE(&hdma_usart1_rx);
      
  //             if(this_time_rx_len == USART1_FRAME_LENGTH)  // 使用USART1帧长度
  //             {
  //                     // 检查帧头是否为0xA6
  //                 if(usart1_rx_buf[0][0] == 0xA6)
  //                 {
  //                         sbus_to_rc_local(&usart1_rx_buf[0][1], &usart1_rc_ctrl);
  //                 }
  //                 //串口接收上板陀螺仪数据
  //                 else if(usart1_rx_buf[0][0] == 0xA5)
  //                 {
  //                     yaw_angle = get_angle_from_gyro(&usart1_rx_buf[0][1]);
  //                 }
  
  //             }
  //         }
  //         else
  //         {
  //             /* 当前使用缓冲区1 */
  //             // 失效DMA
  //             __HAL_DMA_DISABLE(&hdma_usart1_rx);
      
  //             // 获取接收数据长度
  //             this_time_rx_len = USART1_RX_BUF_NUM - hdma_usart1_rx.Instance->NDTR;
      
  //             // 重新设定数据长度
  //             hdma_usart1_rx.Instance->NDTR = USART1_RX_BUF_NUM;
      
  //             // 设定缓冲区0
  //             hdma_usart1_rx.Instance->CR &= ~(DMA_SxCR_CT);  // 使用正确的DMA句柄
              
  //             // 使能DMA
  //             __HAL_DMA_ENABLE(&hdma_usart1_rx);
      
  //             if(this_time_rx_len == USART1_FRAME_LENGTH)
  //             {
  //                     // 检查帧头是否为0xA6
  //                 if(usart1_rx_buf[1][0] == 0xA6)
  //                 {
  //                         // ? 正确的解码：从第1字节开始
  //                         sbus_to_rc_local(&usart1_rx_buf[1][1], &usart1_rc_ctrl);
  //                 }
  //                 else if(usart1_rx_buf[1][0] == 0xA5)
  //                 {
  //                     yaw_angle = get_angle_from_gyro(&usart1_rx_buf[1][1]);
  //                 }
  //             }
  //         }
  //     }
  // }
  
  
  void USART1_IRQHandler(void)
  {
      /* --- 1. 错误自愈：防止发送/接收冲突导致的 ORE 溢出错误 --- */
      if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_ORE) != RESET)
      {
          usart1_stream_length = 0u;
          __HAL_UART_CLEAR_OREFLAG(&huart1); // 清除溢出错误，否则接收会彻底锁死
          
          // 发生错误后，尝试强制恢复 DMA 接收
          __HAL_DMA_DISABLE(&hdma_usart1_rx);
          while (hdma_usart1_rx.Instance->CR & DMA_SxCR_EN) {}
          hdma_usart1_rx.Instance->NDTR = USART1_RX_BUF_NUM;
          __HAL_DMA_ENABLE(&hdma_usart1_rx);
      }
  
      /* --- 2. 空闲中断 IDLE 处理 --- */
      if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_IDLE) != RESET)
      {
          /* 【关键】严格执行清除序列：先读 SR，再读 DR */
          volatile uint32_t temp;
          temp = huart1.Instance->SR; // 第一步
          temp = huart1.Instance->DR; // 第二步，标志位此时才真正清除
  
          uint16_t this_time_rx_len = 0;
          // uint8_t *current_buf;
  
          // 获取当前 DMA 使用的是哪个缓冲区 (双缓冲模式)
          uint8_t current_mem = (hdma_usart1_rx.Instance->CR & DMA_SxCR_CT) ? 1 : 0;
  
          // 暂时失效 DMA 以便读取数据长度并重置
          __HAL_DMA_DISABLE(&hdma_usart1_rx);
          while (hdma_usart1_rx.Instance->CR & DMA_SxCR_EN) {}
  
          // 计算本次接收到的字节数
          this_time_rx_len = USART1_RX_BUF_NUM - hdma_usart1_rx.Instance->NDTR;
          current_buf = usart1_rx_buf[current_mem];
  
          // 重新设定长度并切换到另一个缓冲区 (Ping-Pong 缓冲)
          hdma_usart1_rx.Instance->NDTR = USART1_RX_BUF_NUM;
          if (current_mem == 0)
              hdma_usart1_rx.Instance->CR |= DMA_SxCR_CT;    // 指向 buf[1]
          else
              hdma_usart1_rx.Instance->CR &= ~(DMA_SxCR_CT); // 指向 buf[0]
  
          // 重新使能 DMA，开始下一轮接收
          __HAL_DMA_ENABLE(&hdma_usart1_rx);
  
          /* --- 3. 数据逻辑解析（支持一次空闲中收到多帧） --- */
          usart1_parse_frames(current_buf, this_time_rx_len);
      }
  }
  
  
  void Data_chuli(void){
      // 兼容保留：当前已在 USART1 中断中完成解析
  }
  
  
  
  void data_to_usart1(const usart1_tx_status_t *status)
  {
      uint8_t i;
      static uint32_t last_tx_tick = 0;
      static uint32_t last_motion_tick = 0;
      static uint32_t last_imu_tick = 0;
      uint32_t now_tick = HAL_GetTick();

      if (status == NULL)
      {
          return;
      }

      // 适度限流，避免同链路收发互相挤占
      if ((now_tick - last_tx_tick) < INFANTRY_STATUS_PERIOD_MS)
      {
          return;
      }

      // DMA stream still active -> previous packet not finished
      if ((hdma_usart1_tx.Instance->CR & DMA_SxCR_EN) ||
          ((USART1->SR & USART_SR_TC) == 0u))
      {
          return;
      }

      memset(usart1_tx_buf, 0, sizeof(usart1_tx_buf));
      /* A9 diagnostics: ready/error/sequence, YPR in centidegrees,
       * horizontal yaw rate (float RPM), source tick, reserved, checksum.
       * Keep this separate from A5, which carries control/safety status. */
      if ((uint32_t)(now_tick - last_imu_tick) >= INFANTRY_CHASSIS_IMU_PERIOD_MS)
      {
          int16_t angles_cdeg[3];
          float yaw_rate_rpm = Infantry_IMU_GetGyroZRpm();
          angles_cdeg[0] = (int16_t)(Infantry_IMU_GetYawDeg() * 100.0f);
          angles_cdeg[1] = (int16_t)(Infantry_IMU_GetPitchDeg() * 100.0f);
          angles_cdeg[2] = (int16_t)(Infantry_IMU_GetRollDeg() * 100.0f);
          usart1_tx_buf[0] = INFANTRY_FRAME_CHASSIS_IMU;
          usart1_tx_buf[1] = Infantry_IMU_IsReady();
          usart1_tx_buf[2] = Infantry_IMU_GetError();
          usart1_tx_buf[3] = status->sequence;
          memcpy(&usart1_tx_buf[4], angles_cdeg, sizeof(angles_cdeg));
          memcpy(&usart1_tx_buf[10], &yaw_rate_rpm, sizeof(yaw_rate_rpm));
          memcpy(&usart1_tx_buf[14], &now_tick, sizeof(now_tick));
          last_imu_tick = now_tick;
      }
      else if ((uint32_t)(now_tick - last_motion_tick) >= INFANTRY_CHASSIS_MOTION_PERIOD_MS)
      {
          int16_t targets[4];
          int16_t relative_cdeg = (int16_t)(yaw_angle * 100.0f);
          usart1_tx_buf[0] = INFANTRY_FRAME_CHASSIS_MOTION;
          memcpy(&usart1_tx_buf[1], &relative_cdeg, sizeof(relative_cdeg));
          Chassis_GetWheelSpeedTargets(targets);
          memcpy(&usart1_tx_buf[3], targets, sizeof(targets));
          for (i = 0u; i < 4u; i++)
          {
              int16_t rpm = get_chassis_motor_measure_point(i)->speed_rpm;
              memcpy(&usart1_tx_buf[11u + 2u * i], &rpm, sizeof(rpm));
          }
          last_motion_tick = now_tick;
      }
      else
      {
          usart1_tx_buf[0] = USART1_FRAME_HEADER_WIT;
          usart1_tx_buf[1] = status->flags;
          usart1_tx_buf[2] = status->fault;
          usart1_tx_buf[3] = status->sequence;
          memcpy(&usart1_tx_buf[4], &status->chassis_gyro_speed_rpm, sizeof(float));
          usart1_tx_buf[8] = (uint8_t)(status->shooter_17mm_heat & 0xFFu);
          usart1_tx_buf[9] = (uint8_t)((status->shooter_17mm_heat >> 8) & 0xFFu);
          usart1_tx_buf[10] = (uint8_t)(status->shooter_17mm_heat_limit & 0xFFu);
          usart1_tx_buf[11] = (uint8_t)((status->shooter_17mm_heat_limit >> 8) & 0xFFu);
          usart1_tx_buf[12] = (uint8_t)(status->shooter_barrel_cooling_value & 0xFFu);
          usart1_tx_buf[13] = (uint8_t)((status->shooter_barrel_cooling_value >> 8) & 0xFFu);
          memcpy(&usart1_tx_buf[14], &status->remote_ch2, sizeof(int16_t));
          memcpy(&usart1_tx_buf[16], &status->remote_ch3, sizeof(int16_t));
          usart1_tx_buf[18] = status->remote_left_switch;
      }

      usart1_tx_buf[19] = 0;
      for(i = 0; i < 19; i++)
      {
          usart1_tx_buf[19] += usart1_tx_buf[i];
      }

      usart1_tx_dma_enable(usart1_tx_buf, sizeof(usart1_tx_buf));
      last_tx_tick = now_tick;
  }
  
  uint8_t usart1_tx_is_busy(void)
  {
      return (hdma_usart1_tx.Instance->CR & DMA_SxCR_EN) ? 1U : 0U;
  }
  
  void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
  {
      (void)huart;
  }
  
  
  
















