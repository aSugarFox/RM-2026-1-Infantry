   /*
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     2026-4-12       zxy             Usb给视觉发送陀螺仪数据任务
  *  这里是通过USB串口向视觉系统发送陀螺仪数据，同时监控陀螺仪的状态和温度，并进行温度控制。
  ****************************(C) COPYRIGHT 2024****************************        
  */

#ifndef IMU_USB_DEBUG_H
#define IMU_USB_DEBUG_H

void imu_usb_debug_init(void);
void imu_usb_debug_poll(void);

#endif
