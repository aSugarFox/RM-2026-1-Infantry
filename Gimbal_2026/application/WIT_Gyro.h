   /*
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     2026-4-12       zxy             陀螺仪IIC驱动
  * 这里有接收陀螺仪数据代码，通信使用的IIC
  * 上下C板使用的分别是HWT606（六轴）JY931（九轴）
  * 云台使用HWT606，因为云台不需要赚很多圈，不需要磁轴校准Yaw
  * 地盘使用的时JY931，因为地盘需要赚很多圈，磁轴校准Yaw比较方便，且地盘对磁场干扰较小
  * 实测下来JY931还是会有点飘，一方面是代码太烂，需要再优化。另一方面，也许底盘电机会有磁场干扰（可能性小）
  * 但是安装位置还是要远离磁场干扰源，不能放在电机旁边，不能放在电池旁边，不能放在大电流线缆旁边，不能放在无限下载器旁边
  * 不能放在大铁块旁边，不能放在大铁块旁边，不能放在大铁块旁边（重要的事情说三遍）
  *  
  ****************************(C) COPYRIGHT 2024****************************        
  */

#ifndef __WIT_GYRO_H
#define __IT_GYRO_H

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

/* WIT陀螺仪IIC地址 (根据实际型号调整) */
#define WIT_I2C_ADDR         0x50      // 7位IIC地址 (不含读写位)
#define WIT_READ_ADDR        ((WIT_I2C_ADDR << 1) | 1)  // 读地址
#define WIT_WRITE_ADDR       (WIT_I2C_ADDR << 1)        // 写地址

/* 寄存器地址定义 (根据WIT协议) */
#define WIT_ACCEL_REG        0x34      // 加速度计数据寄存器起始地址
#define WIT_GYRO_REG         0x37      // 陀螺仪数据寄存器起始地址
#define WIT_ANGLE_REG        0x3D      // 角度数据寄存器起始地址 (Roll, Pitch, Yaw)

#define WIT_MAG_REG          0x24      // 磁场数据寄存器起始地址
#define WIT_QUAT_REG         0x51      // 四元数数据寄存器起始地址
#define WIT_TEMP_REG         0x40      // 温度数据寄存器

/* 数据长度 */
#define WIT_AXIS_DATA_LEN     6        // 3轴数据长度 (3轴 * 2字节)
#define WIT_QUAT_DATA_LEN     8        // 四元数数据长度 (4轴 * 2字节)

/* 量程设置 */
#define WIT_ACCEL_RANGE       16       // 加速度计量程 ±16g
#define WIT_GYRO_RANGE        2000     // 陀螺仪量程 ±2000°/s
#define WIT_ANGLE_RANGE       180      // 角度量程 ±180°

/* WIT陀螺仪数据结构体 */
typedef struct {
    float accel[3];        // 加速度 X, Y, Z (g)
    float gyro[3];         // 角速度 X, Y, Z (°/s)
    float angle[3];        // 角度 Roll, Pitch, Yaw (°)
    float mag[3];          // 磁场 X, Y, Z (uT)
    float quat[4];         // 四元数 q0, q1, q2, q3
    int16_t temperature;    // 温度 (0.1°C)
    uint32_t last_update;   // 最后更新时间 (ms)
    uint8_t data_ready;     // 数据就绪标志
} WIT_Data_t;

/* 函数声明 */
uint8_t WIT_IIC_Init(I2C_HandleTypeDef *hi2c);
uint8_t WIT_IIC_ReadReg(uint8_t reg, uint8_t *data, uint16_t len);
uint8_t WIT_IIC_WriteReg(uint8_t reg, uint8_t data);
uint8_t WIT_IIC_ReadAccel(float *accel);
uint8_t WIT_IIC_ReadGyro(float *gyro);
uint8_t WIT_IIC_ReadAngle(float *angle);
uint8_t WIT_IIC_ReadMag(float *mag);
uint8_t WIT_IIC_ReadQuat(float *quat);
int16_t WIT_IIC_ReadTemp(void);
uint8_t WIT_IIC_ReadAll(WIT_Data_t *data);

/* 数据获取函数 */
float WIT_IIC_GetYaw(void);
float WIT_IIC_GetPitch(void);
float WIT_IIC_GetRoll(void);
WIT_Data_t* WIT_IIC_GetDataPtr(void);


#endif
