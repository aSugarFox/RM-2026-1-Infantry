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
#include "WIT_Gyro.h"
/* 私有变量 */
static I2C_HandleTypeDef *wit_i2c = NULL;
static WIT_Data_t wit_data = {0};
// static uint8_t rx_buffer[32];

/**
 * @brief  字节数组转int16（小端模式 - WIT使用小端模式）
 * @param  data: 2字节数组 [低字节, 高字节]
 * @retval int16_t值
 */
static int16_t BytesToShort(uint8_t data[])
{
    return (int16_t)(((uint16_t)data[1] << 8) | data[0]);
}

/**
 * @brief  WIT陀螺仪IIC初始化
 * @param  hi2c: I2C句柄指针
 * @retval 1:成功, 0:失败
 */
uint8_t WIT_IIC_Init(I2C_HandleTypeDef *hi2c)
{
    if(hi2c == NULL) return 0;
    
    wit_i2c = hi2c;
    
    /* 清空数据结构 */
    memset(&wit_data, 0, sizeof(WIT_Data_t));
    
    /* 尝试读取设备ID或版本号来验证连接 */
    uint8_t test_data[2];
    if(HAL_I2C_Mem_Read(wit_i2c, WIT_READ_ADDR, 0x00, I2C_MEMADD_SIZE_8BIT, 
                        test_data, 1, 100) == HAL_OK) {
        wit_data.data_ready = 1;
        return 1;
    }
    
    return 0;
}

/**
 * @brief  读取WIT陀螺仪寄存器
 * @param  reg: 寄存器地址
 * @param  data: 数据缓冲区
 * @param  len: 读取长度
 * @retval 1:成功, 0:失败
 */
uint8_t WIT_IIC_ReadReg(uint8_t reg, uint8_t *data, uint16_t len)
{
    if(wit_i2c == NULL || data == NULL || len == 0) return 0;
    
    if(HAL_I2C_Mem_Read(wit_i2c, WIT_READ_ADDR, reg, I2C_MEMADD_SIZE_8BIT, 
                        data, len, 100) == HAL_OK) {
        return 1;
    }
    return 0;
}

/**
 * @brief  写入WIT陀螺仪寄存器
 * @param  reg: 寄存器地址
 * @param  data: 要写入的数据
 * @retval 1:成功, 0:失败
 */
uint8_t WIT_IIC_WriteReg(uint8_t reg, uint8_t data)
{
    if(wit_i2c == NULL) return 0;
    
    if(HAL_I2C_Mem_Write(wit_i2c, WIT_WRITE_ADDR, reg, I2C_MEMADD_SIZE_8BIT, 
                        &data, 1, 100) == HAL_OK) {
        return 1;
    }
    return 0;
}

/**
 * @brief  读取加速度计数据
 * @param  accel: 输出加速度数组 [3]
 * @retval 1:成功, 0:失败
 */
uint8_t WIT_IIC_ReadAccel(float *accel)
{
    uint8_t buf[6];
    int16_t raw[3];
    
    if(accel == NULL) return 0;
    
    if(!WIT_IIC_ReadReg(WIT_ACCEL_REG, buf, 6)) return 0;
    
    raw[0] = BytesToShort(&buf[0]);
    raw[1] = BytesToShort(&buf[2]);
    raw[2] = BytesToShort(&buf[4]);
    
    accel[0] = (float)raw[0] / 32768.0f * WIT_ACCEL_RANGE;
    accel[1] = (float)raw[1] / 32768.0f * WIT_ACCEL_RANGE;
    accel[2] = (float)raw[2] / 32768.0f * WIT_ACCEL_RANGE;
    
    return 1;
}

/**
 * @brief  读取陀螺仪数据
 * @param  gyro: 输出角速度数组 [3]
 * @retval 1:成功, 0:失败
 */
uint8_t WIT_IIC_ReadGyro(float *gyro)
{
    uint8_t buf[6];
    int16_t raw[3];
    
    if(gyro == NULL) return 0;
    
    if(!WIT_IIC_ReadReg(WIT_GYRO_REG, buf, 6)) return 0;
    
    raw[0] = BytesToShort(&buf[0]);
    raw[1] = BytesToShort(&buf[2]);
    raw[2] = BytesToShort(&buf[4]);
    
    gyro[0] = (float)raw[0] / 32768.0f * WIT_GYRO_RANGE;
    gyro[1] = (float)raw[1] / 32768.0f * WIT_GYRO_RANGE;
    gyro[2] = (float)raw[2] / 32768.0f * WIT_GYRO_RANGE;
    
    return 1;
}

/**
 * @brief  读取角度数据 (Roll, Pitch, Yaw)
 * @param  angle: 输出角度数组 [3]
 * @retval 1:成功, 0:失败
 */
uint8_t WIT_IIC_ReadAngle(float *angle)
{
    uint8_t buf[6];
    int16_t raw[3];
    
    if(angle == NULL) return 0;
    
    if(!WIT_IIC_ReadReg(WIT_ANGLE_REG, buf, 6)) return 0;
    
    raw[0] = BytesToShort(&buf[0]);  // Roll
    raw[1] = BytesToShort(&buf[2]);  // Pitch
    raw[2] = BytesToShort(&buf[4]);  // Yaw
    
    angle[0] = (float)raw[0] / 32768.0f * WIT_ANGLE_RANGE;
    angle[1] = (float)raw[1] / 32768.0f * WIT_ANGLE_RANGE;
    angle[2] = (float)raw[2] / 32768.0f * WIT_ANGLE_RANGE;
    
    return 1;
}

/**
 * @brief  读取磁场数据
 * @param  mag: 输出磁场数组 [3]
 * @retval 1:成功, 0:失败
 */
uint8_t WIT_IIC_ReadMag(float *mag)
{
    uint8_t buf[6];
    int16_t raw[3];
    
    if(mag == NULL) return 0;
    
    if(!WIT_IIC_ReadReg(WIT_MAG_REG, buf, 6)) return 0;
    
    raw[0] = BytesToShort(&buf[0]);
    raw[1] = BytesToShort(&buf[2]);
    raw[2] = BytesToShort(&buf[4]);
    
    mag[0] = (float)raw[0] / 32768.0f * 180.0f;
    mag[1] = (float)raw[1] / 32768.0f * 180.0f;
    mag[2] = (float)raw[2] / 32768.0f * 180.0f;
    
    return 1;
}

/**
 * @brief  读取四元数数据
 * @param  quat: 输出四元数数组 [4]
 * @retval 1:成功, 0:失败
 */
uint8_t WIT_IIC_ReadQuat(float *quat)
{
    uint8_t buf[8];
    int16_t raw[4];
    
    if(quat == NULL) return 0;
    
    if(!WIT_IIC_ReadReg(WIT_QUAT_REG, buf, 8)) return 0;
    
    raw[0] = BytesToShort(&buf[0]);  // q0
    raw[1] = BytesToShort(&buf[2]);  // q1
    raw[2] = BytesToShort(&buf[4]);  // q2
    raw[3] = BytesToShort(&buf[6]);  // q3
    
    quat[0] = (float)raw[0] / 32768.0f;
    quat[1] = (float)raw[1] / 32768.0f;
    quat[2] = (float)raw[2] / 32768.0f;
    quat[3] = (float)raw[3] / 32768.0f;
    
    return 1;
}

/**
 * @brief  读取温度数据
 * @retval 温度值 (0.1°C)
 */
int16_t WIT_IIC_ReadTemp(void)
{
    uint8_t buf[2];
    
    if(!WIT_IIC_ReadReg(WIT_TEMP_REG, buf, 2)) return 0;
    
    return BytesToShort(buf) / 10;  // 温度单位为0.1°C
}

/**
 * @brief  读取所有数据（一次性读取）
 * @param  data: 输出数据结构体指针
 * @retval 1:成功, 0:失败
 */
uint8_t WIT_IIC_ReadAll(WIT_Data_t *data)
{
    uint8_t buf[32];
    int16_t raw[16];
    uint8_t i;
    
    if(data == NULL) return 0;
    
    /* 从0x34开始读取32字节（包含加速度、陀螺仪、角度等）*/
    if(!WIT_IIC_ReadReg(0x34, buf, 24)) return 0;  // 读取24字节足够
    
    /* 解析数据 (每2字节为一个int16) */
    for(i = 0; i < 12; i++) {
        raw[i] = BytesToShort(&buf[i*2]);
    }
    
    /* 加速度 (0x34-0x39) */
    data->accel[0] = (float)raw[0] / 32768.0f * WIT_ACCEL_RANGE;
    data->accel[1] = (float)raw[1] / 32768.0f * WIT_ACCEL_RANGE;
    data->accel[2] = (float)raw[2] / 32768.0f * WIT_ACCEL_RANGE;
    
    /* 陀螺仪 (0x37-0x3C) */
    data->gyro[0] = (float)raw[3] / 32768.0f * WIT_GYRO_RANGE;
    data->gyro[1] = (float)raw[4] / 32768.0f * WIT_GYRO_RANGE;
    data->gyro[2] = (float)raw[5] / 32768.0f * WIT_GYRO_RANGE;
    
    /* 角度 (0x3D-0x42) */
    data->angle[0] = (float)raw[6] / 32768.0f * WIT_ANGLE_RANGE;  // Roll
    data->angle[1] = (float)raw[7] / 32768.0f * WIT_ANGLE_RANGE;  // Pitch
    data->angle[2] = (float)raw[8] / 32768.0f * WIT_ANGLE_RANGE;  // Yaw
    
    /* 温度 (0x40-0x41) 和 磁场可能需要另外读取 */
    
    data->last_update = HAL_GetTick();
    data->data_ready = 1;
    
    return 1;
}

/**
 * @brief  获取Yaw角
 * @retval Yaw角度 (-180 ~ 180)
 */
float WIT_IIC_GetYaw(void)
{
    return wit_data.angle[2];
}

/**
 * @brief  获取Pitch角
 * @retval Pitch角度 (-90 ~ 90)
 */
float WIT_IIC_GetPitch(void)
{
    return wit_data.angle[1];
}

/**
 * @brief  获取Roll角
 * @retval Roll角度 (-180 ~ 180)
 */
float WIT_IIC_GetRoll(void)
{
    return wit_data.angle[0];
}

/**
 * @brief  获取数据结构体指针
 * @retval 指向WIT_Data_t的指针
 */
WIT_Data_t* WIT_IIC_GetDataPtr(void)
{
    return &wit_data;
}
