#include "bsp_buzzer.h"
#include "main.h"
extern TIM_HandleTypeDef htim4;
// 设置蜂鸣器频率和音调
void BEEP_SetFrequency(uint32_t freq)
{
    if(freq == 0)
    {
        // 停止蜂鸣 - 设置占空比为0
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
        return;
    }
    
    uint32_t timer_clock = 84000000;  // 84MHz - 定时器时钟频率
    uint32_t prescaler = 83;          // 预分频值 (84MHz / (83+1) = 1MHz)
    uint32_t period = (timer_clock / (prescaler + 1) / freq) - 1;
    
    // 确保period值在有效范围内
    if(period > 0xFFFF) period = 0xFFFF;  // 不能超过16位定时器最大值
    if(period < 1) period = 1;            // 最小周期值
    
    // 停止定时器并重新配置参数
    HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_3);
    
    // 重新设置周期和占空比
    __HAL_TIM_SET_PRESCALER(&htim4, prescaler);
    __HAL_TIM_SET_AUTORELOAD(&htim4, period);           // 设置自动重装载值(周期)
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, period / 2);  // 50%占空比
    htim4.Instance->EGR = TIM_EGR_UG;
    
    // 重新启动定时器
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
}

// 设置蜂鸣器音量（通过调节占空比）
void BEEP_SetVolume(uint8_t volume)
{
    if(volume > 100) volume = 100;  // 限制音量范围0-100
    
    uint32_t period = __HAL_TIM_GET_AUTORELOAD(&htim4);  // 获取当前周期值
    uint32_t pulse = (period * volume) / 100;            // 计算脉冲宽度
    
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, pulse); // 设置比较值(占空比)
}

// 播放指定频率和持续时间的音调
void BEEP_PlayTone(uint32_t freq, uint32_t duration_ms)
{
    BEEP_SetFrequency(freq);      // 设置频率
    HAL_Delay(duration_ms);       // 持续指定时间
    BEEP_SetFrequency(0);         // 停止蜂鸣
}

// 播放简单的音阶旋律
void BEEP_PlayMelody(void)
{
    uint32_t melody[] = {262, 294, 330, 349, 392, 440, 494, 523};  // C大调音阶频率(Hz)
    uint32_t duration = 200;  // 每个音符持续时间(ms)
    
    for(int i = 0; i < 8; i++)
    {
        BEEP_PlayTone(melody[i], duration);  // 播放音符
        HAL_Delay(50);                       // 音符间间隔
    }
}

// 播放警报声音：交替频率的报警声
void BEEP_PlayAlarm(uint16_t times)
{
    for(int i = 0; i < times; i++)
    {
        BEEP_PlayTone(1000, 100);  // 1kHz, 100ms
        HAL_Delay(100);            // 间隔100ms
        BEEP_PlayTone(1500, 100);  // 1.5kHz, 100ms
        HAL_Delay(200);            // 间隔200ms
    }
}
