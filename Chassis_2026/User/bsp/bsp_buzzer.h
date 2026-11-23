#ifndef BSP_BUZZER_H
#define BSP_BUZZER_H
#include "struct_typedef.h"

//extern void buzzer_on(uint16_t psc, uint16_t pwm);
//extern void buzzer_off(void);
// void BEEP_Init(void);
void BEEP_SetFrequency(uint32_t freq);
void BEEP_SetVolume(uint8_t volume);
void BEEP_PlayTone(uint32_t freq, uint32_t duration_ms);
void BEEP_PlayMelody(void);
void BEEP_PlayAlarm(uint16_t times);



#endif
