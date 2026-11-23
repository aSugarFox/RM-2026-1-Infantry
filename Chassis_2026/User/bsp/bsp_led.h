#ifndef BSP_LED_H
#define BSP_LED_H
#include "struct_typedef.h"


void BSP_LED_Init(void);
void BSP_LED_Show(uint32_t aRGB);
void BSP_LED_Toggle(void);
// extern void aRGB_led_show(uint32_t aRGB);


#endif
