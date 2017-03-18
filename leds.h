#ifndef __LEDS_H__
#define __LEDS_H__

#include "stm32f4xx_gpio.h"

#include <stdint.h>

#define GREEN_LED GPIO_Pin_12
#define ORANGE_LED GPIO_Pin_13
#define RED_LED GPIO_Pin_14
#define BLUE_LED GPIO_Pin_15


void led_toggle(uint_fast16_t led);
void led_set(uint_fast16_t led);
void led_reset(uint_fast16_t led);
void init_leds(void);

#endif /* __LEDS_H__ */
