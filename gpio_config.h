#ifndef __GPIO_CONFIG_H__
#define __GPIO_CONFIG_H__

#include "stm32f4xx_gpio.h"

#include <stdint.h>

typedef struct gpio_config_st
{
    uint32_t RCC_AHBPeriph;
    GPIO_TypeDef * port;
    uint_fast16_t pin;
} gpio_config_st; 

#endif /* __GPIO_CONFIG_H__ */
