#ifndef __GPIO_OUTPUT_H__
#define __GPIO_OUTPUT_H__

#include "stm32f4xx_gpio.h"

#include <stdint.h>

typedef struct gpio_config_st
{
    uint32_t RCC_AHBPeriph;
    GPIO_TypeDef * port;
    uint_fast16_t pin;
} gpio_config_st; 

void gpio_output_initialise(gpio_config_st const * const gpio_config);

void gpio_output_set_active(gpio_config_st const * const gpio_config);
void gpio_output_set_inactive(gpio_config_st const * const gpio_config);

#endif /* __GPIO_OUTPUT_H__ */
