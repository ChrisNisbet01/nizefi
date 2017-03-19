#ifndef __GPIO_OUTPUT_H__
#define __GPIO_OUTPUT_H__

#include "stm32f4xx_gpio.h"

#include <stdint.h>

void gpio_output_initialise(GPIO_TypeDef * const gpio,
                            uint_fast16_t const pin,
                            uint32_t const RCC_AHBPeriph);

void gpio_output_set_active(gpio_config_st const * const gpio_config);
void gpio_output_set_inactive(gpio_config_st const * const gpio_config);

#endif /* __GPIO_OUTPUT_H__ */
