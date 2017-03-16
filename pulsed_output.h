#ifndef __PULSED_OUTPUT_H__
#define __PULSED_OUTPUT_H__

#include "stm32f4xx_gpio.h"

#include <stdint.h>

typedef struct gpio_config_st
{
    uint32_t RCC_AHBPeriph;
    GPIO_TypeDef * port;
    uint_fast16_t pin;
} gpio_config_st; 

typedef struct pulsed_output_st pulsed_output_st;

pulsed_output_st * pulsed_output_get(gpio_config_st const * const gpio_config);
void pulsed_output_schedule(pulsed_output_st * const pulsed_output,
                            uint32_t base_count,
                            uint32_t const initial_delay_us,
                            uint_fast16_t const pulse_us);
uint32_t pulse_output_timer_count_get(pulsed_output_st const * const pulsed_output);

#endif /* __PULSED_OUTPUT_H__ */
