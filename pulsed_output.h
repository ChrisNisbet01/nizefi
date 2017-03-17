#ifndef __PULSED_OUTPUT_H__
#define __PULSED_OUTPUT_H__

#include "stm32f4xx_gpio.h"

#include <stdint.h>

/* TODO - Create a common GPIO config struct for all GPIO. */
typedef struct gpio_config_st
{
    uint32_t RCC_AHBPeriph;
    GPIO_TypeDef * port;
    uint_fast16_t pin;
} gpio_config_st; 

typedef void (* pulsed_output_notification_fn)(void);

typedef struct pulse_output_init_st
{
    gpio_config_st const * gpio_config;
    pulsed_output_notification_fn active_cb; /* Called when the output is set active in ISR. */
    pulsed_output_notification_fn inactive_cb; /* Called when the output is set in active in ISR. */
} pulse_output_init_st;

typedef struct pulsed_output_st pulsed_output_st;

pulsed_output_st * pulsed_output_get(pulse_output_init_st * const init);
void pulsed_output_schedule(pulsed_output_st * const pulsed_output,
                            uint32_t base_count,
                            uint32_t const initial_delay_us,
                            uint_fast16_t const pulse_us);
uint32_t pulse_output_timer_count_get(pulsed_output_st const * const pulsed_output);

#endif /* __PULSED_OUTPUT_H__ */
