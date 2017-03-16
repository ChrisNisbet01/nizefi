#include "pulsed_output.h"
#include "stm32f4xx_gpio.h"
#include "pulser.h"

#include <stddef.h>

struct pulsed_output_st
{
    gpio_config_st const * gpio_config;
    pulser_st * pulser;
};

static void output_active_callback(void * const arg);
static void output_inactive_callback(void * const arg);

#define NUM_PULSED_OUTPUTS 16 /* enough for number of injector outputs + number of ignition outputs. */

static pulsed_output_st pulsed_outputs[NUM_PULSED_OUTPUTS];

static size_t next_pulsed_output;

static void output_active_callback(void * const arg)
{
    /* Called when the pulse goes active. Called from within an 
     * ISR.
     */
    pulsed_output_st * const pulsed_output = arg;
    gpio_config_st const * const gpio_config = pulsed_output->gpio_config;

    GPIO_SetBits(gpio_config->port, gpio_config->pin);
}

static void output_inactive_callback(void * const arg)
{
    /* Called when the pulse goes inactive. Called from within an 
     * ISR.
     */
    pulsed_output_st * const pulsed_output = arg;
    gpio_config_st const * const gpio_config = pulsed_output->gpio_config; 

    GPIO_ResetBits(gpio_config->port, gpio_config->pin);
}

pulsed_output_st * pulsed_output_get(gpio_config_st const * const gpio_config)
{
    pulsed_output_st * pulsed_output;

    if (next_pulsed_output >= NUM_PULSED_OUTPUTS)
    {
        pulsed_output = NULL;
        goto done;
    }
    pulsed_output = &pulsed_outputs[next_pulsed_output];
    next_pulsed_output++;

    pulsed_output->gpio_config = gpio_config;

    pulsed_output->pulser = pulser_get(output_active_callback,
                                       output_inactive_callback,
                                       pulsed_output);

done:
    return pulsed_output;
}

void pulsed_output_schedule(pulsed_output_st * const pulsed_output,
                            uint32_t base_count,
                            uint32_t initial_delay_us,
                            uint_fast16_t pulse_us)
{
    pulse_start(pulsed_output->pulser, base_count, initial_delay_us, pulse_us);
}

uint32_t pulse_output_timer_count_get(pulsed_output_st const * const pulsed_output)
{
    return pulser_timer_count_get(pulsed_output->pulser);
}
