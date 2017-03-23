#include "injector_output.h"
#include "pulser.h"
#include "utils.h"
#include "gpio_output.h"

#include <stddef.h>

struct injector_output_st
{
    gpio_config_st const gpio_config;
};

typedef enum injector_index_t
{
    injector_1_index,
    injector_2_index,
    injector_3_index,
    injector_4_index,
    injector_5_index,
    injector_6_index,
    injector_7_index,
    injector_8_index
} injector_index_t;

/* TODO - Support the gpio used on the Frankenso board. 
 * Support all 8 GPIO. 
 */

static injector_output_st injector_outputs[] =
{
    [injector_1_index] =
    {
        .gpio_config =     
        {
            .RCC_AHBPeriph = RCC_AHB1Periph_GPIOD,
            .port = GPIOD,
            .pin = GPIO_Pin_12
        }
    }
    ,
    [injector_2_index] =
    {
        .gpio_config = 
        {
            .RCC_AHBPeriph = RCC_AHB1Periph_GPIOD,
            .port = GPIOD,
            .pin = GPIO_Pin_13
        }
    }
    ,
    [injector_3_index] =
    {
        .gpio_config = 
        {
            .RCC_AHBPeriph = RCC_AHB1Periph_GPIOD,
            .port = GPIOD,
            .pin = GPIO_Pin_14
        }
    },
    [injector_4_index] =
    {
        .gpio_config = 
        {
            .RCC_AHBPeriph = RCC_AHB1Periph_GPIOD,
            .port = GPIOD,
            .pin = GPIO_Pin_15
        }
    }
};
#define NUM_INJECTOR_GPIOS ARRAY_SIZE(injector_outputs)

static size_t next_injector_output;

injector_output_st * injector_output_get(void)
{
    injector_output_st * injector_output;

    if (next_injector_output >= NUM_INJECTOR_GPIOS)
    {
        injector_output = NULL;
        goto done;
    }

    injector_output = &injector_outputs[next_injector_output];

    next_injector_output++;

    /* Initialise the GPIO. */
    gpio_output_initialise(&injector_output->gpio_config);

done:
    return injector_output;
}

void injector_set_active(injector_output_st * const injector_output)
{
    gpio_output_set_active(&injector_output->gpio_config);
}

void injector_set_inactive(injector_output_st * const injector_output)
{
    gpio_output_set_inactive(&injector_output->gpio_config);
}

