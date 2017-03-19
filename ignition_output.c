#include "ignition_output.h"
#include "pulser.h"
#include "utils.h"
#include "gpio_output.h"

#include <stddef.h>

struct ignition_output_st
{
    gpio_config_st const * const gpio_config;
};

typedef enum ignition_index_t
{
    ignition_1_index,
    ignition_2_index,
    ignition_3_index,
    ignition_4_index,
    ignition_5_index,
    ignition_6_index,
    ignition_7_index,
    ignition_8_index
} ignition_index_t;

static gpio_config_st const ignition_gpios[] =
{
    {
        .RCC_AHBPeriph = RCC_AHB1Periph_GPIOC,
        .port = GPIOC,
        .pin = GPIO_Pin_7
    },
    {
        .RCC_AHBPeriph = RCC_AHB1Periph_GPIOE,
        .port = GPIOE,
        .pin = GPIO_Pin_14
    },
    {
        .RCC_AHBPeriph = RCC_AHB1Periph_GPIOC,
        .port = GPIOC,
        .pin = GPIO_Pin_9
    },
    {
        .RCC_AHBPeriph = RCC_AHB1Periph_GPIOE,
        .port = GPIOE,
        .pin = GPIO_Pin_12
    }
};
#define NUM_IGNITION_GPIOS ARRAY_SIZE(ignition_gpios)

/* Temp debug just use a couple of LED pins. Injectors will use 
 * the other couple. 
 */
static ignition_output_st ignition_outputs[NUM_IGNITION_GPIOS] =
{
    [ignition_1_index] =
    {
        .gpio_config = &ignition_gpios[ignition_1_index]
    }
    ,
    [ignition_2_index] =
    {
    .gpio_config = &ignition_gpios[ignition_2_index]
    }
    ,
    [ignition_3_index] =
    {
        .gpio_config = &ignition_gpios[ignition_3_index]
    },
    [ignition_4_index] =
    {
        .gpio_config = &ignition_gpios[ignition_4_index]
    }
};
static size_t next_ignition_output;

ignition_output_st * ignition_output_get(void)
{
    ignition_output_st * ignition_output;

    if (next_ignition_output >= NUM_IGNITION_GPIOS)
    {
        ignition_output = NULL;
        goto done;
    }

    ignition_output = &ignition_outputs[next_ignition_output];

    next_ignition_output++;

    /* Initialise the GPIO. */
    gpio_output_initialise(ignition_output->gpio_config);

done:
    return ignition_output;
}

void ignition_set_active(ignition_output_st * const ignition_output)
{
    gpio_output_set_active(ignition_output->gpio_config);
}

void ignition_set_inactive(ignition_output_st * const ignition_output)
{
    gpio_output_set_inactive(ignition_output->gpio_config);
}


