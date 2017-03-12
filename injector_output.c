#include "injector_output.h"
#include "pulsed_output.h"

#include <stddef.h>

struct injector_output_st
{
    gpio_config_st const * const gpio_config;
    pulsed_output_st * pulsed_output;
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

static gpio_config_st const injector_gpios[] =
{
    {
        .RCC_AHBPeriph = RCC_AHB1Periph_GPIOD,
        .port = GPIOD,
        .pin = GPIO_Pin_12
    },
    {
        .RCC_AHBPeriph = RCC_AHB1Periph_GPIOD,
        .port = GPIOD,
        .pin = GPIO_Pin_13
    },
    {
        .RCC_AHBPeriph = RCC_AHB1Periph_GPIOD,
        .port = GPIOD,
        .pin = GPIO_Pin_14
    },
    {
        .RCC_AHBPeriph = RCC_AHB1Periph_GPIOD,
        .port = GPIOD,
        .pin = GPIO_Pin_15
    }
};
#define NUM_INJECTOR_GPIOS (sizeof injector_gpios / sizeof injector_gpios[0])

static injector_output_st injector_outputs[NUM_INJECTOR_GPIOS] =
{
    [injector_1_index] =
    {
        .gpio_config = &injector_gpios[injector_1_index]
    }
    ,
    [injector_2_index] =
    {
        .gpio_config = &injector_gpios[injector_2_index]
    }
    ,
    [injector_3_index] =
    {
        .gpio_config = &injector_gpios[injector_3_index]
    },
    [injector_4_index] =
    {
        .gpio_config = &injector_gpios[injector_4_index]
    }
};
static size_t next_injector_output;

static void initialise_injector_gpio(GPIO_TypeDef * const gpio, uint_fast16_t pin, uint32_t const RCC_AHBPeriph)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    /* GPIOD Periph clock enable */
    RCC_AHB1PeriphClockCmd(RCC_AHBPeriph, ENABLE);

    /* Configure in output push-pull mode */
    GPIO_InitStructure.GPIO_Pin = pin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(gpio, &GPIO_InitStructure);
}

/* XXX - Consider an operational mode where mutliple injectors 
 * should be fired at the same time (i.e. batch mode). 
 */
injector_output_st * injector_output_get(void)
{
    injector_output_st * injector_output;
    gpio_config_st const * gpio_config;

    if (next_injector_output >= NUM_INJECTOR_GPIOS)
    {
        injector_output = NULL;
        goto done;
    }
    injector_output = &injector_outputs[next_injector_output];
    gpio_config = injector_output->gpio_config;
    next_injector_output++;

    /* Initialise the GPIO. */
    initialise_injector_gpio(gpio_config->port, gpio_config->pin, gpio_config->RCC_AHBPeriph);

    injector_output->pulsed_output = pulsed_output_get(injector_output->gpio_config);

done:
    return injector_output;
}

void injector_pulse_schedule(injector_output_st * const injector_output,
                             uint32_t initial_delay_us,
                             uint16_t pulse_us)
{
    pulsed_output_schedule(injector_output->pulsed_output, initial_delay_us, pulse_us);
}

