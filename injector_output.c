#include "injector_output.h"
#include "pulser.h"
#include "utils.h"
#include "gpio_config.h"

#include <stddef.h>

struct injector_output_st
{
    gpio_config_st const * const gpio_config;
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
#define NUM_INJECTOR_GPIOS ARRAY_SIZE(injector_gpios)

/* XXX - Must match the number of injector GPIO in the table 
 * above. 
 */
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

static void initialise_injector_gpio(GPIO_TypeDef * const gpio, 
                                     uint_fast16_t const pin, 
                                     uint32_t const RCC_AHBPeriph)
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

    /* TODO: Add support for getting the GPIO to use from a pool of
     * GPIO? Each GPIO would have some flags indicating its
     * capabilites (e.g. injector, ignition, relay, trigger input, 
     * ADC, whatever). 
     */

    next_injector_output++;

    /* Initialise the GPIO. */
    initialise_injector_gpio(gpio_config->port, gpio_config->pin, gpio_config->RCC_AHBPeriph);

done:
    return injector_output;
}

void injector_set_active(injector_output_st * const injector_output)
{
    gpio_config_st const * const gpio_config = injector_output->gpio_config;

    GPIO_SetBits(gpio_config->port, gpio_config->pin);
}

void injector_set_inactive(injector_output_st * const injector_output)
{
    gpio_config_st const * const gpio_config = injector_output->gpio_config;

    GPIO_ResetBits(gpio_config->port, gpio_config->pin);
}

