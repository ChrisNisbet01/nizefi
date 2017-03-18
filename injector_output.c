#include "injector_output.h"
#include "pulser.h"
#include "utils.h"
#include "stm32f4xx_gpio.h"

#include <stddef.h>

/* TODO - Create a common GPIO config struct for all GPIO. */
typedef struct gpio_config_st
{
    uint32_t RCC_AHBPeriph;
    GPIO_TypeDef * port;
    uint_fast16_t pin;
} gpio_config_st;

struct injector_output_st
{
    gpio_config_st const * const gpio_config;

    float close_angle;
    size_t number; /* Which injector is this. NOT the same as the cylinder number. */
    pulser_st * pulser;
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

static volatile float engine_cycle_angle;

float get_angle_when_injector_closed(void)
{
    return engine_cycle_angle;
}

static void injector_active_cb(void * const arg)
{
    injector_output_st * const injector_output = arg;
    gpio_config_st const * const gpio_config = injector_output->gpio_config;

    GPIO_SetBits(gpio_config->port, gpio_config->pin);
}

static void injector_inactive_cb(void * const arg)
{
    injector_output_st * const injector_output = arg;
    gpio_config_st const * const gpio_config = injector_output->gpio_config; 
    float current_engine_cycle_angle_get(void); 

    GPIO_ResetBits(gpio_config->port, gpio_config->pin);

    engine_cycle_angle = current_engine_cycle_angle_get();
}

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
injector_output_st * injector_output_get(size_t const injector_number,
                                         float const injector_close_angle)
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

    injector_output->pulser = pulser_get(injector_active_cb,
                                         injector_inactive_cb,
                                         injector_output);

    /* XXX - This info should be maintained by the object owner. */
    injector_output->number = injector_number;
    injector_output->close_angle = injector_close_angle;

    next_injector_output++;

    /* Initialise the GPIO. */
    initialise_injector_gpio(gpio_config->port, gpio_config->pin, gpio_config->RCC_AHBPeriph);

done:
    return injector_output;
}

float injector_close_angle_get(injector_output_st const * const injector_output)
{
    return injector_output->close_angle;
}

size_t injector_number_get(injector_output_st const * const injector_output)
{
    return injector_output->number;
}

void injector_pulse_schedule(injector_output_st * const injector_output,
                             uint32_t const base_count,
                             uint32_t initial_delay_us,
                             uint16_t pulse_us)
{
    pulser_schedule_pulse(injector_output->pulser, base_count, initial_delay_us, pulse_us);
}

uint32_t injector_timer_count_get(injector_output_st const * const injector_output)
{
    return pulser_timer_count_get(injector_output->pulser);
}
