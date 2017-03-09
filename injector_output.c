#include "injector_output.h"
#include "stm32f4xx_gpio.h"
#include "pulser.h"

#include <stddef.h>

typedef struct gpio_config_st
{
    uint32_t RCC_AHBPeriph;
    GPIO_TypeDef * port;
    uint_fast16_t pin;
} gpio_config_st; 

struct injector_context_st
{
    gpio_config_st const * const gpio_config;
    timed_event_context_st * pulser;
};

typedef enum inject_index_t
{
    injector_1_index,
    injector_2_index,
    injector_3_index,
    injector_4_index,
    injector_5_index,
    injector_6_index,
    injector_7_index,
    injector_8_index
} inject_index_t;

static void injector_active_callback(void * const arg);
static void injector_inactive_callback(void * const arg);

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
    }
};
#define NUM_INJECTOR_GPIOS (sizeof injector_gpios / sizeof injector_gpios[0])

/* Temp debug just use a couple of LED pins. Ignition will use 
 * the other couple. 
 */
static injector_context_st injector_contexts[NUM_INJECTOR_GPIOS] =
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
};

static size_t next_injector_context;

static void injector_active_callback(void * const arg)
{
    /* Called when the pulse goes active. Called from within an 
     * ISR.
     */
    injector_context_st * const injector_context = arg;
    gpio_config_st const * const gpio_config = injector_context->gpio_config;

    GPIO_SetBits(gpio_config->port, gpio_config->pin);
}

static void injector_inactive_callback(void * const arg)
{
    /* Called when the pulse goes inactive. Called from within an 
     * ISR.
     */
    injector_context_st * const injector_context = arg;
    gpio_config_st const * const gpio_config = injector_context->gpio_config; 

    GPIO_ResetBits(gpio_config->port, gpio_config->pin);
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
injector_context_st * injector_output_get(void)
{
    injector_context_st * injector_context;
    gpio_config_st const * gpio_config;

    if (next_injector_context >= NUM_INJECTOR_GPIOS)
    {
        injector_context = NULL;
        goto done;
    }
    injector_context = &injector_contexts[next_injector_context];
    gpio_config = injector_context->gpio_config;
    next_injector_context++;

    /* Initialise the GPIO. */
    initialise_injector_gpio(gpio_config->port, gpio_config->pin, gpio_config->RCC_AHBPeriph);

    injector_context->pulser = pulser_get(injector_active_callback,
                                          injector_inactive_callback,
                                          injector_context);

done:
    return injector_context;
}

void injector_pulse_schedule(injector_context_st * const injector_context,
                             uint32_t initial_delay_us,
                             uint16_t pulse_us)
{
    pulse_start(injector_context->pulser, initial_delay_us, pulse_us);
}

