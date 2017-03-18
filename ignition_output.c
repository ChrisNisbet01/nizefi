#include "ignition_output.h"
#include "pulser.h"
#include "utils.h"
#include "gpio_config.h"

#include <stddef.h>

struct ignition_output_st
{
    gpio_config_st const * const gpio_config;

    float spark_angle;
    size_t number; /* Which ignition is this. NOT the same as the cylinder number. */
    pulser_st * pulser;
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

static volatile float engine_cycle_angle;

float get_angle_when_ignition_sparked(void)
{
    return engine_cycle_angle;
}

static void ignition_active_cb(void * const arg)
{
    ignition_output_st * const ignition_output = arg;
    gpio_config_st const * const gpio_config = ignition_output->gpio_config;

    GPIO_SetBits(gpio_config->port, gpio_config->pin);
}

static void ignition_inactive_cb(void * const arg)
{
    ignition_output_st * const ignition_output = arg;
    gpio_config_st const * const gpio_config = ignition_output->gpio_config; 
    float current_engine_cycle_angle_get(void); 

    GPIO_ResetBits(gpio_config->port, gpio_config->pin);

    engine_cycle_angle = current_engine_cycle_angle_get();
}

static void initialise_ignition_gpio(GPIO_TypeDef * const gpio, uint_fast16_t pin, uint32_t const RCC_AHBPeriph)
{
    /* XXX - Currently exactly the same as the equivalent function 
     * in injector_output.c. Use a common function until they 
     * deviate? 
     */
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

/* XXX - Consider an operational mode where mutliple ignition 
 * outputs should be fired at the same time. 
 */
ignition_output_st * ignition_output_get(size_t const ignition_number,
                                         float const ignition_spark_angle)
{
    ignition_output_st * ignition_output;
    gpio_config_st const * gpio_config;

    if (next_ignition_output >= NUM_IGNITION_GPIOS)
    {
        ignition_output = NULL;
        goto done;
    }

    ignition_output = &ignition_outputs[next_ignition_output];
    gpio_config = ignition_output->gpio_config;

    ignition_output->pulser = pulser_get(ignition_active_cb,
                                         ignition_inactive_cb,
                                         ignition_output);

    /* XXX - This info should be maintained by the object owner. */
    ignition_output->number = ignition_number;
    ignition_output->spark_angle = ignition_spark_angle;

    next_ignition_output++;

    /* Initialise the GPIO. */
    initialise_ignition_gpio(gpio_config->port, gpio_config->pin, gpio_config->RCC_AHBPeriph);

done:
    return ignition_output;
}

size_t ignition_number_get(ignition_output_st * const ignition_output)
{
    return ignition_output->number;
}

void ignition_pulse_schedule(ignition_output_st * const ignition_output,
                             uint32_t const base_count,
                             uint32_t const initial_delay_us,
                             uint16_t const pulse_us)
{
    pulser_schedule_pulse(ignition_output->pulser, base_count, initial_delay_us, pulse_us);
}

uint32_t ignition_timer_count_get(ignition_output_st const * const injector_output)
{
    return pulser_timer_count_get(injector_output->pulser);
}

