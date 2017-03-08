#include "pulser.h"
#include "timed_events.h"
#include "stm32f4xx_gpio.h"

#include <stddef.h>
#include <stdint.h>
#include <inttypes.h>

typedef struct timed_event_context_st timed_event_context_st;
typedef void (* state_handler)(timed_event_context_st * context);

struct timed_event_context_st
{
    volatile uint16_t initial_delay_us;
    volatile uint16_t pulses;
    uint16_t active_us;
    uint16_t inactive_us;
    state_handler handler;
    timer_channel_context_st * timer_context;
    uint16_t gpio_pin;
    uint32_t systick_at_start;
    uint32_t systick_at_gpio_off;
};

static void timed_event_init_handler(timed_event_context_st * context);
static void timed_event_initial_delay_handler(timed_event_context_st * context);
static void timed_event_active_handler(timed_event_context_st * context);
static void timed_event_inactive_handler(timed_event_context_st * context);

static void timed_event_init_handler(timed_event_context_st * context)
{
    context->handler = timed_event_initial_delay_handler;

    timer_channel_schedule_new_event(context->timer_context, context->initial_delay_us);
}

static void timed_event_initial_delay_handler(timed_event_context_st * context)
{
    GPIO_SetBits(GPIOD, context->gpio_pin);
    if (context->systick_at_start == 0)
    {
        context->systick_at_start = SysTick->VAL;
    }

    context->handler = timed_event_active_handler; 

    timer_channel_schedule_followup_event(context->timer_context, context->active_us);
}

static void timed_event_active_handler(timed_event_context_st * context)
{
    GPIO_ResetBits(GPIOD, context->gpio_pin);
    if (context->systick_at_gpio_off == 0)
    {
        context->systick_at_gpio_off = SysTick->VAL;
    }

    if (context->pulses > 0)
    {
        context->pulses--;
    }
    if (context->pulses > 0)
    {
        context->handler = timed_event_inactive_handler;
        timer_channel_schedule_followup_event(context->timer_context, context->inactive_us);
    }
    else
    {
        context->handler = timed_event_init_handler;
        timer_channel_disable(context->timer_context);
    }
}

static void timed_event_inactive_handler(timed_event_context_st * context)
{
    GPIO_SetBits(GPIOD, context->gpio_pin);

    context->handler = timed_event_active_handler;

    timer_channel_schedule_followup_event(context->timer_context, context->active_us);
}

static timed_event_context_st timed_event_contexts[4];

static void timed_event_callback(void * const arg)
{
    timed_event_context_st * const context = arg;

    context->handler(context);
}

void pulse_start(uint32_t pulses, uint16_t pulse_us, uint16_t period_us)
{
    size_t index;
    uint32_t counts[4];

    for (index = 0; index < 4; index++)
    {
        timed_event_context_st * const context = &timed_event_contexts[index];

        counts[index] = timer_channel_get_current_time(context->timer_context);
    }

    for (index = 0; index < 4; index++)
    {
        timed_event_context_st * const context = &timed_event_contexts[index];

        if (context->timer_context != NULL)
        {
            context->handler = timed_event_init_handler;
            context->initial_delay_us = period_us;
            context->active_us = pulse_us;
            context->inactive_us = period_us - pulse_us;
            context->pulses = pulses;

            /* XXX - Fix so that we don't have both init and initial delay 
             * states. 
             */
            context->systick_at_start = 0; 
            context->systick_at_gpio_off = 0;
            timer_channel_schedule_new_based_event(context->timer_context, counts[index], period_us);
        }
    }
}

void init_pulses(void)
{
    size_t index;

    for (index = 0; index < 4; index++)
    {
        timed_event_context_st * const context = &timed_event_contexts[index];

        context->timer_context = timer_channel_get(timed_event_callback, context);
        context->gpio_pin = GPIO_Pin_12 << index;
    }
}

void print_pulse_details(void)
{
    size_t index;

    for (index = 0; index < 4; index++)
    {
        timed_event_context_st * const context = &timed_event_contexts[index];

        if (context->systick_at_start > 0)
        {
            printf("%u start: %"PRIu32" end: %"PRIu32" diff %"PRIu32"\r\n", 
                   index, 
                   context->systick_at_start, 
                   context->systick_at_gpio_off,
                   context->systick_at_start - context->systick_at_gpio_off);
        }
    }
}
