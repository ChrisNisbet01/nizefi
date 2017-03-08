#include "pulser.h"
#include "timed_events.h"
#include "stm32f4xx_gpio.h"

#include <coocox.h>

#include <stddef.h>
#include <stdint.h>
#include <inttypes.h>

#define NUM_PULSERS 4

typedef struct timed_event_context_st timed_event_context_st;
typedef void (* state_handler)(timed_event_context_st * context);

struct timed_event_context_st
{
    volatile uint16_t initial_delay_us;
    volatile uint16_t pulses;
    uint16_t active_us;
    uint16_t inactive_us;
    state_handler isr_handler;
    state_handler task_handler;
    timer_channel_context_st * timer_context;
    uint16_t gpio_pin;
    uint32_t systick_at_start;
    uint32_t systick_at_gpio_off;

    OS_FlagID flag;
};

static void timed_event_init_isr_handler(timed_event_context_st * context);
static void timed_event_init_task_handler(timed_event_context_st * context);

static void timed_event_initial_delay_isr_handler(timed_event_context_st * context);
static void timed_event_initial_delay_task_handler(timed_event_context_st * context);

static void timed_event_active_isr_handler(timed_event_context_st * context);
static void timed_event_active_task_handler(timed_event_context_st * context);

static void timed_event_inactive_isr_handler(timed_event_context_st * context);
static void timed_event_inactive_task_handler(timed_event_context_st * context);

static void indicate_timeout(timed_event_context_st * context)
{
    CoEnterISR();
    isr_SetFlag(context->flag);
    CoExitISR();
}

static void timed_event_init_isr_handler(timed_event_context_st * context)
{
}

static void timed_event_init_task_handler(timed_event_context_st * context)
{
    context->isr_handler = timed_event_initial_delay_isr_handler;
    context->task_handler = timed_event_initial_delay_task_handler;

    timer_channel_schedule_new_event(context->timer_context, context->initial_delay_us);
}

static void timed_event_initial_delay_isr_handler(timed_event_context_st * context)
{
    GPIO_SetBits(GPIOD, context->gpio_pin);
    if (context->systick_at_start == 0)
    {
        context->systick_at_start = SysTick->VAL;
    }
}

static void timed_event_initial_delay_task_handler(timed_event_context_st * context)
{

    context->isr_handler = timed_event_active_isr_handler;
    context->task_handler = timed_event_active_task_handler;

    timer_channel_schedule_followup_event(context->timer_context, context->active_us);
}

static void timed_event_active_isr_handler(timed_event_context_st * context)
{
    GPIO_ResetBits(GPIOD, context->gpio_pin);
    if (context->systick_at_gpio_off == 0)
    {
        context->systick_at_gpio_off = SysTick->VAL;
    }
}

static void timed_event_active_task_handler(timed_event_context_st * context)
{
    if (context->pulses > 0)
    {
        context->pulses--;
    }
    if (context->pulses > 0)
    {
        context->isr_handler = timed_event_inactive_isr_handler;
        context->task_handler = timed_event_inactive_task_handler;
        timer_channel_schedule_followup_event(context->timer_context, context->inactive_us);
    }
    else
    {
        context->isr_handler = timed_event_init_isr_handler;
        context->task_handler = timed_event_init_task_handler;
        timer_channel_disable(context->timer_context);
    }
}

static void timed_event_inactive_isr_handler(timed_event_context_st * context)
{
    GPIO_SetBits(GPIOD, context->gpio_pin);
}

static void timed_event_inactive_task_handler(timed_event_context_st * context)
{
    context->isr_handler = timed_event_active_isr_handler;
    context->task_handler = timed_event_active_task_handler; 

    timer_channel_schedule_followup_event(context->timer_context, context->active_us);
}

static timed_event_context_st timed_event_contexts[NUM_PULSERS];

static void timed_event_callback(void * const arg)
{
    timed_event_context_st * const context = arg;

    context->isr_handler(context);
    /* We signal the pulser task no matter the current state, 
     * so to avoid repitition we put the call to indicate a timeout after the state handler. 
     */
    indicate_timeout(context);
}

void pulse_start(uint32_t pulses, uint16_t pulse_us, uint16_t period_us)
{
    size_t index;
    uint32_t counts[NUM_PULSERS];

    for (index = 0; index < NUM_PULSERS; index++)
    {
        timed_event_context_st * const context = &timed_event_contexts[index];

        counts[index] = timer_channel_get_current_time(context->timer_context);
    }

    for (index = 0; index < NUM_PULSERS; index++)
    {
        timed_event_context_st * const context = &timed_event_contexts[index];

        if (context->timer_context != NULL)
        {
            context->isr_handler = timed_event_init_isr_handler;
            context->task_handler = timed_event_init_task_handler;
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

#define STACK_SIZE_PULSER 1024
static __attribute((aligned(8))) OS_STK pulser_stk[STACK_SIZE_PULSER]; /*!< Define "taskC" task stack */

void pulser_task(void * pdata)
{
    int index;
    uint32_t flags_to_check;

    for (index = 0, flags_to_check = 0; index < NUM_PULSERS; index++)
    {
        timed_event_context_st * const context = &timed_event_contexts[index];
        flags_to_check |= (1 << context->flag);
    }

    (void)pdata;
    while (1)
    {
        size_t index;
        StatusType err;
        U32 readyFlags;

        readyFlags = CoWaitForMultipleFlags(flags_to_check, OPT_WAIT_ANY, 0, &err);
        for (index = 0; index < NUM_PULSERS; index++)
        {
            timed_event_context_st * const context = &timed_event_contexts[index];

            if ((readyFlags & (1 << context->flag)))
            {
                context->task_handler(context); 
            }
        }
    }
}

void init_pulses(void)
{
    size_t index;

    CoCreateTask(pulser_task, 0, 1, &pulser_stk[STACK_SIZE_PULSER - 1], STACK_SIZE_PULSER);

    for (index = 0; index < NUM_PULSERS; index++)
    {
        timed_event_context_st * const context = &timed_event_contexts[index];

        context->flag = CoCreateFlag(Co_TRUE, Co_FALSE);

        context->timer_context = timer_channel_get(timed_event_callback, context);
        context->gpio_pin = GPIO_Pin_12 << index;
    }
}

void print_pulse_details(void)
{
    size_t index;

    for (index = 0; index < NUM_PULSERS; index++)
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

void reset_pulse_details(void)
{
    size_t index;

    printf("reset\r\n");
    for (index = 0; index < NUM_PULSERS; index++)
    {
        timed_event_context_st * const context = &timed_event_contexts[index];

        context->systick_at_start = 0;
        context->systick_at_gpio_off = 0;
    }
}
