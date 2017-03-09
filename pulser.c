#include "pulser.h"
#include "timed_events.h"

#include <coocox.h>

#include <stddef.h>
#include <stdint.h>
#include <inttypes.h>

#define NUM_PULSERS 4

//#define PULSE_DEBUG

typedef void (* state_handler)(timed_event_context_st * context);

struct timed_event_context_st
{
    OS_FlagID flag; /* Used to signal the pulser task to run from the ISR. */

    /* ISR level and task level state machine handlers. */
    state_handler isr_handler;
    state_handler task_handler;

    timer_channel_context_st * timer_context;

    pulser_callback active_callback;
    pulser_callback inactive_callback; 
    void * user_arg;

    uint_fast16_t active_us; /* The period of time the pulse should be active for. */

#if defined(PULSE_DEBUG)
    /* Some debug information. */
    uint32_t systick_at_start;
    uint32_t systick_at_gpio_off;
    uint32_t systick_at_start_task;
    uint32_t systick_at_gpio_off_task; 
#endif
};

static void timed_event_initial_delay_isr_handler(timed_event_context_st * context);
static void timed_event_initial_delay_task_handler(timed_event_context_st * context);

static void timed_event_active_isr_handler(timed_event_context_st * context);
static void timed_event_active_task_handler(timed_event_context_st * context);

static timed_event_context_st timed_event_contexts[NUM_PULSERS]; 
static size_t next_pulser;

static void indicate_timeout(timed_event_context_st * context)
{
    CoEnterISR();
    isr_SetFlag(context->flag);
    CoExitISR();
}

static void timed_event_idle_isr_handler(timed_event_context_st * context)
{
    (void)context;
}

static void timed_event_idle_task_handler(timed_event_context_st * context)
{
    (void)context;
}

static void timed_event_initial_delay_isr_handler(timed_event_context_st * context)
{
    /* The initial delay is over. */
    context->active_callback(context->user_arg);
#if defined(PULSE_DEBUG)
    if (context->systick_at_start == 0)
    {
        context->systick_at_start = SysTick->VAL;
    }
#endif
}

static void timed_event_initial_delay_task_handler(timed_event_context_st * context)
{
#if defined(PULSE_DEBUG)
    if (context->systick_at_start_task == 0)
    {
        context->systick_at_start_task = SysTick->VAL; 
    }
#endif
    context->isr_handler = timed_event_active_isr_handler;
    context->task_handler = timed_event_active_task_handler;

    timer_channel_schedule_followup_event(context->timer_context, context->active_us);
}

static void timed_event_active_isr_handler(timed_event_context_st * context)
{
    /* The pulse time is over. */
    context->inactive_callback(context->user_arg);
#if defined(PULSE_DEBUG)
    if (context->systick_at_gpio_off == 0)
    {
        context->systick_at_gpio_off = SysTick->VAL;
    }
#endif
}

static void timed_event_active_task_handler(timed_event_context_st * context)
{
#if defined(PULSE_DEBUG)
    if (context->systick_at_gpio_off_task == 0)
    {
        context->systick_at_gpio_off_task = SysTick->VAL;
    }
#endif
    context->isr_handler = timed_event_idle_isr_handler;
    context->task_handler = timed_event_idle_task_handler;
    timer_channel_disable(context->timer_context);
}

static void timed_event_callback(void * const arg)
{
    timed_event_context_st * const context = arg;

    context->isr_handler(context);
    /* We signal the pulser task no matter the current state, 
     * so to avoid repitition we put the call to indicate a timeout after the state handler. 
     */
    indicate_timeout(context);
}

void schedule_pulse(timed_event_context_st * const context, uint32_t const base_time, uint32_t const initial_delay_us, uint_fast16_t const pulse_us)
{
    if (context->timer_context != NULL)
    {
        context->isr_handler = timed_event_initial_delay_isr_handler;
        context->task_handler = timed_event_initial_delay_task_handler;
        context->active_us = pulse_us;

#if defined(PULSE_DEBUG)
        context->systick_at_start = 0;
        context->systick_at_gpio_off = 0;
        context->systick_at_start_task = 0;
        context->systick_at_gpio_off_task = 0;
#endif

        timer_channel_schedule_new_based_event(context->timer_context, base_time, initial_delay_us);
    }
}

void pulse_start(timed_event_context_st * const context,
                 uint32_t initial_delay_us, 
                 uint16_t pulse_us)
{
    schedule_pulse(context, timer_channel_get_current_time(context->timer_context), initial_delay_us, pulse_us);
}

timed_event_context_st * pulser_get(pulser_callback const active_callback,
                                    pulser_callback const inactive_callback,
                                    void * const user_arg)
{
    timed_event_context_st * pulser;

    if (next_pulser >= NUM_PULSERS)
    {
        pulser = NULL;
        goto done;
    }

    pulser = &timed_event_contexts[next_pulser];

    pulser->active_callback = active_callback;
    pulser->inactive_callback = inactive_callback;
    pulser->user_arg = user_arg;

    next_pulser++;

done:
    return pulser;
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
    }
}

void print_pulse_details(void)
{
#if defined(PULSE_DEBUG)
    size_t index;

    for (index = 0; index < NUM_PULSERS; index++)
    {
        timed_event_context_st * const context = &timed_event_contexts[index];

        if (context->systick_at_start > 0)
        {
            printf("%u start: %"PRIu32" task: %"PRIu32" end: %"PRIu32" end: %"PRIu32" diff %"PRIu32"\r\n",
                   index, 
                   context->systick_at_start, 
                   context->systick_at_start_task,
                   context->systick_at_gpio_off,
                   context->systick_at_gpio_off_task,
                   context->systick_at_start - context->systick_at_gpio_off);
            printf(" start diff: %"PRIu32" end diff: %"PRIu32"\r\n",
                   context->systick_at_start - context->systick_at_start_task,
                   context->systick_at_gpio_off - context->systick_at_gpio_off_task);
        }
    }
#endif
}

void reset_pulse_details(void)
{
#if defined(PULSE_DEBUG)
    size_t index;

    printf("reset\r\n");
    for (index = 0; index < NUM_PULSERS; index++)
    {
        timed_event_context_st * const context = &timed_event_contexts[index];

        context->systick_at_start = 0;
        context->systick_at_gpio_off = 0;
    }
#endif
}
