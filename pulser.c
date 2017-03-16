#include "pulser.h"
#include "timed_events.h"

#include <coocox.h>

#include <stddef.h>
#include <stdint.h>
#include <inttypes.h>

#define STACK_SIZE_PULSER 1024
#define NUM_PULSERS 16 /* Must be >= number of injectors + number of ignition outputs. */

//#define PULSE_DEBUG

typedef void (* state_handler)(pulser_st * context);

struct pulser_st
{
    OS_FlagID flag; /* Used to signal the pulser task to run from the ISR. */

    /* ISR level and task level state machine handlers. */
    state_handler isr_state_handler;
    state_handler task_state_handler;

    timer_channel_context_st * timer_context;

    pulser_callback active_callback;
    pulser_callback inactive_callback; 
    void * user_arg;

    /* TODO: Add support for overlapping pulses. It is entirely 
     * possible (when engine is highly loaded at high RPM) that 
     * one pulse could be scheduled when the previous hasn't 
     * completed. It should _not_ be possible for the new request 
     * to request a start time before the previous one finishes. 
     * May need to update the initial delay on the new request to 
     * account for the delay in starting the timer. 
     */
    uint_fast16_t active_us; /* The period of time the pulse should be active for. */
    uint_fast16_t initial_delay_left_us;

#if defined(PULSE_DEBUG)
    /* Some debug information. */
    uint32_t systick_at_start;
    uint32_t systick_at_gpio_off;
    uint32_t systick_at_start_task;
    uint32_t systick_at_gpio_off_task; 
#endif
};

static void pulser_initial_delay_isr_handler(pulser_st * context);
static void pulser_initial_delay_task_handler(pulser_st * context);

static void pulser_initial_delay_overflow_isr_handler(pulser_st * context);
static void pulser_initial_delay_overflow_task_handler(pulser_st * context); 

static void pulser_active_isr_handler(pulser_st * context);
static void pulser_active_task_handler(pulser_st * context);

typedef struct pulser_state_st
{
    __attribute((aligned(8))) OS_STK pulser_stk[STACK_SIZE_PULSER];

    pulser_st pulser_contexts[NUM_PULSERS];
    size_t next_pulser;
    uint32_t active_pulser_flags;
} pulser_state_st;

static pulser_state_st pulser_state;

static void indicate_timeout(pulser_st * context)
{
    CoEnterISR();

    isr_SetFlag(context->flag);

    CoExitISR();
}

static void pulser_idle_isr_handler(pulser_st * context)
{
    (void)context;
}

static void pulser_idle_task_handler(pulser_st * context)
{
    (void)context;
}

static void pulser_initial_delay_overflow_isr_handler(pulser_st * context)
{
    (void)context;
}

static void pulser_initial_delay_overflow_task_handler(pulser_st * context)
{
    (void)context;
    if (context->initial_delay_left_us > 40000)
    {
        context->initial_delay_left_us -= 30000;
        timer_channel_schedule_followup_event(context->timer_context, 30000); 
    }
    else
    {
        context->isr_state_handler =  pulser_initial_delay_isr_handler;
        context->task_state_handler = pulser_initial_delay_task_handler;

        timer_channel_schedule_followup_event(context->timer_context, context->initial_delay_left_us);
        context->initial_delay_left_us = 0;
    }
}

static void pulser_initial_delay_isr_handler(pulser_st * context)
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

static void pulser_initial_delay_task_handler(pulser_st * context)
{
#if defined(PULSE_DEBUG)
    if (context->systick_at_start_task == 0)
    {
        context->systick_at_start_task = SysTick->VAL; 
    }
#endif
    /* TODO: Add support for pulses longer than 65536 by adding a 
     * new state that schedules follow up events, but leaves the 
     * pulser active. Add similar support for longer initial delays.
     */
    context->isr_state_handler =  pulser_active_isr_handler; 
    context->task_state_handler = pulser_active_task_handler;

    timer_channel_schedule_followup_event(context->timer_context, context->active_us);
}

static void pulser_active_isr_handler(pulser_st * context)
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

static void pulser_active_task_handler(pulser_st * context)
{
#if defined(PULSE_DEBUG)
    if (context->systick_at_gpio_off_task == 0)
    {
        context->systick_at_gpio_off_task = SysTick->VAL;
    }
#endif
    context->isr_state_handler = pulser_idle_isr_handler;
    context->task_state_handler = pulser_idle_task_handler;
    timer_channel_disable(context->timer_context);
}

static void pulser_timer_callback(void * const arg)
{
    pulser_st * const context = arg;

    context->isr_state_handler(context);
    /* We signal the pulser task no matter the current state.
     * To avoid repitition we put the call to signal the pulser task after the state handler. 
     */
    indicate_timeout(context);
}

void schedule_pulse(pulser_st * const context, uint32_t const base_time, uint32_t const initial_delay_us, uint_fast16_t const pulse_us)
{
    if (context->timer_context != NULL)
    {
        uint32_t initial_delay = initial_delay_us;
        context->active_us = pulse_us; /* Remember the pulse width. */

        if (initial_delay_us > 40000)
        {
            initial_delay = 30000;
            context->initial_delay_left_us = initial_delay_us - 30000;
            context->isr_state_handler = pulser_initial_delay_overflow_isr_handler;
            context->task_state_handler = pulser_initial_delay_overflow_task_handler;
        }
        else
        {
            context->initial_delay_left_us = 0;
            context->isr_state_handler = pulser_initial_delay_isr_handler;
            context->task_state_handler = pulser_initial_delay_task_handler; 
        }


#if defined(PULSE_DEBUG)
        context->systick_at_start = 0;
        context->systick_at_gpio_off = 0;
        context->systick_at_start_task = 0;
        context->systick_at_gpio_off_task = 0;
#endif

        timer_channel_schedule_new_based_event(context->timer_context, base_time, initial_delay);
    }
}

void pulse_start(pulser_st * const context,
                 uint32_t base_count,
                 uint32_t initial_delay_us, 
                 uint_fast16_t pulse_us)
{
    schedule_pulse(context, base_count, initial_delay_us, pulse_us);
}

pulser_st * pulser_get(pulser_callback const active_callback,
                    pulser_callback const inactive_callback,
                    void * const user_arg)
{
    pulser_st * pulser;

    if (pulser_state.next_pulser >= NUM_PULSERS)
    {
        pulser = NULL;
        goto done;
    }

    pulser = &pulser_state.pulser_contexts[pulser_state.next_pulser];

    pulser->active_callback = active_callback;
    pulser->inactive_callback = inactive_callback;
    pulser->user_arg = user_arg;

    pulser_state.next_pulser++;
    pulser_state.active_pulser_flags |= 1 << pulser->flag;

done:
    return pulser;
}

static void pulser_task(void * pdata)
{
    (void)pdata;

    while (1)
    {
        size_t index;
        StatusType err;
        U32 readyFlags;

        readyFlags = CoWaitForMultipleFlags(pulser_state.active_pulser_flags, OPT_WAIT_ANY, 0, &err);

        for (index = 0; index < pulser_state.next_pulser; index++)
        {
            pulser_st * const context = &pulser_state.pulser_contexts[index];

            if ((readyFlags & (1 << context->flag)))
            {
                context->task_state_handler(context); 
            }
        }
    }
}

void init_pulsers(void)
{
    size_t index;

    CoCreateTask(pulser_task, 0, 2, &pulser_state.pulser_stk[STACK_SIZE_PULSER - 1], STACK_SIZE_PULSER);

    for (index = 0; index < NUM_PULSERS; index++)
    {
        pulser_st * const context = &pulser_state.pulser_contexts[index];

        context->flag = CoCreateFlag(Co_TRUE, Co_FALSE);

        context->timer_context = timer_channel_get(pulser_timer_callback, context);
    }
}

uint32_t pulser_timer_count_get(pulser_st const * const pulser)
{
    return timer_channel_get_current_time(pulser->timer_context);
}

void print_pulse_details(void)
{
#if defined(PULSE_DEBUG)
    size_t index;

    for (index = 0; index < NUM_PULSERS; index++)
    {
        pulser_st * const context = &pulser_state.pulser_contexts[index];

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
        pulser_st * const context = &pulser_state.pulser_contexts[index];

        context->systick_at_start = 0;
        context->systick_at_gpio_off = 0;
    }
#endif
}

