#include "pulser.h"
#include "timed_events.h"
#include "leds.h"
#include "hi_res_timer.h"

#include <coocox.h>

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <inttypes.h>
#include <stdio.h>

#define STACK_SIZE_PULSER 1024
#define NUM_PULSERS 16 /* Must be >= number of injectors + number of ignition outputs && <= number of timer channels. */

#define MAXIMUM_TIMER_LENGTH_SECS 20

/* If any scheduled timer specifies a time great then this the 
 * delay is performed in stages to avoid timer overflow 
 * issues. 
 */
#define MAX_TIMER_TICKS_BEFORE_STAGING 40000UL
/* If staging to avoid overflow, this is the amount of time to 
 * use between stages. This value should be less than 
 * MAX_TIMER_TICKS_BEFORE_STAGING so that if a timer time is 
 * only slightly greater than MAX_TIMER_TICKS_BEFORE_STAGING 
 * there will still be a reasonable time left to schedule the 
 * next event. 
 */
#define TIMER_STAGE_TICKS 30000UL

typedef void (* state_handler)(pulser_st * context);

struct pulser_st
{
    OS_FlagID event_completion_flag; /* Used to signal the pulser task to run from the ISR. */
    uint32_t event_completion_bit;

    /* ISR level and task level state machine handlers. */
    volatile state_handler isr_state_handler;
    volatile state_handler task_state_handler;

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

    uint_fast16_t timer_end_us; /* time when the current event will occur. */

    uint_fast32_t initial_delay_left_us;

    volatile bool pending_event;
    volatile uint32_t pending_base_time;
    volatile uint32_t pending_initial_delay_us;
    volatile uint_fast16_t pending_pulse_us;
    volatile int_fast16_t event_overlap; /* Overlap between end of current event and base time of the next. */
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

    isr_SetFlag(context->event_completion_flag);

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

    if (context->initial_delay_left_us > MAX_TIMER_TICKS_BEFORE_STAGING)
    {
        context->initial_delay_left_us -= TIMER_STAGE_TICKS;
        context->timer_end_us += TIMER_STAGE_TICKS;

        timer_channel_schedule_followup_event(context->timer_context, TIMER_STAGE_TICKS);
    }
    else
    {
        context->isr_state_handler =  pulser_initial_delay_isr_handler;
        context->task_state_handler = pulser_initial_delay_task_handler;
        context->timer_end_us += context->initial_delay_left_us;

        timer_channel_schedule_followup_event(context->timer_context, context->initial_delay_left_us);
        context->initial_delay_left_us = 0;
    }
}

static void pulser_initial_delay_isr_handler(pulser_st * context)
{
    /* The initial delay is over. */
    context->active_callback(context->user_arg);
}

static void pulser_initial_delay_task_handler(pulser_st * context)
{
    /* TODO: Add support for pulses longer than 65536 by adding a 
     * new state that schedules follow up events, but leaves the 
     * pulser active. Add similar support for longer initial delays.
     */
    context->isr_state_handler =  pulser_active_isr_handler; 
    context->task_state_handler = pulser_active_task_handler;

    context->timer_end_us += context->active_us;

    timer_channel_schedule_followup_event(context->timer_context, context->active_us);
}

static void pulser_active_isr_handler(pulser_st * context)
{
    /* The pulse time is over. */
    context->inactive_callback(context->user_arg);
}

static void pulser_active_task_handler(pulser_st * context)
{
    context->isr_state_handler = pulser_idle_isr_handler;
    context->task_state_handler = pulser_idle_task_handler;

    timer_channel_disable(context->timer_context);

    /* Schedule the next pulse if one is waiting */
    if (context->pending_event)
    {
        context->pending_event = false;
        pulser_schedule_pulse(context, 
                              context->pending_base_time, 
                              context->pending_initial_delay_us,
                              context->pending_pulse_us);
    }
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

void pulser_schedule_pulse(pulser_st * const context, uint32_t const base_time, uint32_t const initial_delay_us, uint_fast16_t const pulse_us)
{
    uint32_t initial_delay = initial_delay_us;
    context->active_us = pulse_us; /* Remember the pulse width. */

    if (context->task_state_handler == pulser_active_task_handler)
    {
        /* The pulser context hasn't finished with the previous 
         * event. 
         */
        context->pending_base_time = base_time;
        context->pending_initial_delay_us = initial_delay_us;
        context->pending_pulse_us = pulse_us;
        context->event_overlap = (int16_t)(context->timer_end_us - context->pending_base_time);

        context->pending_base_time += context->event_overlap;
        context->pending_initial_delay_us -= context->event_overlap;

        context->pending_event = true; 

        goto done;
    }

    if (initial_delay_us > MAXIMUM_TIMER_LENGTH_SECS * TIMER_FREQUENCY)
    {
        /* Shouldn't happen, but does once when first getting going. 
         * FIXME - XXX - Find out why this happens. Appears to be -ve 
         * initial delay i.e. 0 - injector pulse width. 
         */
        goto done;
    }

    if (initial_delay_us > MAX_TIMER_TICKS_BEFORE_STAGING)
    {
        initial_delay = TIMER_STAGE_TICKS;
        context->initial_delay_left_us = initial_delay_us - TIMER_STAGE_TICKS;
        context->isr_state_handler = pulser_initial_delay_overflow_isr_handler;
        context->task_state_handler = pulser_initial_delay_overflow_task_handler;
    }
    else
    {
        context->initial_delay_left_us = 0;
        context->isr_state_handler = pulser_initial_delay_isr_handler;
        context->task_state_handler = pulser_initial_delay_task_handler; 
    }

    context->timer_end_us = base_time + initial_delay;

    timer_channel_schedule_new_based_event(context->timer_context, base_time, initial_delay);

done:
    return;
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
    pulser_state.active_pulser_flags |= pulser->event_completion_bit;

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

            if ((readyFlags & context->event_completion_bit) != 0)
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

        context->event_completion_flag = CoCreateFlag(Co_TRUE, Co_FALSE);
        context->event_completion_bit = 1 << context->event_completion_flag;

        context->timer_context = timer_channel_get(pulser_timer_callback, context);

        /* If context->timer_context == NULL this is a big problem. 
         * Should log some kind of event and halt the firmware. 
         */

        context->isr_state_handler = pulser_idle_isr_handler;
        context->task_state_handler = pulser_idle_task_handler;
    }
}

uint32_t pulser_timer_count_get(pulser_st const * const pulser)
{
    return timer_channel_get_current_time(pulser->timer_context);
}

void print_pulser_debug(void)
{
    printf("eo: %d id: %"PRIu32"\r\n", 
           (int)pulser_state.pulser_contexts[0].event_overlap,
           (uint32_t)pulser_state.pulser_contexts[0].initial_delay_left_us);
}
