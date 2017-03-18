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

typedef void (* state_handler)(pulser_st * pulser);

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
    uint_fast16_t pulse_width_us; /* The period of time the pulse should be active for. */

    uint_fast16_t timer_end_us; /* time when the current event will occur. */

    uint_fast32_t initial_delay_left_us;

    volatile bool pending_event;
    pulser_schedule_st pending_schedule;
};

static void pulser_initial_delay_isr_handler(pulser_st * pulser);
static void pulser_initial_delay_task_handler(pulser_st * pulser);

static void pulser_initial_delay_overflow_isr_handler(pulser_st * pulser);
static void pulser_initial_delay_overflow_task_handler(pulser_st * pulser); 

static void pulser_active_isr_handler(pulser_st * pulser);
static void pulser_active_task_handler(pulser_st * pulser);

typedef struct pulser_state_st
{
    __attribute((aligned(8))) OS_STK pulser_stk[STACK_SIZE_PULSER];

    pulser_st pulsers[NUM_PULSERS];
    size_t next_pulser;
    uint32_t active_pulser_flags;
} pulser_state_st;

static pulser_state_st pulser_state;

static void indicate_timeout(pulser_st * pulser)
{
    CoEnterISR();

    isr_SetFlag(pulser->event_completion_flag);

    CoExitISR();
}

static void pulser_idle_isr_handler(pulser_st * pulser)
{
    (void)pulser;
}

static void pulser_idle_task_handler(pulser_st * pulser)
{
    (void)pulser;
}

static void pulser_initial_delay_overflow_isr_handler(pulser_st * pulser)
{
    (void)pulser;
}

static void pulser_set_state_idle(pulser_st * pulser)
{
    pulser->isr_state_handler = pulser_idle_isr_handler;
    pulser->task_state_handler = pulser_idle_task_handler;

    /* Schedule the next pulse if one is waiting */
    if (pulser->pending_event)
    {
        pulser->pending_event = false;
        pulser_schedule_pulse(pulser,
                              &pulser->pending_schedule);
    }
}

static void pulser_set_state_initial_delay(pulser_st * pulser)
{
    pulser->isr_state_handler =  pulser_initial_delay_isr_handler;
    pulser->task_state_handler = pulser_initial_delay_task_handler;
}

static void pulser_set_state_active(pulser_st * pulser)
{
    pulser->isr_state_handler =  pulser_active_isr_handler;
    pulser->task_state_handler = pulser_active_task_handler;
}

static void pulser_set_state_initial_delay_overflow(pulser_st * pulser)
{
    pulser->isr_state_handler = pulser_initial_delay_overflow_isr_handler;
    pulser->task_state_handler = pulser_initial_delay_overflow_task_handler;
}

static void pulser_initial_delay_overflow_task_handler(pulser_st * pulser)
{
    (void)pulser;

    if (pulser->initial_delay_left_us > MAX_TIMER_TICKS_BEFORE_STAGING)
    {
        pulser->initial_delay_left_us -= TIMER_STAGE_TICKS;
        pulser->timer_end_us += TIMER_STAGE_TICKS;

        timer_channel_schedule_followup_event(pulser->timer_context, TIMER_STAGE_TICKS);
    }
    else
    {
        pulser_set_state_initial_delay(pulser);

        pulser->timer_end_us += pulser->initial_delay_left_us;

        timer_channel_schedule_followup_event(pulser->timer_context, pulser->initial_delay_left_us);
        pulser->initial_delay_left_us = 0;
    }
}

static void pulser_initial_delay_isr_handler(pulser_st * pulser)
{
    /* The initial delay is over. */
    pulser->active_callback(pulser->user_arg);
}

static void pulser_initial_delay_task_handler(pulser_st * pulser)
{
    /* TODO: Add support for pulses longer than 65536 by adding a 
     * new state that schedules follow up events, but leaves the 
     * pulser active.
     */
    pulser_set_state_active(pulser);

    pulser->timer_end_us += pulser->pulse_width_us;

    timer_channel_schedule_followup_event(pulser->timer_context, pulser->pulse_width_us);
}

static void pulser_active_isr_handler(pulser_st * pulser)
{
    /* The pulse time is over. */
    pulser->inactive_callback(pulser->user_arg);
}

static void pulser_active_task_handler(pulser_st * pulser)
{
    timer_channel_disable(pulser->timer_context);

    pulser_set_state_idle(pulser);
}

static void pulser_timer_callback(void * const arg)
{
    pulser_st * const pulser = arg;

    pulser->isr_state_handler(pulser);
    /* We signal the pulser task no matter the current state.
     * To avoid repitition we put the call to signal the pulser task after the state handler. 
     */
    indicate_timeout(pulser);
}

void pulser_schedule_pulse(pulser_st * const pulser, 
                           pulser_schedule_st const * const pulser_schedule)
{
    uint32_t initial_delay;

    /* XXX Need to deal with race conditions here. */
    if (pulser->task_state_handler != pulser_idle_task_handler)
    {
        /* The pulser context hasn't finished with the previous 
         * event. 
         */
        pulser->pending_schedule = *pulser_schedule;
        pulser->pending_event = true; 

        goto done;
    }

    if (pulser_schedule->initial_delay_us > MAXIMUM_TIMER_LENGTH_SECS * TIMER_FREQUENCY)
    {
        /* Shouldn't happen, but does once when first getting going. 
         * FIXME - XXX - Find out why this happens. Appears to be -ve 
         * initial delay i.e. 0 - injector pulse width. 
         */
        goto done;
    }

    if (pulser_schedule->initial_delay_us > MAX_TIMER_TICKS_BEFORE_STAGING)
    {
        pulser->initial_delay_left_us = pulser_schedule->initial_delay_us - TIMER_STAGE_TICKS;
        initial_delay = TIMER_STAGE_TICKS;
        pulser_set_state_initial_delay_overflow(pulser);
    }
    else
    {
        initial_delay = pulser_schedule->initial_delay_us;
        pulser->initial_delay_left_us = 0;
        pulser_set_state_initial_delay(pulser);
    }

    pulser->pulse_width_us = pulser_schedule->pulse_width_us; /* Remember the pulse width. */
    pulser->timer_end_us = pulser_schedule->base_time + initial_delay;

    timer_channel_schedule_new_based_event(pulser->timer_context, pulser_schedule->base_time, initial_delay);

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

    pulser = &pulser_state.pulsers[pulser_state.next_pulser];

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
            pulser_st * const pulser = &pulser_state.pulsers[index];

            if ((readyFlags & pulser->event_completion_bit) != 0)
            {
                pulser->task_state_handler(pulser); 
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
        pulser_st * const pulser = &pulser_state.pulsers[index];

        pulser->event_completion_flag = CoCreateFlag(Co_TRUE, Co_FALSE);
        pulser->event_completion_bit = 1 << pulser->event_completion_flag;
        pulser->pending_event = false;

        pulser->timer_context = timer_channel_get(pulser_timer_callback, pulser);

        /* If pulser->timer_context == NULL this is a big problem. 
         * Should log some kind of event and halt the firmware. 
         */

        pulser_set_state_idle(pulser);
    }
}

uint32_t pulser_timer_count_get(pulser_st const * const pulser)
{
    return timer_channel_get_current_time(pulser->timer_context);
}

void print_pulser_debug(void)
{
    printf("initial delay remaining: %"PRIu32"\r\n", 
           (uint32_t)pulser_state.pulsers[0].initial_delay_left_us);
}
