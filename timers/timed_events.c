#include "timed_events.h"
#include "queue.h"

#include <stm32f4xx_tim.h>
#include <stdbool.h>
#include <stddef.h>

typedef void (* timed_event_handler)(void * arg);
typedef struct timer_st timer_st; 

typedef struct timer_channel_context_st
{
    volatile uint32_t * ccr;

    LIST_ENTRY(timer_channel_context_st) entry;
    bool in_use;

    timer_st const * timer;
    uint32_t const capture_compare_interrupt;
    uint32_t const capture_compare_event_source;

    bool free_after_event; /* Set if the user attempts to free the timer while it is still in use. */
    uint32_t last_capture_compare_count; /* TIM counter value at last event. Used as the base when scheduling new events. */

    timed_event_handler handler;
    void * arg;

} timer_channel_context_st;

typedef struct timer_context_st
{
    uint32_t frequency;
    LIST_HEAD(,timer_channel_context_st) used_timer_list;
    LIST_HEAD(,timer_channel_context_st) unused_timer_list;
} timer_context_st;

struct timer_st
{
    TIM_TypeDef const * TIM;
    size_t const num_channels;
    timer_channel_context_st * const channels;
};

static timer_channel_context_st tim1_timer_channel_contexts[] =
{
    {
        .capture_compare_interrupt = TIM_IT_CC1,
        .capture_compare_event_source = TIM_EventSource_CC1
    },
    {
        .capture_compare_interrupt = TIM_IT_CC2,
        .capture_compare_event_source = TIM_EventSource_CC2
    },
    {
        .capture_compare_interrupt = TIM_IT_CC3,
        .capture_compare_event_source = TIM_EventSource_CC3
    },
    {
        .capture_compare_interrupt = TIM_IT_CC4,
        .capture_compare_event_source = TIM_EventSource_CC4
    }
};

static timer_channel_context_st tim3_timer_channel_contexts[] =
{
    {
        .capture_compare_interrupt = TIM_IT_CC1,
        .capture_compare_event_source = TIM_EventSource_CC1
    },
    {
        .capture_compare_interrupt = TIM_IT_CC2,
        .capture_compare_event_source = TIM_EventSource_CC2
    },
    {
        .capture_compare_interrupt = TIM_IT_CC3,
        .capture_compare_event_source = TIM_EventSource_CC3
    },
    {
        .capture_compare_interrupt = TIM_IT_CC4,
        .capture_compare_event_source = TIM_EventSource_CC4
    }
};

static timer_channel_context_st tim4_timer_channel_contexts[] =
{
    {
        .capture_compare_interrupt = TIM_IT_CC1,
        .capture_compare_event_source = TIM_EventSource_CC1
    },
    {
        .capture_compare_interrupt = TIM_IT_CC2,
        .capture_compare_event_source = TIM_EventSource_CC2
    },
    {
        .capture_compare_interrupt = TIM_IT_CC3,
        .capture_compare_event_source = TIM_EventSource_CC3
    },
    {
        .capture_compare_interrupt = TIM_IT_CC4,
        .capture_compare_event_source = TIM_EventSource_CC4
    }
};

static timer_channel_context_st tim8_timer_channel_contexts[] =
{
    {
        .capture_compare_interrupt = TIM_IT_CC1,
        .capture_compare_event_source = TIM_EventSource_CC1
    },
    {
        .capture_compare_interrupt = TIM_IT_CC2,
        .capture_compare_event_source = TIM_EventSource_CC2
    },
    {
        .capture_compare_interrupt = TIM_IT_CC3,
        .capture_compare_event_source = TIM_EventSource_CC3
    },
    {
        .capture_compare_interrupt = TIM_IT_CC4,
        .capture_compare_event_source = TIM_EventSource_CC4
    }
};

static timer_st const timers[] =
{
    {
        .TIM = TIM1,
        .num_channels = 4,
        .channels = tim1_timer_channel_contexts
    },
    {
        .TIM = TIM3,
        .num_channels = 4,
        .channels = tim3_timer_channel_contexts
    },
    {
        .TIM = TIM4,
        .num_channels = 4,
        .channels = tim4_timer_channel_contexts
    },
    {
        .TIM = TIM8,
        .num_channels = 4,
        .channels = tim8_timer_channel_contexts
    }
};
#define NUM_TIMERS (sizeof timers / sizeof timers[0])

static timer_context_st timer_context;

void timed_events_init(size_t timer_frequency)
{
    size_t timer_index;

    /* Set up each of the timers used by this code to run at the 
     * specified frequency. 
     */
    LIST_INIT(&timer_context.unused_timer_list);
    LIST_INIT(&timer_context.used_timer_list);

    for (timer_index = 0; timer_index < NUM_TIMERS; timer_index++)
    {
        timer_st const * timer = &timers[timer_index];

        size_t channel_index;

        for (channel_index = 0; channel_index < timer->num_channels; channel_index++)
        {
            timer_channel_context_st * const channel = &timer->channels[channel_index];

            channel->timer = timer;
            LIST_INSERT_HEAD(&timer_context.unused_timer_list, channel, entry);
        }
    }
}
