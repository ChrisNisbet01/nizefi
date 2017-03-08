#include "timed_events.h"
#include "queue.h"

#include <stm32f4xx_tim.h>
#include <stdbool.h>
#include <stddef.h>

typedef void (* timed_event_handler)(void * arg);
typedef struct timer_st timer_st; 

typedef struct capture_compare_config_st
{
    uint32_t const capture_compare_interrupt;
    uint32_t const capture_compare_event_source;
    void (* TIM_SetCompare)(TIM_TypeDef * TIMx, uint32_t Compare);
    uint32_t (* TIM_GetCapture)(TIM_TypeDef * TIMx); 
} capture_compare_config_st;

typedef enum capture_compare_index_t
{
    capture_1_index,
    capture_2_index,
    capture_3_index,
    capture_4_index
} capture_compare_index_t;

struct timer_channel_context_st
{
    volatile uint32_t * ccr;

    LIST_ENTRY(timer_channel_context_st) entry;
    bool in_use;

    timer_st const * timer;

    capture_compare_config_st const * capture_config;

    bool free_after_event; /* Set if the user attempts to free the timer while it is still in use. */

    timed_event_handler handler;
    void * arg;

};

typedef struct timer_context_st
{
    uint32_t frequency;
    LIST_HEAD(,timer_channel_context_st) used_timer_list;
    LIST_HEAD(,timer_channel_context_st) unused_timer_list;
} timer_context_st;

struct timer_st
{
    TIM_TypeDef * TIM;
    void (* RCC_APBPeriphClockCmd)(uint32_t RCC_APB1Periph, FunctionalState NewState);
    uint32_t RCC_APBPeriph;
    uint8_t IRQ_channel;
    bool use_PCLK2;

    size_t num_channels;
    timer_channel_context_st * channels;
};

static capture_compare_config_st capture_compare_configs[] =
{
    [capture_1_index] =
    {
        .capture_compare_interrupt = TIM_IT_CC1,
        .capture_compare_event_source = TIM_EventSource_CC1,
        .TIM_SetCompare = TIM_SetCompare1,
        .TIM_GetCapture = TIM_GetCapture1
    },
    [capture_2_index] =
    {
        .capture_compare_interrupt = TIM_IT_CC2,
        .capture_compare_event_source = TIM_EventSource_CC2,
        .TIM_SetCompare = TIM_SetCompare2,
        .TIM_GetCapture = TIM_GetCapture2
    },
    [capture_3_index] =
    {
        .capture_compare_interrupt = TIM_IT_CC3,
        .capture_compare_event_source = TIM_EventSource_CC3,
        .TIM_SetCompare = TIM_SetCompare3,
        .TIM_GetCapture = TIM_GetCapture3
    },
    [capture_4_index] =
    {
        .capture_compare_interrupt = TIM_IT_CC4,
        .capture_compare_event_source = TIM_EventSource_CC4,
        .TIM_SetCompare = TIM_SetCompare4,
        .TIM_GetCapture = TIM_GetCapture4
    }
};

static timer_channel_context_st tim1_timer_channel_contexts[] =
{
    {
        .capture_config = &capture_compare_configs[capture_1_index]
    },
    {
        .capture_config = &capture_compare_configs[capture_2_index]
    },
    {
        .capture_config = &capture_compare_configs[capture_3_index]
    },
    {
        .capture_config = &capture_compare_configs[capture_4_index]
    }
};

static timer_channel_context_st tim3_timer_channel_contexts[] =
{
    {
        .capture_config = &capture_compare_configs[capture_1_index]
    },
    {
        .capture_config = &capture_compare_configs[capture_2_index]
    },
    {
        .capture_config = &capture_compare_configs[capture_3_index]
    },
    {
        .capture_config = &capture_compare_configs[capture_4_index]
    }
};

static timer_channel_context_st tim4_timer_channel_contexts[] =
{
    {
        .capture_config = &capture_compare_configs[capture_1_index]
    },
    {
        .capture_config = &capture_compare_configs[capture_2_index]
    },
    {
        .capture_config = &capture_compare_configs[capture_3_index]
    },
    {
        .capture_config = &capture_compare_configs[capture_4_index]
    }
};

static timer_channel_context_st tim8_timer_channel_contexts[] =
{
    {
        .capture_config = &capture_compare_configs[capture_1_index]
    },
    {
        .capture_config = &capture_compare_configs[capture_2_index]
    },
    {
        .capture_config = &capture_compare_configs[capture_3_index]
    },
    {
        .capture_config = &capture_compare_configs[capture_4_index]
    }
};

typedef enum timer_index_t
{
    timer_1_index,
    timer_3_index,
    timer_4_index,
    timer_8_index,
} timer_index_t;

static timer_st const timers[] =
{
    [timer_1_index] = 
    {
        .TIM = TIM1,
        .RCC_APBPeriphClockCmd = RCC_APB2PeriphClockCmd,
        .RCC_APBPeriph = RCC_APB2Periph_TIM1,
        .IRQ_channel = TIM1_CC_IRQn,
        .num_channels = 4,
        .channels = tim1_timer_channel_contexts,
        .use_PCLK2 = true
    },
    [timer_3_index] =
    {
        .TIM = TIM3,
        .RCC_APBPeriphClockCmd = RCC_APB1PeriphClockCmd,
        .RCC_APBPeriph = RCC_APB1Periph_TIM3,
        .IRQ_channel = TIM3_IRQn,
        .num_channels = 4,
        .channels = tim3_timer_channel_contexts,
        .use_PCLK2 = false
    },
    [timer_4_index] =
    {
        .TIM = TIM4,
        .RCC_APBPeriphClockCmd = RCC_APB1PeriphClockCmd,
        .RCC_APBPeriph = RCC_APB1Periph_TIM4,
        .IRQ_channel = TIM4_IRQn,
        .num_channels = 4,
        .channels = tim4_timer_channel_contexts,
        .use_PCLK2 = false
    },
    [timer_8_index] =
    {
        .TIM = TIM8,
        .RCC_APBPeriphClockCmd = RCC_APB2PeriphClockCmd,
        .RCC_APBPeriph = RCC_APB2Periph_TIM8,
        .IRQ_channel = TIM8_CC_IRQn,
        .num_channels = 4,
        .channels = tim8_timer_channel_contexts,
        .use_PCLK2 = true
    }
};
#define NUM_TIMERS (sizeof timers / sizeof timers[0])

static timer_context_st timer_context;

static void initTimerTimeBase(TIM_TypeDef * tim, uint_fast16_t period, uint_fast32_t frequency_hz, bool const use_PCLK2)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    uint32_t CLK_Frequency;

    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);

    RCC_ClocksTypeDef clocks;
    uint32_t multiplier;

    RCC_GetClocksFreq(&clocks);

    if (use_PCLK2)
    {
        multiplier = 2;
        CLK_Frequency =  multiplier * clocks.PCLK2_Frequency;
    }
    else
    {
        if (clocks.PCLK1_Frequency == clocks.SYSCLK_Frequency)
        {
            multiplier = 1;
        }
        else
        {
            multiplier = 2;
        }
        CLK_Frequency =  multiplier * clocks.PCLK1_Frequency;
    }

    TIM_TimeBaseStructure.TIM_Prescaler = (CLK_Frequency / frequency_hz) - 1;

    TIM_TimeBaseStructure.TIM_Period = period;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

    TIM_TimeBaseInit(tim, &TIM_TimeBaseStructure);
}

static void timer_init(timer_st const * const timer, uint32_t frequency)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    /* TIMx clock enable */
    timer->RCC_APBPeriphClockCmd(timer->RCC_APBPeriph, ENABLE);

    initTimerTimeBase(timer->TIM, 0xffff, frequency, timer->use_PCLK2);

    /* Enable TIM4 Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = timer->IRQ_channel;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 4;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    /* Enable the timer. */
    TIM_Cmd(timer->TIM, ENABLE);
}

void timed_events_init(uint32_t timer_frequency)
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
            capture_compare_config_st const * const capture_config = channel->capture_config;

            channel->timer = timer;
            LIST_INSERT_HEAD(&timer_context.unused_timer_list, channel, entry);

            /* Disable the channel interrupts before getting the timer up 
             * and running. 
             */
            TIM_ITConfig(timer->TIM, capture_config->capture_compare_interrupt, DISABLE);

            timer_init(timer, timer_frequency);
        }
    }
}

timer_channel_context_st * timer_channel_get(void (* const cb)(void * const arg), void * const arg)
{
    timer_channel_context_st * channel;

    channel = LIST_FIRST(&timer_context.unused_timer_list);

    if (channel == NULL)
    {
        goto done;
    }

    LIST_REMOVE(channel, entry);
    channel->in_use = true;
    channel->handler = cb;
    channel->arg = arg;
    LIST_INSERT_HEAD(&timer_context.used_timer_list, channel, entry);

done:
    return channel;
}

void timer_channel_free(timer_channel_context_st * const channel)
{
    /* TODO: ensure the channel is in the inuse list. */

    LIST_REMOVE(channel, entry);
    channel->in_use = false;
    LIST_INSERT_HEAD(&timer_context.unused_timer_list, channel, entry);
}

void timer_channel_schedule_followup_event(timer_channel_context_st * const channel, uint32_t const delay_us)
{
    capture_compare_config_st const * const capture_config = channel->capture_config; 
    /* Assumes that the capture interrupt is still enabled. */
    TIM_TypeDef * const TIMx = channel->timer->TIM;

    capture_config->TIM_SetCompare(TIMx, (uint16_t)(capture_config->TIM_GetCapture(TIMx) + delay_us));
}

void timer_channel_schedule_new_event(timer_channel_context_st * const channel, uint32_t const delay_us)
{
    capture_compare_config_st const * const capture_config = channel->capture_config;
    TIM_TypeDef * const TIMx = channel->timer->TIM;
    uint32_t const cnt = TIM_GetCounter(TIMx);

    TIM_ITConfig(TIMx, capture_config->capture_compare_interrupt, DISABLE);
    TIM_ClearITPendingBit(TIMx, capture_config->capture_compare_interrupt);

    capture_config->TIM_SetCompare(TIMx, (uint16_t)(cnt + delay_us));

    TIM_ITConfig(TIMx, capture_config->capture_compare_interrupt, ENABLE);
}

void timer_channel_schedule_new_based_event(timer_channel_context_st * const channel, uint32_t const base, uint32_t const delay_us)
{
    capture_compare_config_st const * const capture_config = channel->capture_config;
    TIM_TypeDef * const TIMx = channel->timer->TIM;

    TIM_ITConfig(TIMx, capture_config->capture_compare_interrupt, DISABLE);
    TIM_ClearITPendingBit(TIMx, capture_config->capture_compare_interrupt);

    capture_config->TIM_SetCompare(TIMx, (uint16_t)(base + delay_us));

    TIM_ITConfig(TIMx, capture_config->capture_compare_interrupt, ENABLE);
}

uint32_t timer_channel_get_current_time(timer_channel_context_st * const channel)
{
    TIM_TypeDef * const TIMx = channel->timer->TIM;

    return TIM_GetCounter(TIMx);
}

void timer_channel_disable(timer_channel_context_st * const channel)
{
    capture_compare_config_st const * const capture_config = channel->capture_config;
    TIM_TypeDef * const TIMx = channel->timer->TIM;

    TIM_ITConfig(TIMx, capture_config->capture_compare_interrupt, DISABLE);
}

static void TIM_Handle_CC_IRQ(TIM_TypeDef * const TIMx, timer_channel_context_st * const channel)
{
    capture_compare_config_st const * const capture_config = channel->capture_config;

    if (TIM_GetITStatus(TIMx, capture_config->capture_compare_interrupt) != RESET)
    {
        TIM_ClearITPendingBit(TIMx, capture_config->capture_compare_interrupt);
        if (channel->handler != NULL)
        {
            /* It is left to the handler to stop the interrupts or 
             * schedule another event. 
             */
            channel->handler(channel->arg);
        }
    }
}

static void TIM_IRQ_Handler(timer_st const * const timer)
{
    TIM_TypeDef * const TIMx = timer->TIM;
    size_t index;

    for (index = 0; index < timer->num_channels; index++)
    {
        timer_channel_context_st * const channel = &timer->channels[index];

        TIM_Handle_CC_IRQ(TIMx, channel);
    }
}

void TIM3_IRQHandler(void)
{
    timer_st const * const timer = &timers[timer_3_index];

    TIM_IRQ_Handler(timer);
}

void TIM4_IRQHandler(void)
{
    timer_st const * const timer = &timers[timer_4_index];

    TIM_IRQ_Handler(timer);
}

void TIM1_CC_IRQHandler(void)
{
    timer_st const * const timer = &timers[timer_1_index];

    TIM_IRQ_Handler(timer);
}

void TIM8_CC_IRQHandler(void)
{
    timer_st const * const timer = &timers[timer_8_index];

    TIM_IRQ_Handler(timer);
}
