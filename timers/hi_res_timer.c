#include "hi_res_timer.h"

#include <stdlib.h>
#include <stdbool.h>

#include <stm32f4xx_tim.h>
#include <stm32f4xx_rcc.h>

#define HIRESTIM TIM2
#define HIRESTIM_IRQn TIM2_IRQn
#define HIRESTTIM_CLK RCC_APB1Periph_TIM2
#define HIRESTIM_IRQHandler TIM2_IRQHandler

typedef enum channel_index_t
{
    CH1_IDX,
    MAX_CHANNEL_INDEX
} channel_index_t;

typedef struct channel_config_st {
    uint16_t channel;
    uint16_t interruptBit;
    uint32_t (* TIM_GetCaptureFn)(TIM_TypeDef * TIMx);
} channel_config_st;

/* XXX - Big cleanup required. config doesn't need to contain 
 * the timer as only one is used. 
 * Only certain pins can be used for input capture. It turns out 
 * that PA5 (crank trigger input is attached to CC1 on TIM2, but 
 * the CAM trigger input can not be attached to TIM2 CC at all. 
 */
typedef struct input_capture_config_st
{
    /* GPIO settings */
    GPIO_TypeDef * gpio;
    uint_fast16_t pin;
    uint_fast8_t pinSource;
    uint_fast8_t pinAF;

    uint_fast32_t RCC_AHBPeriph;

    /* Associated timer settings */
    TIM_TypeDef * tim;
    uint_fast8_t irq;
    channel_config_st const * const channel_config;
} input_capture_config_st; 

typedef struct input_capture_context_st
{
    void (* callback)(uint32_t const timestamp);
} input_capture_context_st;

/* Ensure there is one of these configured for each timer channel used. */
static const channel_config_st channel_configs[] = {
    [CH1_IDX] = {TIM_Channel_1, TIM_IT_CC1, TIM_GetCapture1},
};

static const input_capture_config_st input_capture_configs[] =
{
    {
        .gpio = GPIOA,
        .pin = GPIO_Pin_5,
        .pinSource = GPIO_PinSource5,
        .pinAF = GPIO_AF_TIM2,
        .RCC_AHBPeriph = RCC_AHB1Periph_GPIOA,

        /* Timer related */
        .tim = HIRESTIM,
        .irq = HIRESTIM_IRQn,
        .channel_config = &channel_configs[CH1_IDX]
    }
};

#define NUM_TRIGGER_INPUT_CONFIGS	(sizeof(input_capture_configs)/sizeof(input_capture_configs[0]))
#define TRIGGER_INPUT_CONFIG_INDEX(p)	((p)-channel_configs)

static input_capture_context_st input_capture_contexts[1];

/* XXX - Still need to make a common function for all of the 
 * various GPIO init functions to use. 
 */
static void configure_input_trigger_gpio_pin(uint32_t const RCC_AHBPeriph,
                                             GPIO_TypeDef * const port,
                                             uint32_t pin)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    /* Need to put the GPIO into AF mode. No pullups required as 
     * the external wiring will pull the signal up or down. 
     * Ensure the peripheral clock is enabled for this GPIO port. 
     */
    /* Enable the clock for the GPIO port. */
    RCC_AHB1PeriphClockCmd(RCC_AHBPeriph, ENABLE);

    /* Set pin as input */
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_Pin = pin;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL; /* External wiring should always pull one way or the other. */
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;

    GPIO_Init(port, &GPIO_InitStruct);
}


static void initTimerTimeBase(TIM_TypeDef * tim, uint_fast16_t period, uint_fast32_t frequency_hz)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);

    RCC_ClocksTypeDef clocks;
    uint32_t multiplier;
    RCC_GetClocksFreq(&clocks);

    if (clocks.PCLK1_Frequency == clocks.SYSCLK_Frequency)
    {
        multiplier = 1;
    }
    else
    {
        multiplier = 2;
    }
    uint32_t CLK_Frequency =  multiplier * clocks.PCLK1_Frequency;

    RCC_GetClocksFreq(&clocks);

    TIM_TimeBaseStructure.TIM_Prescaler = (CLK_Frequency / frequency_hz) - 1;

    TIM_TimeBaseStructure.TIM_Period = period;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

    TIM_TimeBaseInit(tim, &TIM_TimeBaseStructure);
}

static void initTimerNVIC(uint_fast8_t irq)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    NVIC_InitStructure.NVIC_IRQChannel = irq;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

    NVIC_Init(&NVIC_InitStructure);
}

static void handle_input_capture(channel_config_st const * const channel_config)
{
    size_t const channel_index = TRIGGER_INPUT_CONFIG_INDEX(channel_config);
    input_capture_context_st * const input_capture_context = &input_capture_contexts[channel_index];

    TIM_ClearITPendingBit(HIRESTIM, channel_config->interruptBit);

    if (input_capture_context->callback != NULL)
    {
        uint32_t const timestamp = channel_config->TIM_GetCaptureFn(HIRESTIM); 
        input_capture_context->callback(timestamp);
    }
}

void HIRESTIM_IRQHandler(void)
{
    if (TIM_GetITStatus(HIRESTIM, TIM_IT_CC1))
    {
        TIM_ClearITPendingBit(HIRESTIM, TIM_IT_CC1);
        handle_input_capture(&channel_configs[CH1_IDX]);
    }
}

uint32_t hi_res_counter_val(void)
{
    return HIRESTIM->CNT;
}

#define CRANK_INPUT_CAPTURE_INDEX 0

static void initInputCapture(TIM_TypeDef * tim, uint_fast8_t channel, uint_fast16_t polarity)
{
    TIM_ICInitTypeDef TIM_ICInitStructure;

    TIM_ICStructInit(&TIM_ICInitStructure);
    TIM_ICInitStructure.TIM_Channel = channel;
    TIM_ICInitStructure.TIM_ICPolarity = polarity;
    TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
    TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    TIM_ICInitStructure.TIM_ICFilter = 0x0;

    TIM_ICInit(tim, &TIM_ICInitStructure);
}

/* TODO: Add support enabling and disabling the input trigger 
 * without unconfiguring it. 
 */
static void register_input_trigger_callback(size_t input_capture_index,
                                            void(* callback)(uint32_t const timestamp_us))
{
    input_capture_context_st * const input_capture_context = &input_capture_contexts[input_capture_index];
    input_capture_config_st const * const input_capture_config = &input_capture_configs[input_capture_index];

    input_capture_context->callback = callback;

    configure_input_trigger_gpio_pin(input_capture_config->RCC_AHBPeriph,
                                     input_capture_config->gpio,
                                     input_capture_config->pin);

    /* Connect the timer to the pin. 
     */
    GPIO_PinAFConfig(input_capture_config->gpio, input_capture_config->pinSource, input_capture_config->pinAF); 

    /* TODO: Configurable input capture edge. */
    initInputCapture(input_capture_config->tim, input_capture_config->channel_config->channel, TIM_ICPolarity_Falling);

    /* Enable the interrupts. */
    TIM_ITConfig(input_capture_config->tim, input_capture_config->channel_config->interruptBit, ENABLE);
}

void register_crank_trigger_callback(void (* callback)(uint32_t const timestamp_us))
{
    register_input_trigger_callback(CRANK_INPUT_CAPTURE_INDEX, callback);
}

void initHiResTimer(uint32_t const frequency)
{
    /* enable timer clock */
    RCC_APB1PeriphClockCmd(HIRESTTIM_CLK, ENABLE);

    initTimerTimeBase(HIRESTIM, 0xffffffff, frequency);

    initTimerNVIC(HIRESTIM_IRQn);

    /* start the timer */
    TIM_Cmd(HIRESTIM, ENABLE);
}


