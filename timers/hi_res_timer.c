#include <stdlib.h>
#include <stdbool.h>

#include <stm32f4xx_tim.h>
#include <stm32f4xx_rcc.h>

static void (* callback)(void);

#define HIRESTIM			TIM2
#define HIRESTIM_IRQn		TIM2_IRQn
#define HIRESTTIM_CLK		RCC_APB1Periph_TIM2
#define HIRESTIM_IRQHandler TIM2_IRQHandler

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
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

    NVIC_Init(&NVIC_InitStructure);
}

void HIRESTIM_IRQHandler(void)
{
    TIM_ClearITPendingBit(HIRESTIM, TIM_IT_Update);
    if (callback != NULL)
    {
        callback();
    }
}

uint32_t hi_res_counter_val(void)
{
    return HIRESTIM->CNT;
}

void initHiResTimer(uint32_t periodMicrosecs, void (* appCallback)(void))
{
    callback = appCallback;
    /* enable timer clock */
    RCC_APB1PeriphClockCmd(HIRESTTIM_CLK, ENABLE);

    initTimerTimeBase(HIRESTIM, 0xffffffff, 1000000);

    //initTimerNVIC(HIRESTIM_IRQn);

    /* enable update interrupt */
    //TIM_ITConfig(HIRESTIM, TIM_IT_Update, ENABLE);

    /* start the timer */
    TIM_Cmd(HIRESTIM, ENABLE);
}


