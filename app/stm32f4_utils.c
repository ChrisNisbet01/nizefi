#include "stm32f4_utils.h"

#include "stm32f4xx_rcc.h"

void stm32f4_enable_IRQ(uint_fast8_t const irq,
                        uint_fast8_t const priority,
                        uint_fast8_t const sub_priority)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    NVIC_InitStructure.NVIC_IRQChannel = irq;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = priority;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = sub_priority;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

    NVIC_Init(&NVIC_InitStructure);
}

void stm32f4_disable_IRQ(uint_fast8_t const irq)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    NVIC_InitStructure.NVIC_IRQChannel = irq;
    NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;

    NVIC_Init(&NVIC_InitStructure);
}

/* Configure a timer to run with the specified period and 
 * frequency. This function does not enable the timer, nor stop 
 * it before it is configured. 
 */
void stm32f4_timer_configure(TIM_TypeDef * tim, uint_fast32_t period, uint_fast32_t frequency_hz, bool const use_PCLK2)
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


