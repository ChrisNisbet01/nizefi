/**
  ******************************************************************************
  * @file    main.c
  * @author  psavr@gmx.ch
  * @version V1.0
  * @date    17-Oktober-2012
  * @brief   Demo-Application for STM32F4-Discovery Board
  *          including RTOS CoOS V1.1.4 (2011.04.20)
  *          including printf() for USART2 (=Default) or USART3
  ******************************************************************************
  * @attention
  *
  * This Example is maintained under the PsAvr "BEERWARE-LICENSE"
  * Feel free to use it, to modify it and to distribute it. If you like
  * it and we meet us once in a pub or a bar, feel free to spend me a beer!
  *
  * I would appreciate it very much, if you would return any corrections
  * and/or improvements back to me and/or to the CooCox community.
  ******************************************************************************
  */

/*---------------------------- Include ---------------------------------------*/
#include <stdio.h>
#include <CoOS.h>
#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include <stm32f4xx_tim.h>

#include "stm32f4xx_rcc.h"
#include "stm32f4xx_exti.h"
#include "stm32f4xx_syscfg.h"
#include "usart.h"
#include <math.h>

#include "hi_res_timer.h"
#include "serial_task.h"

/*---------------------------- Symbol Define -------------------------------*/
#define STACK_SIZE_TASKC 1024              /*!< Define "taskC" task size */
#define STACK_SIZE_TASKD 1024              /*!< Define "taskD" task size */

/* Private typedef -----------------------------------------------------------*/
GPIO_InitTypeDef GPIO_InitStructure;
/*---------------------------- Variable Define -------------------------------*/
static __attribute((aligned(8))) OS_STK taskC_stk[STACK_SIZE_TASKC]; /*!< Define "taskC" task stack */
static __attribute((aligned(8))) OS_STK taskD_stk[STACK_SIZE_TASKD]; /*!< Define "taskD" task stack */

static void common_thread_task(char const * const task_name, 
                               unsigned int gpio_pin, 
                               unsigned int const delay_ticks)
{
    printf("CoOS task %s started\r\n", task_name);
    while (1)
    {
        //GPIO_ToggleBits(GPIOD, gpio_pin); 
        CoTickDelay(delay_ticks);
    }
}

/* Handle PA0 interrupt */
void EXTI0_IRQHandler(void) {
    /* Make sure that interrupt flag is set */
    if (EXTI_GetITStatus(EXTI_Line0) != RESET) {
        /* Clear interrupt flag */

        EXTI_ClearITPendingBit(EXTI_Line0);
        /* Do your stuff when PA0 is changed */
        //GPIO_ToggleBits(GPIOD, GPIO_Pin_15);
    }
}

void EXTI9_5_IRQHandler(void)
{
    /* Make sure that interrupt flag is set */
    if (EXTI_GetITStatus(EXTI_Line5) != RESET) {
        /* Clear interrupt flag */
        EXTI_ClearITPendingBit(EXTI_Line5);

        /* Do your stuff when PA0 is changed */
        //GPIO_ToggleBits(GPIOD, GPIO_Pin_15);
    }
}

static void init_button(void)
{
    /* Set variables used */
    GPIO_InitTypeDef GPIO_InitStruct;
    EXTI_InitTypeDef EXTI_InitStruct;
    NVIC_InitTypeDef NVIC_InitStruct;
    
    /* Enable clock for GPIOA */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    /* Enable clock for SYSCFG */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
    
    /* Set pin as input */
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    /* Tell system that you will use PA0 for EXTI_Line0 */
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource0);
    
    /* PA0 is connected to EXTI_Line0 */
    EXTI_InitStruct.EXTI_Line = EXTI_Line0;
    /* Enable interrupt */
    EXTI_InitStruct.EXTI_LineCmd = ENABLE;
    /* Interrupt mode */
    EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
    /* Triggers on rising and falling edge */
    EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
    /* Add to EXTI */
    EXTI_Init(&EXTI_InitStruct);
 
    /* Add IRQ vector to NVIC */
    /* PD0 is connected to EXTI_Line0, which has EXTI0_IRQn vector */
    NVIC_InitStruct.NVIC_IRQChannel = EXTI0_IRQn;
    /* Set priority */
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x00;
    /* Set sub priority */
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0x00;
    /* Enable interrupt */
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    /* Add to NVIC */
    NVIC_Init(&NVIC_InitStruct);
}

static void init_crank_signal(void)
{
    /* Set variables used */
    GPIO_InitTypeDef GPIO_InitStruct;
    EXTI_InitTypeDef EXTI_InitStruct;
    NVIC_InitTypeDef NVIC_InitStruct;

    /* Enable clock for GPIOA */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    /* Enable clock for SYSCFG */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

    /* Set pin as input */
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_5;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* Tell system that you will use PA5 for EXTI_Line5 */
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource5);

    /* PA5 is connected to EXTI_Line5 */
    EXTI_InitStruct.EXTI_Line = EXTI_Line5;
    /* Enable interrupt */
    EXTI_InitStruct.EXTI_LineCmd = ENABLE;
    /* Interrupt mode */
    EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
    /* Triggers on rising and falling edge */
    EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Falling;
    /* Add to EXTI */
    EXTI_Init(&EXTI_InitStruct);

    /* Add IRQ vector to NVIC */
    /* PA5 is connected to EXTI_Line5-9, which has EXTI1_IRQn vector */
    NVIC_InitStruct.NVIC_IRQChannel = EXTI9_5_IRQn;
    /* Set priority */
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x00;
    /* Set sub priority */
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0x00;
    /* Enable interrupt */
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    /* Add to NVIC */
    NVIC_Init(&NVIC_InitStruct);
}

static void init_leds(void)
{
    /* GPIOD Periph clock enable */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

    /* Configure PD13, PD14 and PD15 in output push-pull mode */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14  | GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    GPIO_PinAFConfig(GPIOD, GPIO_PinSource12, GPIO_AF_TIM4);

    /* Configure PD12 to be connected to TIM4 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
}

uint16_t _pulse_width_us; 
uint16_t _period_us;
volatile uint32_t _pulses;
volatile bool _pulse_set;

void TIM4_IRQHandler(void)
{
    TIM_TypeDef * TIMx = TIM4;

    if (TIM_GetITStatus(TIMx, TIM_IT_CC1) != RESET)
    {
        TIM_ClearITPendingBit(TIMx, TIM_IT_CC1);
        if (_pulse_set)
        {
            _pulse_set = false;
            GPIO_ResetBits(GPIOD, GPIO_Pin_14);

            if (_pulses > 0)
            {
                _pulses--;
            }
            if (_pulses == 0)
            {
                TIM_ITConfig(TIMx, TIM_IT_CC1, DISABLE);
            }
            else
            {
                TIM_SetCompare1(TIMx, (uint16_t)(TIM_GetCounter(TIMx) + _period_us));
            }
        }
        else
        {
            _pulse_set = true;
            TIM_SetCompare1(TIMx, TIM_GetCounter(TIMx) + _pulse_width_us);

            GPIO_SetBits(GPIOD, GPIO_Pin_14);
        }
    }
}

void timer_ccr_init(TIM_TypeDef * TIMx)
{
  TIM_OCInitTypeDef  TIM_OCInitStructure;

  /* always initialise local variables before use */
  TIM_OCStructInit (&TIM_OCInitStructure);

  /* just use basic Output Compare Mode*/
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Toggle;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;

  TIM_OC1Init(TIMx, &TIM_OCInitStructure);
  TIM_OC1PreloadConfig(TIMx, TIM_OCPreload_Disable);
}

void pulse_start(uint32_t pulses, uint16_t pulse_us, uint16_t period_us)
{
    TIM_TypeDef * TIMx = TIM4;

    _pulse_width_us = pulse_us;
    _period_us = period_us - pulse_us;
    _pulses = pulses;
    _pulse_set = false;

    TIM_ClearITPendingBit(TIMx, TIM_IT_CC1);
    TIM_ITConfig(TIMx, TIM_IT_CC1, ENABLE);
    TIM_GenerateEvent(TIMx, TIM_EventSource_CC1);
}
 
void taskC(void * pdata)
{
    (void)pdata;
    while (true)
    {
        if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0))
        {
            pulse_start(3, 1000, 4000);
        }
        CoTickDelay(CFG_SYSTICK_FREQ / 4);
    }
}

void taskD(void * pdata)
{
    (void)pdata;
    //common_thread_task("D", GPIO_Pin_13, CFG_SYSTICK_FREQ);
    while (true)
    {
        CoTickDelay(CFG_SYSTICK_FREQ);
    }
}

void hi_res_tick(void)
{
    GPIO_ToggleBits(GPIOD, GPIO_Pin_15);
}

 
void pulse_cfg(void)
{
    TIM_TypeDef * TIMx = TIM4; 
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    /* TIM4 clock enable */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

    /* Time base configuration */
    TIM_TimeBaseStructure.TIM_Period = 65535;
    TIM_TimeBaseStructure.TIM_Prescaler = (84000000 / 5000) - 1; //84 - 1; //Prescale clock to generate 1MHz
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIMx, &TIM_TimeBaseStructure);

    /* Enable TIM4 Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 4;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    TIM_ITConfig(TIMx, TIM_IT_CC1, DISABLE);
    /* TIM4 enable counter */
    TIM_Cmd(TIMx, ENABLE);

    timer_ccr_init(TIMx); 


}

void init_pulses(void)
{
    /* Initialise system */

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4); //4 bits for pre-emption priority 0 bits for subpriority
    pulse_cfg();
}



int main(void)
{
    SystemInit();

    init_leds();
    initHiResTimer(1000000, hi_res_tick);

    init_button();
    init_crank_signal();

    init_pulses();

    CoInitOS(); /*!< Initialise CoOS */

    /*!< Create three tasks	*/
    CoCreateTask(taskC, 0, 1, &taskC_stk[STACK_SIZE_TASKC - 1], STACK_SIZE_TASKC);
    CoCreateTask(taskD, 0, 2, &taskD_stk[STACK_SIZE_TASKD - 1], STACK_SIZE_TASKD);

    initSerialTask();

    fprintf(stderr, "CoOS RTOS: Starting scheduler\r\n");

    CoStartOS(); /*!< Start multitask	           */

    //printf("CoOS RTOS: Scheduler stopped\n");
    while (1)
    {
    }
}

