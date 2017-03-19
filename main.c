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
#include <OsArch.h>

#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include <stm32f4xx_tim.h>

#include "stm32f4xx_rcc.h"
#include "stm32f4xx_exti.h"
#include "stm32f4xx_syscfg.h"
#include "usart.h"

#include "timed_events.h"
#include "pulser.h"
#include "injector_control.h"
#include "ignition_control.h"
#include "trigger_input.h"
#include "leds.h"
#include "hi_res_timer.h"
#include "serial_task.h"
#include "queue.h"
#include "utils.h"

#include <math.h>
#include <inttypes.h>

/*---------------------------- Symbol Define -------------------------------*/


/* Private typedef -----------------------------------------------------------*/

/*---------------------------- Variable Define -------------------------------*/

/* TODO: Create an engine context structure to hold all runtime information.
 */

static trigger_wheel_36_1_context_st * trigger_context;

#if 0
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
    /* Add IRQ vector to NVIC */
    /* PA0 is connected to EXTI_Line0, which has EXTI0_IRQn vector */
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
#else
/* Button GPIO used by crank signal. The function also appears to be enabling interrupts on A5, not A), which is what the button is conected to.
*/
#endif

unsigned int get_engine_cycle_degrees(void)
{
    /* TODO: Calculate from configuration (two stroke/four stroke). */
    return 720;
}

float get_config_injector_close_angle(void)
{
    /* TODO: Calculate from configuration. */

    return -50.0;
}

float current_engine_cycle_angle_get(void)
{
    return trigger_36_1_engine_cycle_angle_get(trigger_context);
}

float get_ignition_advance(void)
{
    /* TODO get from configuration */
    return 10.0; /* NB - value is in degrees BTDC. */
}

uint32_t get_ignition_dwell_us(void)
{
    return 3000;
}

float get_ignition_maximum_advance(void)
{
    return 50.0; /* Debug. */

}

#define MAIN_TASK_STACK_SIZE 1024
static __attribute((aligned(8))) OS_STK main_task_stack[MAIN_TASK_STACK_SIZE];
static void main_task(void * arg)
{
    UNUSED(arg);

    init_trigger_signals(trigger_context);

    while (1)
    {
        CoTickDelay(1000000);
    }
}

int main(void)
{
    SystemInit();

    init_leds();

    initHiResTimer(TIMER_FREQUENCY);

    //init_button();

    timed_events_init(TIMER_FREQUENCY);

    CoInitOS(); /*!< Initialise CoOS */

    initSerialTask();

    init_pulsers();

    trigger_context = trigger_36_1_init();

    injection_initialise(trigger_context);
    ignition_initialise(trigger_context);

    /* XXX signal processing should start after the RTOS starts. 
     */
    CoCreateTask(main_task,
                 NULL,
                 1,
                 &main_task_stack[MAIN_TASK_STACK_SIZE - 1],
                 MAIN_TASK_STACK_SIZE); 

    fprintf(stderr, "CoOS RTOS: Starting scheduler\r\n");

    CoStartOS(); /* Start scheduler. */

}

