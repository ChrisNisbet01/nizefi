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
#include "injector_output.h"
#include "ignition_output.h"
#include "trigger_wheel_36_1.h"
#include "leds.h"
#include "hi_res_timer.h"
#include "serial_task.h"
#include "queue.h"

#include <math.h>

/*---------------------------- Symbol Define -------------------------------*/
#define STACK_SIZE_TASKC 1024              /*!< Define "taskC" task size */
#define STACK_SIZE_TASKD 1024              /*!< Define "taskD" task size */


/* Private typedef -----------------------------------------------------------*/

/*---------------------------- Variable Define -------------------------------*/
GPIO_InitTypeDef GPIO_InitStructure;
static __attribute((aligned(8))) OS_STK taskC_stk[STACK_SIZE_TASKC]; /*!< Define "taskC" task stack */
static __attribute((aligned(8))) OS_STK taskD_stk[STACK_SIZE_TASKD]; /*!< Define "taskD" task stack */

static injector_output_st * injector_1;
static injector_output_st * injector_2;
static injector_output_st * injector_3;
static injector_output_st * injector_4;
static ignition_output_st * ignition_1;
static trigger_wheel_36_1_context_st * trigger_context; 

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

static void init_leds(void)
{
    /* GPIOD Periph clock enable */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

    /* Configure PD13, PD14 and PD15 in output push-pull mode */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14  | GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

#if 0 /* Used if tying GPIO pins to timers. */
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource12, GPIO_AF_TIM4);

    /* Configure PD12 to be connected to TIM4 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
#endif
}

 
void taskC(void * pdata)
{
    (void)pdata;
    while (true)
    {
        if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0))
        {
            //injector_pulse_schedule(injector_1, 5000, 2500);
            //ignition_pulse_schedule(ignition_1, 50000, 2500);
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

void debug_injector_pulse(void)
{
    //injector_output_st * const injector = injector_1;

    //injector_pulse_schedule(injector, 100, 1000);
}

void injector_pulse_callback(float const crank_angle, 
                             uint32_t timestamp,
                             void * const user_arg)
{
    injector_output_st * const injector = user_arg;

    injector_pulse_schedule(injector, 100, 4000);
}

int main(void)
{
    SystemInit();

    init_leds();
    initHiResTimer(1000000, hi_res_tick);

    //init_button();

    timed_events_init(1000000);    

    CoInitOS(); /*!< Initialise CoOS */

    initSerialTask();

    init_pulsers();

    injector_1 = injector_output_get();
    injector_2 = injector_output_get();
    injector_3 = injector_output_get();
    injector_4 = injector_output_get();
    ignition_1 = ignition_output_get();

    trigger_context = trigger_36_1_init();
    trigger_36_1_register_callback(trigger_context, 0.0, injector_pulse_callback, injector_1);
    trigger_36_1_register_callback(trigger_context, 180.0, injector_pulse_callback, injector_2);
    trigger_36_1_register_callback(trigger_context, 360.0, injector_pulse_callback, injector_3);
    trigger_36_1_register_callback(trigger_context, 540.0, injector_pulse_callback, injector_4);

    init_trigger_signals(trigger_context); /* Done after CoInitOS() as it uses CoOS resources. */

    /* Create some dummy tasks */ 
    CoCreateTask(taskC, 0, 3, &taskC_stk[STACK_SIZE_TASKC - 1], STACK_SIZE_TASKC);
    //CoCreateTask(taskD, 0, 9, &taskD_stk[STACK_SIZE_TASKD - 1], STACK_SIZE_TASKD);

    fprintf(stderr, "CoOS RTOS: Starting scheduler\r\n");

    CoStartOS(); /* Start scheduler. */

    //printf("CoOS RTOS: Scheduler stopped\n");
    while (1)
    {
    }
}

