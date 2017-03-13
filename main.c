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
#include "trigger_input.h"
#include "leds.h"
#include "hi_res_timer.h"
#include "serial_task.h"
#include "queue.h"

#include <math.h>

/*---------------------------- Symbol Define -------------------------------*/


/* Private typedef -----------------------------------------------------------*/

/*---------------------------- Variable Define -------------------------------*/
GPIO_InitTypeDef GPIO_InitStructure;

/* TODO: Create an engine context structure to hold all runtime information.
 */
static injector_output_st * injectors[INJECTOR_MAX];
static ignition_output_st * ignitions[IGNITION_MAX];

static trigger_wheel_36_1_context_st * trigger_context;

static unsigned int num_injectors_get(void)
{
    /* XXX - Get the value from the configuration. */
    return 4;
}

static unsigned int num_ignition_outputs_get(void)
{
    /* XXX - Get the value from the configuration. */
    return 4;
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

 
void injector_pulse_callback(float const crank_angle, 
                             uint32_t timestamp,
                             void * const user_arg)
{
    injector_output_st * const injector = user_arg;
    (void)crank_angle;
    (void)timestamp; 

    injector_pulse_schedule(injector, 100, 4000);
}

void ignition_pulse_callback(float const crank_angle,
                             uint32_t timestamp,
                             void * const user_arg)
{
    ignition_output_st * const ignition = user_arg;
    (void)crank_angle;
    (void)timestamp; 

    ignition_pulse_schedule(ignition, 100, 2000);
}

static void get_injector_outputs(void)
{
    size_t index;

    for (index = 0; index < num_injectors_get(); index++)
    {
        injectors[index] = injector_output_get();
    }
}

static void get_ignition_outputs(void)
{
    size_t index;

    for (index = 0; index < num_ignition_outputs_get(); index++)
    {
        ignitions[index] = ignition_output_get();
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

    get_injector_outputs();
    get_ignition_outputs();

    trigger_context = trigger_36_1_init();

    /* temp debug do some output pulses at various times in the 
     * engine cycle. 
     */
    trigger_36_1_register_callback(trigger_context, 0.0, injector_pulse_callback, injectors[0]);
    trigger_36_1_register_callback(trigger_context, 180.0, injector_pulse_callback, injectors[1]);
    trigger_36_1_register_callback(trigger_context, 360.0, injector_pulse_callback, injectors[2]);
    trigger_36_1_register_callback(trigger_context, 540.0, injector_pulse_callback, injectors[3]);

    trigger_36_1_register_callback(trigger_context, 20.0, ignition_pulse_callback, ignitions[0]);
    trigger_36_1_register_callback(trigger_context, 200.0, ignition_pulse_callback, ignitions[1]);
    trigger_36_1_register_callback(trigger_context, 380.0, ignition_pulse_callback, ignitions[2]);
    trigger_36_1_register_callback(trigger_context, 560.0, ignition_pulse_callback, ignitions[3]);

    init_trigger_signals(trigger_context); /* Done after CoInitOS() as it uses CoOS resources. */


    fprintf(stderr, "CoOS RTOS: Starting scheduler\r\n");

    CoStartOS(); /* Start scheduler. */

    //printf("CoOS RTOS: Scheduler stopped\n");
    while (1)
    {
    }
}

