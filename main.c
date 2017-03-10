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

#include "hi_res_timer.h"
#include "serial_task.h"
#include "queue.h"

#include <math.h>

/*---------------------------- Symbol Define -------------------------------*/
#define STACK_SIZE_TASKC 1024              /*!< Define "taskC" task size */
#define STACK_SIZE_TASKD 1024              /*!< Define "taskD" task size */


typedef struct trigger_signal_st
{
    STAILQ_ENTRY(trigger_signal_st) entry;

    uint32_t timestamp;
} trigger_signal_st;

/* Private typedef -----------------------------------------------------------*/

/*---------------------------- Variable Define -------------------------------*/
GPIO_InitTypeDef GPIO_InitStructure;
static __attribute((aligned(8))) OS_STK taskC_stk[STACK_SIZE_TASKC]; /*!< Define "taskC" task stack */
static __attribute((aligned(8))) OS_STK taskD_stk[STACK_SIZE_TASKD]; /*!< Define "taskD" task stack */

#define TRIGGER_SIGNAL_TASK_STACK_SIZE 1024
static __attribute((aligned(8))) OS_STK trigger_signal_task_stack[TRIGGER_SIGNAL_TASK_STACK_SIZE]; /*!< Define "taskD" task stack */

static injector_output_st * injector_1;
static ignition_output_st * ignition_1; 

#define NUM_TRIGGER_SIGNALS 15
static trigger_signal_st trigger_signals[NUM_TRIGGER_SIGNALS];
static trigger_signal_st * trigger_signal_queue[NUM_TRIGGER_SIGNALS];
OS_EventID trigger_signal_message_queue_id; 
trigger_wheel_36_1_context_st * trigger_context;

static STAILQ_HEAD(, trigger_signal_st) trigger_signal_free_list;

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

static trigger_signal_st * trigger_signal_get(void)
{
    /* This function is called from within an IRQ. Entries are placed back into the queue with interrupts disabled.
    */
    trigger_signal_st * const trigger_signal = STAILQ_FIRST(&trigger_signal_free_list);

    if (trigger_signal != NULL)
    {
        STAILQ_REMOVE_HEAD(&trigger_signal_free_list, entry);
    }

    return trigger_signal;
}

static void trigger_signal_put(trigger_signal_st * const trigger_signal)
{
    IRQ_DISABLE_SAVE();

    STAILQ_INSERT_TAIL(&trigger_signal_free_list, trigger_signal, entry);

    IRQ_ENABLE_RESTORE();
}

/* Handle PA0 interrupt */
void EXTI0_IRQHandler(void) {
    /* Make sure that interrupt flag is set */
    if (EXTI_GetITStatus(EXTI_Line0) != RESET) {
        /* Clear interrupt flag */

        EXTI_ClearITPendingBit(EXTI_Line0);

        /* TODO: configurable edge? */
        if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0) == Bit_RESET) /* Trigger on fallig edge. */
        {
            trigger_signal_st * const trigger_signal = trigger_signal_get();

            if (trigger_signal != NULL)
            {
                trigger_signal->timestamp = hi_res_counter_val();

                CoEnterISR();

                isr_PostQueueMail(trigger_signal_message_queue_id, trigger_signal);

                CoExitISR();
            }
        }

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

static void init_crank_trigger_signal_list(void)
{
    size_t index;

    STAILQ_INIT(&trigger_signal_free_list); 
    for (index = 0; index < NUM_TRIGGER_SIGNALS; index++)
    {
        trigger_signal_st * const trigger_signal = &trigger_signals[index];

        STAILQ_INSERT_TAIL(&trigger_signal_free_list, trigger_signal, entry);
    }
}

void debug_injector_pulse(void)
{
    injector_pulse_schedule(injector_1, 100, 3000);
}

void trigger_signal_task(void * pdata)
{
    unsigned int tooth = 0;
    OS_EventID message_queue_id = *(OS_EventID *)pdata;

    while (1)
    {
        StatusType err;
        trigger_signal_st * trigger_signal;
        uint32_t timestamp;

        trigger_signal =  CoPendQueueMail(message_queue_id, 0, &err);

        timestamp = trigger_signal->timestamp;

        trigger_signal_put(trigger_signal);

        /* TODO: handle trigger signals properly. */
        trigger_36_1_handle_pulse(trigger_context, timestamp);

    }
}

static void init_crank_signal(void)
{
    trigger_signal_message_queue_id = CoCreateQueue((void * *)&trigger_signal_queue, NUM_TRIGGER_SIGNALS, EVENT_SORT_TYPE_FIFO);
    trigger_context = trigger_36_1_init();

    init_crank_trigger_signal_list();

    CoCreateTask(trigger_signal_task, 
                 &trigger_signal_message_queue_id, 
                 1, 
                 &trigger_signal_task_stack[TRIGGER_SIGNAL_TASK_STACK_SIZE - 1], 
                 TRIGGER_SIGNAL_TASK_STACK_SIZE);


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



int main(void)
{
    SystemInit();

    init_leds();
    initHiResTimer(1000000, hi_res_tick);

    init_button();

    timed_events_init(1000000);    

    CoInitOS(); /*!< Initialise CoOS */

    init_crank_signal();

    initSerialTask();
    init_pulses();
    injector_1 = injector_output_get();
    ignition_1 = ignition_output_get();

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

