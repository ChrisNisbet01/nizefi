#include "trigger_input.h"
#include "trigger_wheel_36_1.h"
#include "queue.h"
#include "hi_res_timer.h"
#include <CoOS.h>
#include <OsArch.h>

#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_exti.h"
#include "stm32f4xx_syscfg.h"

#include <stddef.h>

/* 
    Note:
    Franksenso board has crank on PA5 and cam on PC6. This needs fixing up.
*/

typedef struct trigger_signal_st
{
    STAILQ_ENTRY(trigger_signal_st) entry;

    trigger_signal_source_t source;
    uint32_t timestamp;
} trigger_signal_st; 

#define TRIGGER_SIGNAL_TASK_STACK_SIZE 1024
static __attribute((aligned(8))) OS_STK trigger_signal_task_stack[TRIGGER_SIGNAL_TASK_STACK_SIZE];

#define TRIGGER_SIGNAL_QUEUE_LEN 5
static trigger_signal_st trigger_signals[TRIGGER_SIGNAL_QUEUE_LEN];
static trigger_signal_st * trigger_signal_queue[TRIGGER_SIGNAL_QUEUE_LEN];
OS_EventID trigger_signal_message_queue_id;
trigger_wheel_36_1_context_st * trigger_context;

static STAILQ_HEAD(, trigger_signal_st)trigger_signal_free_list; 


static int min_queue_length;
static int current_queue_length;
static trigger_signal_st * trigger_signal_get(void)
{
    /* This function is called from within an IRQ. Entries are placed back into the queue with interrupts disabled.
    */
    trigger_signal_st * const trigger_signal = STAILQ_FIRST(&trigger_signal_free_list);

    if (trigger_signal != NULL)
    {
        STAILQ_REMOVE_HEAD(&trigger_signal_free_list, entry);
        current_queue_length--;
    }

    return trigger_signal;
}

static void trigger_signal_put(trigger_signal_st * const trigger_signal)
{
    IRQ_DISABLE_SAVE();
    if (current_queue_length < min_queue_length)
    {
        min_queue_length = current_queue_length;
    }
    current_queue_length++;
    STAILQ_INSERT_TAIL(&trigger_signal_free_list, trigger_signal, entry);

    IRQ_ENABLE_RESTORE();
}

static void handle_trigger_signal(trigger_signal_source_t const source, uint32_t const timestamp)
{
    trigger_signal_st * const trigger_signal = trigger_signal_get();

    if (trigger_signal != NULL)
    {
        trigger_signal->timestamp = timestamp;
        trigger_signal->source = source;

        CoEnterISR();

        isr_PostQueueMail(trigger_signal_message_queue_id, trigger_signal);

        CoExitISR();
    }
}

static void handle_crank_trigger_signal(void)
{
    /* TODO: configurable edge? 
     * XXX - Triggering edge should already be set up so should be 
     * no need to check IO state. 
     */
    handle_trigger_signal(trigger_signal_source_crank, hi_res_counter_val());
}

static void handle_cam_trigger_signal(void)
{
    /* XXX - TODO. */
    handle_trigger_signal(trigger_signal_source_cam, hi_res_counter_val());
}

/* Handle PA0 interrupt */
void EXTI0_IRQHandler(void)
{
    /* Make sure that interrupt flag is set */
    if (EXTI_GetITStatus(EXTI_Line0) != RESET)
    {
        /* Clear interrupt flag */
        EXTI_ClearITPendingBit(EXTI_Line0);

        handle_crank_trigger_signal();
    }
    if (EXTI_GetITStatus(EXTI_Line6) != RESET)
    {
        /* Clear interrupt flag */
        EXTI_ClearITPendingBit(EXTI_Line6);

        handle_cam_trigger_signal();
    }
}


static void init_trigger_signal_list(void)
{
    size_t index;

    STAILQ_INIT(&trigger_signal_free_list);
    for (index = 0; index < TRIGGER_SIGNAL_QUEUE_LEN; index++)
    {
        trigger_signal_st * const trigger_signal = &trigger_signals[index];

        STAILQ_INSERT_TAIL(&trigger_signal_free_list, trigger_signal, entry);
        current_queue_length++;
    }
}

int min_queue_length_get(void)
{
    int const length = min_queue_length;

    min_queue_length = 100;

    return length;
}

float rpm_get(void)
{
    return trigger_36_1_rpm_get(trigger_context);
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
        trigger_signal_source_t trigger_source;

        trigger_signal =  CoPendQueueMail(message_queue_id, 0, &err);

        timestamp = trigger_signal->timestamp;
        trigger_source = trigger_signal->source;

        trigger_signal_put(trigger_signal);

        /* TODO: handle trigger signals properly. */
        trigger_36_1_handle_pulse(trigger_context, trigger_source, timestamp);

    }
}

void init_trigger_signals(void)
{
    /* Set variables used */
    GPIO_InitTypeDef GPIO_InitStruct;
    EXTI_InitTypeDef EXTI_InitStruct;
    NVIC_InitTypeDef NVIC_InitStruct;

    /* Setup crankshaft and camshaft trigger inputs. Currently 
     * only doing crankshaft signals. 
     */

    init_trigger_signal_list(); 


    trigger_signal_message_queue_id = CoCreateQueue((void * *)&trigger_signal_queue, TRIGGER_SIGNAL_QUEUE_LEN, EVENT_SORT_TYPE_FIFO);

    /* XXX - FIXME. Get trigger wheel decoder to register for 
     * trigger events from this module. Also support enabling and 
     * disabling of the events. 
     */
    trigger_context = trigger_36_1_init();

    CoCreateTask(trigger_signal_task,
                 &trigger_signal_message_queue_id,
                 1,
                 &trigger_signal_task_stack[TRIGGER_SIGNAL_TASK_STACK_SIZE - 1],
                 TRIGGER_SIGNAL_TASK_STACK_SIZE);


    /* Enable clock for GPIOA */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

    /* Set pin as input */
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* Enable clock for SYSCFG. Required to get access to 
     * SYSCFG_EXTICRx. 
     */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

    /* Tell system that you will use PA0 for EXTI_Line0. */
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource0);

    /* PA0 is connected to EXTI_Line0 */
    EXTI_InitStruct.EXTI_Line = EXTI_Line0;
    /* Enable interrupt */
    EXTI_InitStruct.EXTI_LineCmd = ENABLE;
    /* Interrupt mode */
    EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Falling; /* XXX - Configurable? */
    /* Add to EXTI */
    EXTI_Init(&EXTI_InitStruct);

    /* Add IRQ vector to NVIC */
    /* PA0 is connected to EXTI_Line0, which has EXTI0_IRQn 
       vector*/
    NVIC_InitStruct.NVIC_IRQChannel = EXTI0_IRQn;
    /* Set priority */
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x00; /* Highest priority. 
                                                                 XXX - Should this be higher than IRQ handling ignition and injector outputs?
                                                               */
    /* Set sub priority */
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0x00;
    /* Enable interrupt */
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    /* Add to NVIC */
    NVIC_Init(&NVIC_InitStruct);
}

