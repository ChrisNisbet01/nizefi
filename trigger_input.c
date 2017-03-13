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

typedef struct trigger_gpio_config_st
{
    uint32_t RCC_AHBPeriph;
    GPIO_TypeDef * port;
    uint_fast16_t pin;
    uint_fast8_t EXTI_PortSource;
    uint_fast8_t EXTI_PinSource; 
    uint32_t EXTI_Line;
    uint_fast8_t NVIC_IRQChannel;
} trigger_gpio_config_st;

typedef struct trigger_signal_st
{
    STAILQ_ENTRY(trigger_signal_st) entry;

    trigger_signal_source_t source;
    uint32_t timestamp;
} trigger_signal_st; 

#define TRIGGER_SIGNAL_TASK_STACK_SIZE 1024
static __attribute((aligned(8))) OS_STK trigger_signal_task_stack[TRIGGER_SIGNAL_TASK_STACK_SIZE];

/* Separate crank and cam signal messages are defined because 
 * their signals are handled by different ISRs and I don't 
 * think there is any way to protect against one ISR getting 
 * interrupted by another. 
 */
#define CRANK_TRIGGER_SIGNAL_QUEUE_LEN 5
static trigger_signal_st crank_trigger_signals[CRANK_TRIGGER_SIGNAL_QUEUE_LEN];

#define CAM_TRIGGER_SIGNAL_QUEUE_LEN 5
static trigger_signal_st cam_trigger_signals[CAM_TRIGGER_SIGNAL_QUEUE_LEN]; 

#define TOTAL_TRIGGER_SIGNAL_LEN (CRANK_TRIGGER_SIGNAL_QUEUE_LEN + CAM_TRIGGER_SIGNAL_QUEUE_LEN)

static trigger_signal_st * trigger_signal_queue[TOTAL_TRIGGER_SIGNAL_LEN];

OS_EventID trigger_signal_message_queue_id;
/* TODO: Need a trigger input context with fields for the 
 * trigger context and the crank and cam callbacks which are 
 * yet to be supported. 
 */
static trigger_wheel_36_1_context_st * trigger_context;

typedef STAILQ_HEAD(trigger_signal_list, trigger_signal_st) trigger_signal_list;

static trigger_signal_list crank_trigger_signal_free_list;
static trigger_signal_list cam_trigger_signal_free_list;

static const trigger_gpio_config_st crank_trigger_gpio_config =
{
    .RCC_AHBPeriph = RCC_AHB1Periph_GPIOA,
    .port = GPIOA,
    .pin = GPIO_Pin_5,
    .EXTI_PortSource = EXTI_PortSourceGPIOA,
    .EXTI_PinSource = EXTI_PinSource5,
    .EXTI_Line = EXTI_Line5,
    .NVIC_IRQChannel = EXTI9_5_IRQn
};

static const trigger_gpio_config_st cam_trigger_gpio_config =
{
    .RCC_AHBPeriph = RCC_AHB1Periph_GPIOC,
    .port = GPIOC,
    .pin = GPIO_Pin_6,
    .EXTI_PortSource = EXTI_PortSourceGPIOC,
    .EXTI_PinSource = EXTI_PinSource6,
    .EXTI_Line = EXTI_Line6,
    .NVIC_IRQChannel = EXTI9_5_IRQn
}; 

static trigger_signal_st * trigger_signal_get(trigger_signal_list * const list_head)
{
    /* This function is called from within an IRQ. Entries are placed back into the queue with interrupts disabled.
    */
    trigger_signal_st * const trigger_signal = STAILQ_FIRST(list_head);

    if (trigger_signal != NULL)
    {
        STAILQ_REMOVE_HEAD(list_head, entry);
    }

    return trigger_signal;
}

static void trigger_signal_put(trigger_signal_list * const list_head,
                               trigger_signal_st * const trigger_signal)
{
    IRQ_DISABLE_SAVE();

    STAILQ_INSERT_TAIL(list_head, trigger_signal, entry);

    IRQ_ENABLE_RESTORE();
}

static trigger_signal_st * crank_trigger_signal_get(void)
{
    return trigger_signal_get(&crank_trigger_signal_free_list);
}

static trigger_signal_st * cam_trigger_signal_get(void)
{
    return trigger_signal_get(&cam_trigger_signal_free_list);
}

static void crank_trigger_signal_put(trigger_signal_st * const trigger_signal)
{
    trigger_signal_put(&crank_trigger_signal_free_list, trigger_signal);
}

static void cam_trigger_signal_put(trigger_signal_st * const trigger_signal)
{
    trigger_signal_put(&cam_trigger_signal_free_list, trigger_signal);
}

static void handle_trigger_signal(trigger_signal_st * const trigger_signal,
                                  trigger_signal_source_t const source, 
                                  uint32_t const timestamp)
{
    trigger_signal->timestamp = timestamp;
    trigger_signal->source = source;

    CoEnterISR();

    isr_PostQueueMail(trigger_signal_message_queue_id, trigger_signal);

    CoExitISR();
}

static void handle_crank_trigger_signal(uint32_t const timestamp)
{
    trigger_signal_st * const trigger_signal = crank_trigger_signal_get();

    if (trigger_signal != NULL)
    {
        handle_trigger_signal(trigger_signal,
                              trigger_signal_source_crank, 
                              timestamp);
    }
}

static void handle_cam_trigger_signal(uint32_t const timestamp)
{
    trigger_signal_st * const trigger_signal = cam_trigger_signal_get();

    if (trigger_signal != NULL)
    {
        handle_trigger_signal(trigger_signal,
                              trigger_signal_source_cam, 
                              timestamp);
    }
}

/* Handle PA0 interrupt. */
void EXTI0_IRQHandler(void)
{
    uint32_t timestamp = hi_res_counter_val();

    /* XXX - Need to determine the EXTI_Line to check for some 
     * other way. Hard-coded assumption that this is for the crank 
     * is no good. 
     */

    /* Make sure that interrupt flag is set */
    if (EXTI_GetITStatus(EXTI_Line0) != RESET)
    {
        /* Clear interrupt flag */
        EXTI_ClearITPendingBit(EXTI_Line0);

        handle_crank_trigger_signal(timestamp);
    }
}

void EXTI9_5_IRQHandler(void)
{
    uint32_t timestamp = hi_res_counter_val();

    /* XXX - Need to determine the EXTI_Line to check for some 
     * other way. Hard-coded assumption that this is for the cam or 
     * crank is no good. 
     */

    /* Make sure that interrupt flag is set */
    if (EXTI_GetITStatus(EXTI_Line6) != RESET)
    {
        /* Clear interrupt flag */
        EXTI_ClearITPendingBit(EXTI_Line6);

        handle_cam_trigger_signal(timestamp);
    }
    if (EXTI_GetITStatus(EXTI_Line5) != RESET)
    {
        /* Clear interrupt flag */
        EXTI_ClearITPendingBit(EXTI_Line5);

        handle_crank_trigger_signal(timestamp);
    }
}


static void init_trigger_signal_list(trigger_signal_list * const list_head, 
                                     trigger_signal_st * const trigger_signals,
                                     size_t num_trigger_signals)
{
    size_t index;

    STAILQ_INIT(list_head);
    for (index = 0; index < num_trigger_signals; index++)
    {
        trigger_signal_st * const trigger_signal = &trigger_signals[index];

        STAILQ_INSERT_TAIL(list_head, trigger_signal, entry);
    }
}

static void init_crank_trigger_signal_list(void)
{
    init_trigger_signal_list(&crank_trigger_signal_free_list, crank_trigger_signals, CRANK_TRIGGER_SIGNAL_QUEUE_LEN);
}

static void init_cam_trigger_signal_list(void)
{
    init_trigger_signal_list(&cam_trigger_signal_free_list, cam_trigger_signals, CAM_TRIGGER_SIGNAL_QUEUE_LEN);
}

static void init_trigger_signal_lists(void)
{
    init_crank_trigger_signal_list();
    init_cam_trigger_signal_list();
}

int min_queue_length_get(void)
{
    int const length = 100;

    /* TODO - Reinstate some statistics. It appears that the queue 
     * length goes to 0 at times (due to printf?). 
     */
    return length;
}

float rpm_get(void)
{
    return trigger_36_1_rpm_get(trigger_context);
}

float crank_angle_get(void)
{
    return trigger_36_1_crank_angle_get(trigger_context);
}

float engine_cycle_angle_get(void)
{
    return trigger_36_1_engine_cycle_angle_get(trigger_context);
}

void trigger_input_task(void * pdata)
{
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

        switch (trigger_source)
        {
            case trigger_signal_source_crank:
                crank_trigger_signal_put(trigger_signal);
                trigger_36_1_handle_crank_pulse(trigger_context, timestamp);
                break;
            case trigger_signal_source_cam:
                cam_trigger_signal_put(trigger_signal);
                trigger_36_1_handle_cam_pulse(trigger_context, timestamp);
                break;
        }
    }
}

static void configure_gpio_pin(trigger_gpio_config_st const * const gpio_config)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    /* Enable the clock for the GPIO port. */
    RCC_AHB1PeriphClockCmd(gpio_config->RCC_AHBPeriph, ENABLE);

    /* Set pin as input */
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_Pin = gpio_config->pin;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL; /* External wiring should always pull one way or the other. */
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;

    GPIO_Init(gpio_config->port, &GPIO_InitStruct);
}

/* TODO: This can be used for all interrupt enables. Put this 
 * into a separate module. 
 */
static void configure_nested_vector_interrupt_controller(uint_fast8_t const NVIC_IRQChannel,
                                                         uint_fast8_t const priority,
                                                         uint_fast8_t const sub_priority)
{
    NVIC_InitTypeDef NVIC_InitStruct; 

    /* Add IRQ vector to NVIC */
    NVIC_InitStruct.NVIC_IRQChannel = NVIC_IRQChannel;
    /* Set priority */
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = priority; /* Highest priority. 
                                                                 XXX - Should this be higher or lower priority than the IRQ handling 
                                                                 ignition and injector outputs?
                                                               */
    /* Set sub priority */
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = sub_priority;

    /* Enable interrupt */
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;

    /* Add to NVIC */
    NVIC_Init(&NVIC_InitStruct);
}

static void connect_pin_to_external_interrupt(uint32_t const EXTI_Line,
                                              EXTITrigger_TypeDef const EXTI_Trigger)
{
    EXTI_InitTypeDef EXTI_InitStruct;

    /* Connect the pin to the external interrupt line. */
    EXTI_InitStruct.EXTI_Line = EXTI_Line;
    /* Enable interrupt */
    EXTI_InitStruct.EXTI_LineCmd = ENABLE;
    /* Interrupt mode */
    EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger;

    /* Add to EXTI */
    EXTI_Init(&EXTI_InitStruct);
}

static void configure_gpio_external_irq(trigger_gpio_config_st const * const gpio_config)
{

    /* Enable clock for SYSCFG. Required to get access to 
     * SYSCFG_EXTICRx. 
     */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

    /* Connect the port:pin to an external interrupt. */
    SYSCFG_EXTILineConfig(gpio_config->EXTI_PortSource, gpio_config->EXTI_PinSource);

    /* Make this interrupt the highest priority. */
    /* XXX - Need to figure out the best interrupt priorities. */
    configure_nested_vector_interrupt_controller(gpio_config->NVIC_IRQChannel, 0, 0);

    connect_pin_to_external_interrupt(gpio_config->EXTI_Line,
                                      EXTI_Trigger_Falling); /* XXX - Configurable? */
}

static void initialise_crank_trigger_input(void)
{
    register_crank_trigger_callback(handle_crank_trigger_signal);
}

static void initialise_cam_trigger_input(void)
{
    trigger_gpio_config_st const * const gpio_config = &cam_trigger_gpio_config;

    configure_gpio_pin(gpio_config);

    configure_gpio_external_irq(gpio_config);

}

void init_trigger_signals(trigger_wheel_36_1_context_st * const trigger_wheel_context)
{
    /* Setup crankshaft and camshaft trigger inputs. Currently 
     * only doing crankshaft signals. 
     */

    /* The trigger messages and message queue must be set up 
     * before the GPIO starts generating interrupts so that the 
     * ISR has valid message queues to read from once IRQs start 
     * happening. 
     */
    init_trigger_signal_lists(); 

    trigger_signal_message_queue_id = CoCreateQueue((void * *)&trigger_signal_queue, TOTAL_TRIGGER_SIGNAL_LEN, EVENT_SORT_TYPE_FIFO);

    /* XXX - FIXME. Get trigger wheel decoder to register for 
     * trigger events from this module. Also support enabling and 
     * disabling of the events. 
     */
    trigger_context = trigger_wheel_context;

    CoCreateTask(trigger_input_task,
                 &trigger_signal_message_queue_id,
                 1,
                 &trigger_signal_task_stack[TRIGGER_SIGNAL_TASK_STACK_SIZE - 1],
                 TRIGGER_SIGNAL_TASK_STACK_SIZE);

    initialise_crank_trigger_input();
    initialise_cam_trigger_input();
}

