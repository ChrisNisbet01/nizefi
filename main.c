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
#include "utils.h"

#include <math.h>
#include <inttypes.h>

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
 
uint32_t get_injector_pulse_width_us(void)
{
    /* TODO - Calculate pulse width. */
    return 800;
}

#define TEST_INJECTOR_DEAD_TIME_US 500UL
uint32_t get_injector_dead_time_us(void)
{
    /* TODO - Calculate from battery voltage and configuration table. */
    return TEST_INJECTOR_DEAD_TIME_US;
}

/* Store some debug values */
static uint32_t debug_injector_us_until_open;
static uint32_t debug_injector_pulse_width_us;
float debug_injector_close_angle_atdc;
float debug_injector_scheduling_angle;
float debug_time_to_next_injector_close;
uint32_t debug_latency;
uint32_t debug_timer_base_count;
int debug_injector_number;

void print_injector_debug(void)
{
    float get_angle_when_injector_closed(void);

    printf("\r\ninj %d\r\n", debug_injector_number);
    printf("delay %"PRIu32" width %"PRIu32"\r\n", debug_injector_us_until_open, debug_injector_pulse_width_us);
    printf("close angle %f scheduling angle %f time to close %f\r\n",
           debug_injector_close_angle_atdc,
           debug_injector_scheduling_angle,
           debug_time_to_next_injector_close);
    printf("latency %"PRIu32" base %"PRIu32"\r\n\r\n", debug_latency, debug_timer_base_count);
    printf("\r\nactual close %f\r\n", get_angle_when_injector_closed());
}

float current_engine_cycle_angle_get(void)
{
    return trigger_36_1_engine_cycle_angle_get(trigger_context);
}

void injector_pulse_callback(float const crank_angle,
                             uint32_t timestamp,
                             void * const user_arg)
{
    injector_output_st * const injector = user_arg;
#if 1
    float const injector_close_angle = injector_close_angle_get(injector); /* Angle we want the injector closed. */
    float const injector_scheduling_angle = trigger_36_1_engine_cycle_angle_get(trigger_context); /* Current engine angle. */
    /* Determine how long it will take to rotate this many degrees. */
    float const time_to_next_injector_close = trigger_36_1_rotation_time_get(trigger_context,
                                                                             (float)get_engine_cycle_degrees() - normalise_engine_cycle_angle(injector_scheduling_angle - injector_close_angle));
    /* The injector pulse width must include the time taken to open the injector (dead time). */
    uint32_t const injector_pulse_width_us = get_injector_pulse_width_us() + get_injector_dead_time_us();
    uint32_t const injector_us_until_open = lrintf(time_to_next_injector_close * TIMER_FREQUENCY)
                                                    - injector_pulse_width_us;
    uint32_t const latency = hi_res_counter_val() - timestamp;
    uint32_t const injector_timer_count = injector_timer_count_get(injector);
    uint32_t const timer_base_count = (injector_timer_count - latency) & 0xffff; /* This is the time from which we base the injector event. */

    (void)crank_angle;
    (void)timestamp;

    if ((int)injector_us_until_open < 0)
    {
        /* XXX - TODO - Update a statistic? */
        goto done;
    }

    debug_latency = latency;
    debug_timer_base_count = timer_base_count;
    debug_injector_close_angle_atdc = injector_close_angle;
    debug_injector_scheduling_angle = injector_scheduling_angle;
    debug_time_to_next_injector_close = time_to_next_injector_close;
    debug_injector_us_until_open = injector_us_until_open;
    debug_injector_pulse_width_us = injector_pulse_width_us;
    debug_injector_number = injector_number_get(injector);
#else
    uint32_t const injector_pulse_width_us = get_injector_pulse_width_us();
    uint32_t const injector_us_until_open = 100; 

#endif
    /* Called roughly 360 degrees before the injector is due to close. */
    injector_pulse_schedule(injector, timer_base_count, injector_us_until_open, injector_pulse_width_us);

done:
    return;
}

float get_ignition_advance(void)
{
    /* TODO get from configuration */
    return 10.0; /* NB - value is in degrees BTDC. */
}

void ignition_pulse_callback(float const crank_angle,
                             uint32_t timestamp,
                             void * const user_arg)
{
    ignition_output_st * const ignition = user_arg;
    (void)crank_angle;
    (void)timestamp; 

    ignition_pulse_schedule(ignition, ignition_timer_count_get(ignition), 100, 2000);
}

static void get_injector_outputs(void)
{
    unsigned int const num_injectors = num_injectors_get();
    unsigned int degrees_per_engine_cycle = get_engine_cycle_degrees();
    float const degrees_per_cylinder_injection = (float)degrees_per_engine_cycle / num_injectors;
    float const injector_close_angle = get_config_injector_close_angle();
    size_t index;

    for (index = 0; index < num_injectors_get(); index++)
    {
        injectors[index] = injector_output_get(index,
                                               normalise_engine_cycle_angle(injector_close_angle + (degrees_per_cylinder_injection * index)));
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

static void setup_injector_scheduling(trigger_wheel_36_1_context_st * const trigger_context)
{
    unsigned int const num_injectors = num_injectors_get();
    size_t index;

    /* By using the angle at which the injector closes to schedule the next event there should be enough time to 
       get the start of the injector pulse scheduled in. 
       This is with the assumption that that the injector duty cycle never goes beyond something like 80-85%.
    */
    for (index = 0; index < num_injectors; index++)
    {
        injector_output_st * const injector = injectors[index];
        float const injector_close_to_schedule_delay = 0.0;

        float const injector_scheduling_angle = normalise_engine_cycle_angle(injector_close_angle_get(injector) 
                                                                             + injector_close_to_schedule_delay);
        /* Temp debug add in some slack to help prevent event overlap. 
           Means that injector duty is limited to 100  * (720-20) / 720%. 
         */

        trigger_36_1_register_callback(trigger_context,
                                       normalise_engine_cycle_angle(injector_scheduling_angle),
                                       injector_pulse_callback,
                                       injector);
    }
}

float get_ignition_spark_angle_btdc(void)
{
    return 10.0;

}
static void setup_ignition_scheduling(trigger_wheel_36_1_context_st * const trigger_context)
{
    unsigned int const num_sparks = num_ignition_outputs_get();
    unsigned int degrees_per_engine_cycle = get_engine_cycle_degrees();
    float const degrees_per_cylinder_ignition = (float)degrees_per_engine_cycle / num_sparks;
    float const spark_angle_btdc = get_ignition_spark_angle_btdc();
    float const ignition_scheduling_angle = normalise_engine_cycle_angle(720.0 - 360.0 - spark_angle_btdc);
    size_t index;

    /* By doing the scheduling 1 revolution before the spark there should be enough time to get the start of the igntion pulse scheduled in. 
    */
    //for (index = 0; index < num_sparks; index++)
    for (index = 0; index < 0; index++)
    {
        trigger_36_1_register_callback(trigger_context,
                                       normalise_engine_cycle_angle(ignition_scheduling_angle + (degrees_per_cylinder_ignition * index)),
                                       ignition_pulse_callback,
                                       ignitions[index]);
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
    setup_injector_scheduling(trigger_context);
    setup_ignition_scheduling(trigger_context);

    init_trigger_signals(trigger_context); /* Done after CoInitOS() as it uses CoOS resources. */


    fprintf(stderr, "CoOS RTOS: Starting scheduler\r\n");

    CoStartOS(); /* Start scheduler. */

}

