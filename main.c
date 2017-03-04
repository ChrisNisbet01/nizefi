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
#include "stm32f4xx_rcc.h"
#include "usart.h"
#include <math.h>

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
        GPIO_ToggleBits(GPIOD, gpio_pin); 
        CoTickDelay(delay_ticks);
    }
}

void taskC(void* pdata)
{
    (void)pdata;
    common_thread_task("C", GPIO_Pin_12, CFG_SYSTICK_FREQ / 4);
}

void taskD(void * pdata)
{
    (void)pdata;
    common_thread_task("D", GPIO_Pin_13, CFG_SYSTICK_FREQ);
}

static void init_leds(void)
{
    /* GPIOD Periph clock enable */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
    /* Configure PD12, PD13, PD14 and PD15 in output push-pull mode */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14  | GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
}

int main(void)
{
    SystemInit();

    init_leds();

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

