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

#include "serial_task.h"

/*---------------------------- Symbol Define -------------------------------*/
#define STACK_SIZE_TASKA 128              /*!< Define "taskA" task size */
#define STACK_SIZE_TASKB 128              /*!< Define "taskB" task size */
#define STACK_SIZE_TASKC 128              /*!< Define "taskC" task size */
#define STACK_SIZE_TASKD 128              /*!< Define "taskD" task size */

/* Private typedef -----------------------------------------------------------*/
GPIO_InitTypeDef GPIO_InitStructure;
/*---------------------------- Variable Define -------------------------------*/
static __attribute((aligned(8))) OS_STK taskA_stk[STACK_SIZE_TASKA]; /*!< Define "taskA" task stack */
static __attribute((aligned(8))) OS_STK taskB_stk[STACK_SIZE_TASKB]; /*!< Define "taskB" task stack */
static __attribute((aligned(8))) OS_STK taskC_stk[STACK_SIZE_TASKC]; /*!< Define "taskC" task stack */
static __attribute((aligned(8))) OS_STK taskD_stk[STACK_SIZE_TASKD]; /*!< Define "taskD" task stack */

static void common_thread_task(char const * const task_name, 
                               char const * const msg,
                               unsigned int gpio_pin, 
                               unsigned int const delay_ticks)
{
    unsigned int i = 0;
    printf("CoOS task %s: started\n", task_name);
    while (1)
    {
        if (msg != NULL)
        {
            printf(msg);
        }
        else
        {
            if (i & 1)
            {
                GPIO_SetBits(GPIOD, gpio_pin);
            }
            else
            {
                GPIO_ResetBits(GPIOD, gpio_pin);
            }
        }
        CoTickDelay(delay_ticks);  //25 x 10ms = 250ms
        i++;
    }
}

void taskA(void * pdata)
{
    (void)pdata;
    common_thread_task("A", NULL, GPIO_Pin_12, CFG_SYSTICK_FREQ / 4);
}

void taskB(void* pdata)
{
    (void)pdata;
    common_thread_task("B", NULL, GPIO_Pin_13, CFG_SYSTICK_FREQ / 4);
}

void taskC(void* pdata)
{
    (void)pdata;
    common_thread_task("C", NULL, GPIO_Pin_14, CFG_SYSTICK_FREQ / 4);
}

void taskD(void* pdata)
{
    (void)pdata;
    common_thread_task("C", "1", GPIO_Pin_15, CFG_SYSTICK_FREQ);
}


int main(void)
{
    SystemInit();

#if 0
    USART_Configuration();
    printf("STM32F4-Discovery Board is running\n");
    printf("CoOS RTOS V-1.1.6; Demo for STM32F4\n");
    printf("-----------------------------------\n");
#endif
    /* GPIOD Periph clock enable */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
    /* Configure PD12, PD13, PD14 and PD15 in output push-pull mode */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14  | GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    CoInitOS(); /*!< Initialise CoOS */

    /*!< Create three tasks	*/
    printf("CoOS RTOS: Creating tasks\n");
    CoCreateTask(taskA, 0, 0, &taskA_stk[STACK_SIZE_TASKA - 1], STACK_SIZE_TASKA);
    CoCreateTask(taskB, 0, 1, &taskB_stk[STACK_SIZE_TASKB - 1], STACK_SIZE_TASKB);
    CoCreateTask(taskC, 0, 2, &taskC_stk[STACK_SIZE_TASKC - 1], STACK_SIZE_TASKC);
    CoCreateTask(taskD, 0, 3, &taskD_stk[STACK_SIZE_TASKD - 1], STACK_SIZE_TASKD);

    initSerialTask();

    printf("CoOS RTOS: Starting scheduler\n");
    CoStartOS(); /*!< Start multitask	           */

    printf("CoOS RTOS: Scheduler stopped\n");
    while (1)
    {
    }
}

