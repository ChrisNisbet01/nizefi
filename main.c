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

/*---------------------------- Symbol Define -------------------------------*/
#define STACK_SIZE_TASKA 128              /*!< Define "taskA" task size */
#define STACK_SIZE_TASKB 128              /*!< Define "taskB" task size */
#define STACK_SIZE_TASKC 128              /*!< Define "taskC" task size */
#define STACK_SIZE_TASKD 128              /*!< Define "taskD" task size */

/* Private typedef -----------------------------------------------------------*/
GPIO_InitTypeDef GPIO_InitStructure;
/*---------------------------- Variable Define -------------------------------*/
OS_STK taskA_stk[STACK_SIZE_TASKA]; /*!< Define "taskA" task stack */
OS_STK taskB_stk[STACK_SIZE_TASKB]; /*!< Define "taskB" task stack */
OS_STK taskC_stk[STACK_SIZE_TASKC]; /*!< Define "taskC" task stack */
OS_STK taskD_stk[STACK_SIZE_TASKD]; /*!< Define "taskD" task stack */

/**
 *******************************************************************************
 * @brief       "taskA" task code
 * @param[in]   None
 * @param[out]  None
 * @retval      None
 * @par Description
 * @details    This task use to crate mutex and flags,print message "taskA running".
 *             Indicate "taskA" had been executed.
 *******************************************************************************
 */
void taskA(void* pdata)
{
  unsigned int a = 0;
  printf("CoOS taskA: started\n");
  while (1)
    {
      if (a & 1)
        {
          GPIO_SetBits(GPIOD, GPIO_Pin_12);
        }
      else
        {
          GPIO_ResetBits(GPIOD, GPIO_Pin_12);
        }
      a++;
      CoTickDelay(25);  //25 x 10ms = 250ms
    }
}

/**
 *******************************************************************************
 * @brief       "taskB" task code
 * @param[in]   None
 * @param[out]  None
 * @retval      None
 * @par Description
 * @details    This task use to print message "taskB running". Indicate "taskB"
 *             had been executed.
 *******************************************************************************
 */
void taskB(void* pdata)
{
  unsigned int b = 0;
  printf("CoOS taskB: started\n");
  while (1)
    {
      if (b & 1)
        {
          GPIO_SetBits(GPIOD, GPIO_Pin_13);
        }
      else
        {
          GPIO_ResetBits(GPIOD, GPIO_Pin_13);
        }
      b++;
      CoTickDelay(25);  //50 x 10ms = 500ms
    }
}

/**
 *******************************************************************************
 * @brief       "taskC" task code
 * @param[in]   None
 * @param[out]  None
 * @retval      None
 * @par Description
 * @details    This task use to print message "taskB running". Indicate "taskB"
 *             had been executed.
 *******************************************************************************
 */
void taskC(void* pdata)
{
  unsigned int c = 0;
  printf("CoOS taskC: started\n");
  while (1)
    {
      if (c & 1)
        {
          GPIO_SetBits(GPIOD, GPIO_Pin_14);
        }
      else
        {
          GPIO_ResetBits(GPIOD, GPIO_Pin_14);
        }
      c++;
      CoTickDelay(25);  //100 x 10ms = 1000ms
    }
}


/**
 *******************************************************************************
 * @brief       "taskC" task code
 * @param[in]   None
 * @param[out]  None
 * @retval      None
 * @par Description
 * @details    This task use to print message "taskB running". Indicate "taskB"
 *             had been executed.
 *******************************************************************************
 */
void taskD(void* pdata)
{
  unsigned int d = 0;
  printf("CoOS taskD: started\n");
  while (1)
    {
      if (d & 1)
        {
          GPIO_ResetBits(GPIOD, GPIO_Pin_15);
        }
      else
        {
          GPIO_SetBits(GPIOD, GPIO_Pin_15);
        }
      d++;
      CoTickDelay(50);  //50 x 10ms = 500ms
    }
}


int main(void)
{
  SystemInit();
  USART_Configuration();
  printf("STM32F4-Discovery Board is booting!\n");
  printf("-----------------------------------\n");
  printf("CoOS RTOS V-1.1.4; Demo for STM32F4\n");
  printf("V-01 17.10.2012 (w) by psavr@gmx.ch\n");
  printf("-----------------------------------\n");

  /* GPIOD Periph clock enable */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
  /* Configure PD12, PD13, PD14 and PD15 in output pushpull mode */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14  | GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOD, &GPIO_InitStructure);

  CoInitOS(); /*!< Initial CooCox CoOS          */

  /*!< Create three tasks	*/
  printf("CoOS RTOS: Creating tasks\n");
  //CoCreateTask(taskA, 0, 0, &taskA_stk[STACK_SIZE_TASKA-1], STACK_SIZE_TASKA);
  //CoCreateTask(taskB, 0, 1, &taskB_stk[STACK_SIZE_TASKB-1], STACK_SIZE_TASKB);
  //CoCreateTask(taskC, 0, 2, &taskC_stk[STACK_SIZE_TASKC-1], STACK_SIZE_TASKC);
  CoCreateTask(taskD, 0, 3, &taskD_stk[STACK_SIZE_TASKD-1], STACK_SIZE_TASKD);
  GPIO_SetBits(GPIOD, GPIO_Pin_12); 
  GPIO_SetBits(GPIOD, GPIO_Pin_13);
  GPIO_SetBits(GPIOD, GPIO_Pin_14);
  GPIO_SetBits(GPIOD, GPIO_Pin_15);

  printf("CoOS RTOS: Starting scheduler\n");
  CoStartOS(); /*!< Start multitask	           */

  printf("CoOS RTOS: Scheduler stopped\n");
  while (1)
  {
  }
}

