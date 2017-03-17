#include "stm32f4_utils.h"
#include "utils.h"

#include <stdlib.h>
#include <stdint.h>

#if defined(STM32F30X)
#include <stm32f30x_usart.h>
#include <stm32f30x_gpio.h>
#include <stm32f30x_rcc.h>
#include <stm32f30x_misc.h>
#elif defined(STM32F4XX)
#include <stm32f4xx_usart.h>
#include <stm32f4xx_gpio.h>
#include <stm32f4xx_rcc.h>
#include <misc.h>
#elif defined(STM32F10X)
#include <stm32f10x_usart.h>
#include <stm32f10x_gpio.h>
#include <stm32f10x_rcc.h>
#include <misc.h>
#endif

#include "usart.h"

typedef enum usart_idx_t {
	USART1_IDX,
	MAX_USARTS
} usart_idx_t;


typedef struct usart_port_config_st
{
	USART_TypeDef		*usart;

	uint_fast16_t		txPin;
	uint_fast8_t		txPinSource;

	uint_fast16_t		rxPin;
	uint_fast8_t		rxPinSource;

    uint_fast8_t		txPinAF;
    uint_fast8_t		rxPinAF;

    uint_fast32_t		RCC_AHBPeriph;

	GPIO_TypeDef		*gpioPort;	/* Assumes same port for both pins */

	void 				(*RCC_APBPeriphClockCmd)(uint32_t RCC_APBPeriph, FunctionalState NewState);
	uint_fast32_t		RCC_APBPeriph;
	uint_fast16_t		irq;

} usart_port_config_st;

static const usart_port_config_st usart_configs[] =
{
    [USART1_IDX] =
    {
        .usart = USART1,
        .txPin = GPIO_Pin_6,
        .txPinSource = GPIO_PinSource6,
        .rxPin = GPIO_Pin_7,
        .rxPinSource = GPIO_PinSource7,
        .txPinAF = GPIO_AF_USART1,
        .rxPinAF = GPIO_AF_USART1,
        .gpioPort = GPIOB,
        .RCC_AHBPeriph = RCC_AHB1Periph_GPIOB,

        .RCC_APBPeriphClockCmd = RCC_APB2PeriphClockCmd,
        .RCC_APBPeriph = RCC_APB2Periph_USART1,

        .irq = USART1_IRQn
    }
};
#define NB_UART_PORTS ARRAY_SIZE(usart_configs)
#define GET_USART_IDX(ptr)	((ptr)-usart_configs)

static usart_cb_st usartCallbackInfo[NB_UART_PORTS];

static usart_port_config_st const * usartConfigLookup( USART_TypeDef *usart )
{
	int i;

	for ( i=0; i < MAX_USARTS; i++ )
	{
		if ( usart_configs[i].usart == usart )
		{
			return &usart_configs[i];
		}
	}

	return NULL;
}

void const * stm32_usart_init(usart_init_st *cfg)
{
	usart_port_config_st const * uart_config;

	if ( (uart_config=usartConfigLookup(cfg->usart)) == NULL )
		goto done;

	usartCallbackInfo[GET_USART_IDX(uart_config)] = cfg->callback;

	/* enable appropriate clocks */
    RCC_AHB1PeriphClockCmd(uart_config->RCC_AHBPeriph, ENABLE);

    uart_config->RCC_APBPeriphClockCmd(uart_config->RCC_APBPeriph, ENABLE);

	/* configure appropriate GPIO pins */
    GPIO_InitTypeDef  GPIO_InitStructure;

    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;

    if (cfg->mode & usart_mode_tx)
    {
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
        GPIO_InitStructure.GPIO_Pin = uart_config->txPin;

        /* connect pin to USART */
        GPIO_PinAFConfig(uart_config->gpioPort, uart_config->txPinSource, uart_config->txPinAF);

        GPIO_Init(uart_config->gpioPort, &GPIO_InitStructure);
    }

    if (cfg->mode & usart_mode_rx)
    {
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
        GPIO_InitStructure.GPIO_Pin = uart_config->rxPin;

        /* connect pin to USART */
        GPIO_PinAFConfig(uart_config->gpioPort, uart_config->rxPinSource, uart_config->rxPinAF);

        GPIO_Init(uart_config->gpioPort, &GPIO_InitStructure);
    }

    /* Make this IRQ lower priority than the ones used for engine 
     * control related tasks. 
     */
    stm32f4_enable_IRQ(uart_config->irq, 4, 2);

done:
    return uart_config;
}

static void usartIrqHandler(usart_port_config_st const * const uart_config, usart_cb_st * runtime)
{
    if (USART_GetITStatus(uart_config->usart, USART_IT_RXNE) != RESET)
    {
        uint8_t const rx_char = uart_config->usart->DR;

        /* Reading the DR register also clears USART_IT_RXNE in the 
         * interrupt status register. 
         */
        /* Receive data ready. */
    	if (runtime->putRxChar != NULL)
    	{
            runtime->putRxChar(runtime->pv, rx_char);
    	}
    }

    if (USART_GetITStatus(uart_config->usart, USART_IT_TXE) != RESET)
    {
        int disable_transmit = 1;

        /* Transmit register empty. Either send another char if one is 
         * ready or shut TX down. 
         */
        if (runtime->getTxChar != NULL)
        {
            int const ch = runtime->getTxChar(runtime->pv);

            if (ch >= 0)
            {
                USART_SendData(uart_config->usart, ch);
                /* THis will also clear USART_IT_TXE in the 
                 * interrupt status register.
                 */
                disable_transmit = 0;
            }
        }
        if (disable_transmit)
        {
            USART_ITConfig(uart_config->usart, USART_IT_TXE, DISABLE);
        }
    }

    if (USART_GetITStatus(uart_config->usart, USART_IT_ORE) != RESET)
    {
        USART_ClearITPendingBit (uart_config->usart, USART_IT_ORE);
        // TODO: statistic?
    }
}

void USART1_IRQHandler(void)
{
    usartIrqHandler(&usart_configs[USART1_IDX], &usartCallbackInfo[USART1_IDX]);
}


