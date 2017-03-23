#include "gpio_output.h"

void gpio_output_initialise(gpio_config_st const * const gpio_config)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    /* GPIOD Periph clock enable */
    RCC_AHB1PeriphClockCmd(gpio_config->RCC_AHBPeriph, ENABLE);

    /* Configure in output push-pull mode */
    GPIO_InitStructure.GPIO_Pin = gpio_config->pin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(gpio_config->port, &GPIO_InitStructure);
}

void gpio_output_set_active(gpio_config_st const * const gpio_config)
{
    GPIO_SetBits(gpio_config->port, gpio_config->pin);
}

void gpio_output_set_inactive(gpio_config_st const * const gpio_config)
{
    GPIO_ResetBits(gpio_config->port, gpio_config->pin);
}

