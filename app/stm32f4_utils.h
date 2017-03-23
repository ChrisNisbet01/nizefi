#ifndef __STM32F4_UTILS_H__
#define __STM32F4_UTILS_H__

#include <stm32f4xx_tim.h>

#include <stdint.h>
#include <stdbool.h>

void stm32f4_enable_IRQ(uint_fast8_t const irq,
                        uint_fast8_t const priority,
                        uint_fast8_t const sub_priority);

void stm32f4_disable_IRQ(uint_fast8_t const irq);

void stm32f4_timer_configure(TIM_TypeDef * tim, uint_fast32_t period, uint_fast32_t frequency_hz, bool const use_PCLK2);


#endif /* __STM32F4_UTILS_H__ */
