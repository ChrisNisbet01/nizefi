#ifndef __INJECTOR_OUTPUT_H__
#define __INJECTOR_OUTPUT_H__

#include <stdint.h>

#define INJECTOR_MAX 8

typedef struct injector_output_st injector_output_st;

injector_output_st * injector_output_get(void);
void injector_pulse_schedule(injector_output_st * const injector_output,
                             uint32_t const base_count,
                             uint32_t initial_delay_us,
                             uint16_t pulse_us);

uint32_t injector_timer_count_get(injector_output_st const * const injector_output);

#endif /* __INJECTOR_OUTPUT_H__ */
