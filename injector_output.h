#ifndef __INJECTOR_OUTPUT_H__
#define __INJECTOR_OUTPUT_H__

#include <stdint.h>
#include <stddef.h>

#define INJECTOR_MAX 8

typedef struct injector_output_st injector_output_st;

injector_output_st * injector_output_get(size_t const injector_number,
                                         float const injector_close_angle);

void injector_pulse_schedule(injector_output_st * const injector_output,
                             uint32_t const base_count,
                             uint32_t initial_delay_us,
                             uint16_t pulse_us);

uint32_t injector_timer_count_get(injector_output_st const * const injector_output);
float injector_close_angle_get(injector_output_st const * const injector_output);
size_t injector_number_get(injector_output_st const * const injector_output);


#endif /* __INJECTOR_OUTPUT_H__ */
