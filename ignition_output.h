#ifndef __IGNITION_OUTPUT_H__
#define __IGNITION_OUTPUT_H__

#include <stdint.h>

#define IGNITION_MAX 8

typedef struct ignition_output_st ignition_output_st;

ignition_output_st * ignition_output_get(void);
void ignition_pulse_schedule(ignition_output_st * const ignition_output,
                             uint32_t const base_count,
                             uint32_t const initial_delay_us,
                             uint16_t const pulse_us);

uint32_t ignition_timer_count_get(ignition_output_st const * const injector_output);

#endif /* __IGNITION_OUTPUT_H__ */
