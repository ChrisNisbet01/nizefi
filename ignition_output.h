#ifndef __IGNITION_OUTPUT_H__
#define __IGNITION_OUTPUT_H__

#include <stdint.h>

typedef struct ignition_output_st ignition_output_st;

ignition_output_st * ignition_output_get(void);
void ignition_pulse_schedule(ignition_output_st * const ignition_output,
                             uint32_t initial_delay_us,
                             uint16_t pulse_us);

#endif /* __IGNITION_OUTPUT_H__ */
