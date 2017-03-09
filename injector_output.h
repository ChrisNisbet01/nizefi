#ifndef __INJECTOR_OUTPUT_H__
#define __INJECTOR_OUTPUT_H__

#include <stdint.h>

typedef struct injector_output_st injector_output_st;

injector_output_st * injector_output_get(void);
void injector_pulse_schedule(injector_output_st * const injector_output,
                             uint32_t initial_delay_us,
                             uint16_t pulse_us);

#endif /* __INJECTOR_OUTPUT_H__ */
