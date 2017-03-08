#ifndef __PULSER_H__
#define __PULSER_H__

#include <stdint.h>

void pulse_start(uint32_t pulses, uint16_t pulse_us, uint16_t period_us);
void init_pulses(void);
void print_pulse_details(void);


#endif /* __PULSER_H__ */
