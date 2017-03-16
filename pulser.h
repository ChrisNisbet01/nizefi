#ifndef __PULSER_H__
#define __PULSER_H__

#include <stdint.h>

typedef struct pulser_st pulser_st;
typedef void (* pulser_callback)(void * const user_arg);

pulser_st * pulser_get(pulser_callback const active_callback,
                    pulser_callback const inactive_callback, 
                    void * const user_arg);

void pulse_start(pulser_st * const pulser,
                 uint32_t base_count,
                 uint32_t initial_delay_us,
                 uint_fast16_t pulse_us);
void init_pulsers(void);
uint32_t pulser_timer_count_get(pulser_st const * const pulser);

void print_pulse_details(void);
void reset_pulse_details(void);


#endif /* __PULSER_H__ */
