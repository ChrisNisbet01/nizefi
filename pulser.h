#ifndef __PULSER_H__
#define __PULSER_H__

#include <stdint.h>

typedef struct pulser_st pulser_st;
typedef void (* pulser_callback)(void * const user_arg);

pulser_st * pulser_get(pulser_callback const active_callback,
                    pulser_callback const inactive_callback, 
                    void * const user_arg);

void pulser_schedule_pulse(pulser_st * const context, 
                           uint32_t const base_time, 
                           uint32_t const initial_delay_us, 
                           uint_fast16_t const pulse_us);

void init_pulsers(void);
uint32_t pulser_timer_count_get(pulser_st const * const pulser);


#endif /* __PULSER_H__ */
