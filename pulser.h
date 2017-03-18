#ifndef __PULSER_H__
#define __PULSER_H__

#include <stdint.h>

typedef struct pulser_st pulser_st;
typedef void (* pulser_callback)(void * const user_arg);
typedef struct pulser_schedule_st
{
    uint32_t base_time; /* Timer clock value that the initial delay is based from. */
    uint32_t initial_delay_us; /* The initial delay to the start of the pulse. */
    uint_fast16_t pulse_width_us; /* The pulse width. */
} pulser_schedule_st;

pulser_st * pulser_get(pulser_callback const active_callback,
                    pulser_callback const inactive_callback, 
                    void * const user_arg);

void pulser_schedule_pulse(pulser_st * const context,
                           pulser_schedule_st const * const pulser_schedule);

void init_pulsers(void);
uint32_t pulser_timer_count_get(pulser_st const * const pulser);

#endif /* __PULSER_H__ */
