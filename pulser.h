#ifndef __PULSER_H__
#define __PULSER_H__

#include <stdint.h>

typedef struct timed_event_context_st timed_event_context_st; 
typedef void (* pulser_callback)(void * const user_arg);

timed_event_context_st * pulser_get(pulser_callback const active_callback, 
                                    pulser_callback const inactive_callback, 
                                    void * const user_arg);

void pulse_start(timed_event_context_st * const pulser,
                 uint32_t initial_delay_us, 
                 uint16_t pulse_us);
void init_pulses(void);
void print_pulse_details(void);
void reset_pulse_details(void);


#endif /* __PULSER_H__ */
