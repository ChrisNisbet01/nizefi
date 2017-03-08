#ifndef __TIMED_EVENTS_H__
#define __TIMED_EVENTS_H__

#include <stdint.h>

typedef struct timer_channel_context_st timer_channel_context_st;

void timed_events_init(uint32_t timer_frequency);
timer_channel_context_st * timer_channel_get(void(* const cb)(void * const arg), void * const arg);
void timer_channel_free(timer_channel_context_st * const channel);
void timer_channel_schedule_followup_event(timer_channel_context_st * const channel, uint32_t const delay_us); 
void timer_channel_schedule_new_event(timer_channel_context_st * const channel, uint32_t const delay_us); 

void timer_channel_schedule_new_based_event(timer_channel_context_st * const channel, 
                                            uint32_t const base, 
                                            uint32_t const delay_us);

uint32_t timer_channel_get_current_time(timer_channel_context_st * const channel);
void timer_channel_disable(timer_channel_context_st * const channel);

#endif /* __TIMED_EVENTS_H__ */
