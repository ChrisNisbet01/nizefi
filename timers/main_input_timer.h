#ifndef __MAIN_INPUT_TIMER_H__
#define __MAIN_INPUT_TIMER_H__


#include <stdint.h>

/* Define the frequency of both the trigger input and output 
 * pulse timers. 
 */
#define TIMER_FREQUENCY 1000000


void main_input_timer_init(uint32_t const frequency);
uint32_t main_input_timer_count_get(void);

/* FIXME: better API. */
void register_crank_trigger_callback(void (* callback)(uint32_t const timestamp_us));

#endif /* __MAIN_INPUT_TIMER_H__ */
