#ifndef __HI_RES_TIMER_H__
#define __HI_RES_TIMER_H__


#include <stdint.h>

void initHiResTimer(uint32_t const frequency);
uint32_t hi_res_counter_val(void);

/* FIXME: better API. */
void register_crank_trigger_callback(void (* callback)(uint32_t const timestamp_us));

#endif /* __HI_RES_TIMER_H__ */
