#ifndef __HI_RES_TIMER_H__
#define __HI_RES_TIMER_H__

#include <stdint.h>

void initHiResTimer(uint32_t periodMicrosecs, void (* appCallback)(void));
uint32_t hi_res_counter_val(void);

#endif /* __HI_RES_TIMER_H__ */
