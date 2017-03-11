#ifndef __TRIGGER_WHEEL_36_1_H__
#define __TRIGGER_WHEEL_36_1_H__

#include "trigger_input.h"

#include <stdint.h>

typedef struct trigger_wheel_36_1_context_st trigger_wheel_36_1_context_st;

trigger_wheel_36_1_context_st * trigger_36_1_init(void);

void trigger_36_1_handle_crank_pulse(trigger_wheel_36_1_context_st * const context, 
                               uint32_t const timestamp);

void trigger_36_1_handle_cam_pulse(trigger_wheel_36_1_context_st * const context,
                                   uint32_t const timestamp);

float trigger_36_1_rpm_get(trigger_wheel_36_1_context_st * const context);

#endif /* __TRIGGER_WHEEL_36_1_H__ */
