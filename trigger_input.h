#ifndef __TRIGGER_INPUT_H__
#define __TRIGGER_INPUT_H__

#include "trigger_wheel_36_1.h"

/* Define the support trigger input sources. */
typedef enum trigger_signal_source_t
{
    trigger_signal_source_crank,
    trigger_signal_source_cam
} trigger_signal_source_t; 

void init_trigger_signals(trigger_wheel_36_1_context_st * const context);

#endif /* __TRIGGER_INPUT_H__ */
