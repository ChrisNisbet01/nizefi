#ifndef __TRIGGER_INPUT_H__
#define __TRIGGER_INPUT_H__

/* Define the support trigger input sources. */
typedef enum trigger_signal_source_t
{
    trigger_signal_source_crank,
    trigger_signal_source_cam
} trigger_signal_source_t; 

void init_trigger_signals(void);

#endif /* __TRIGGER_INPUT_H__ */
