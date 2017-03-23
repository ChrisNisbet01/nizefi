#ifndef __TRIGGER_WHEEL_36_1_H__
#define __TRIGGER_WHEEL_36_1_H__

typedef struct trigger_wheel_36_1_context_st trigger_wheel_36_1_context_st;

#include <stdint.h>


typedef void (* trigger_event_callback)(float const crank_angle_atdc, 
                                        uint32_t timestamp,
                                        void * const arg); 

trigger_wheel_36_1_context_st * trigger_36_1_init(void);

void trigger_36_1_register_callback(trigger_wheel_36_1_context_st * const context,
                                    float const engine_degrees,
                                    trigger_event_callback callback,
                                    void * const user_arg);

void trigger_36_1_handle_crank_pulse(trigger_wheel_36_1_context_st * const context, 
                                     uint32_t const timestamp);

void trigger_36_1_handle_cam_pulse(trigger_wheel_36_1_context_st * const context,
                                   uint32_t const timestamp);

float trigger_36_1_rpm_get(trigger_wheel_36_1_context_st * const context);

float trigger_36_1_crank_angle_get(trigger_wheel_36_1_context_st * const context);

float trigger_36_1_engine_cycle_angle_get(trigger_wheel_36_1_context_st * const context);
float trigger_36_1_rotation_time_get(trigger_wheel_36_1_context_st * const context, float const rotation_angle);

#endif /* __TRIGGER_WHEEL_36_1_H__ */
