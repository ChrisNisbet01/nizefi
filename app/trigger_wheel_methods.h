#ifndef __TRIGGER_WHEEL_METHODS_H__
#define __TRIGGER_WHEEL_METHODS_H__

typedef struct trigger_wheel_st trigger_wheel_st;

#include <stdint.h>


typedef void (* trigger_event_callback)(float const crank_angle_atdc,
                                        uint32_t timestamp,
                                        void * const arg);

typedef trigger_wheel_st * (* trigger_wheel_init_fn)(void);
typedef void (* trigger_wheel_register_callback_fn)(trigger_wheel_st * const context,
                                                 float const engine_degrees,
                                                 trigger_event_callback callback,
                                                 void * const user_arg);

typedef void (* trigger_wheel_handle_crank_pulse_fn)(trigger__st * const context,
                                                uint32_t const timestamp);

typedef void (* trigger_wheel_handle_cam_pulse_fn)(trigger_wheel__st * const context,
                                               uint32_t const timestamp);

typedef float (* trigger_wheel_rpm_get_fn)(trigger_wheel__st * const context);

typedef float (* trigger_wheel_crank_angle_get_fn)(trigger_wheel__st * const context);

typedef float (* trigger_wheel_engine_cycle_angle_get_fn)(trigger_wheel_st * const context);

typedef struct trigger_wheel_methods_st
{
    trigger_wheel_init_fn init; 
    trigger_wheel_register_callback_fn register_callback;
    trigger_wheel_handle_crank_pulse_fn handle_crank_pulse;
    trigger_wheel_handle_cam_pulse_fn handle_cam_pulse;
    trigger_wheel_rpm_get_fn rpm_get;
    trigger_wheel_crank_angle_get_fn crank_angle_get;
    trigger_wheel_engine_cycle_angle_get_fn cycle_angle_get;
} trigger_wheel_methods_st;

#endif /* __TRIGGER_WHEEL_METHODS_H__ */
