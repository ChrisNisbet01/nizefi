#include "injector_control.h"
#include "injector_output.h"
#include "utils.h"
#include "main.h"
#include "hi_res_timer.h"

#include <math.h>
#include <stdio.h>
#include <inttypes.h>

static injector_output_st * injectors[MAX_INJECTORS]; 

/* temp debug. Don't have multiple copies of this 
 * trigger_wheel pointer lying around the place. 
 */
static trigger_wheel_36_1_context_st * trigger_wheel;

static unsigned int num_injectors_get(void)
{
    /* XXX - Get the value from the configuration. */
    return 4;
}

uint32_t get_injector_pulse_width_us(void)
{
    /* TODO - Calculate pulse width. */
    return 800;
}

#define TEST_INJECTOR_DEAD_TIME_US 500UL
uint32_t get_injector_dead_time_us(void)
{
    /* TODO - Calculate from battery voltage and configuration table. */
    return TEST_INJECTOR_DEAD_TIME_US;
}

/* Store some debug values */
static uint32_t debug_injector_us_until_open;
static uint32_t debug_injector_pulse_width_us;
float debug_injector_close_angle_atdc;
float debug_injector_scheduling_angle;
float debug_time_to_next_injector_close;
uint32_t debug_latency;
uint32_t debug_timer_base_count;
int debug_injector_number;

void print_injector_debug(void)
{
    float get_angle_when_injector_closed(void);

    printf("\r\ninj %d\r\n", debug_injector_number);
    printf("delay %"PRIu32" width %"PRIu32"\r\n", debug_injector_us_until_open, debug_injector_pulse_width_us);
    printf("close angle %f scheduling angle %f time to close %f\r\n",
           debug_injector_close_angle_atdc,
           debug_injector_scheduling_angle,
           debug_time_to_next_injector_close);
    printf("latency %"PRIu32" base %"PRIu32"\r\n\r\n", debug_latency, debug_timer_base_count);
    printf("\r\nactual close %f\r\n", get_angle_when_injector_closed());
}

static void injector_pulse_callback(float const crank_angle,
                                    uint32_t timestamp,
                                    void * const user_arg)
{
    injector_output_st * const injector = user_arg;

    UNUSED(crank_angle);
    UNUSED(timestamp);

#if 1
    float const injector_close_angle = injector_close_angle_get(injector); /* Angle we want the injector closed. */
    float const injector_scheduling_angle = trigger_36_1_engine_cycle_angle_get(trigger_wheel); /* Current engine angle. */
    /* Determine how long it will take to rotate this many degrees. */
    float const time_to_next_injector_close = trigger_36_1_rotation_time_get(trigger_wheel,
                                                                             (float)get_engine_cycle_degrees() - normalise_engine_cycle_angle(injector_scheduling_angle - injector_close_angle));
    /* The injector pulse width must include the time taken to open the injector (dead time). */
    uint32_t const injector_pulse_width_us = get_injector_pulse_width_us() + get_injector_dead_time_us();
    uint32_t const injector_us_until_open = lrintf(time_to_next_injector_close * TIMER_FREQUENCY) - injector_pulse_width_us;
    uint32_t const latency = hi_res_counter_val() - timestamp;
    uint32_t const injector_timer_count = injector_timer_count_get(injector);
    uint32_t const timer_base_count = (injector_timer_count - latency) & 0xffff; /* This is the time from which we base the injector event. */

    if ((int)injector_us_until_open < 0)
    {
        /* This seems to occur once at startup time. */
        /* XXX - TODO - Update a statistic? */
        goto done;
    }

    debug_latency = latency;
    debug_timer_base_count = timer_base_count;
    debug_injector_close_angle_atdc = injector_close_angle;
    debug_injector_scheduling_angle = injector_scheduling_angle;
    debug_time_to_next_injector_close = time_to_next_injector_close;
    debug_injector_us_until_open = injector_us_until_open;
    debug_injector_pulse_width_us = injector_pulse_width_us;
    debug_injector_number = injector_number_get(injector);
#else
    uint32_t const timer_base_count = injector_timer_count_get(injector); /* This is the time from which we base the injector event. */
    uint32_t const injector_pulse_width_us = get_injector_pulse_width_us();
    uint32_t const injector_us_until_open = 100;

#endif
    /* Called roughly 360 degrees before the injector is due to close. */
    injector_pulse_schedule(injector, timer_base_count, injector_us_until_open, injector_pulse_width_us);

done:
    return;
}

static void get_injector_outputs(void)
{
    unsigned int const num_injectors = num_injectors_get();
    unsigned int degrees_per_engine_cycle = get_engine_cycle_degrees();
    float const degrees_per_cylinder_injection = (float)degrees_per_engine_cycle / num_injectors;
    float const injector_close_angle = get_config_injector_close_angle();
    size_t index;

    for (index = 0; index < num_injectors_get(); index++)
    {
        injectors[index] = injector_output_get(index,
                                               normalise_engine_cycle_angle(injector_close_angle + (degrees_per_cylinder_injection * index)));
    }
}

static void setup_injector_scheduling(trigger_wheel_36_1_context_st * const trigger_wheel)
{
    unsigned int const num_injectors = num_injectors_get();
    size_t index;

    /* By using the angle at which the injector closes to schedule the next event there should be enough time to 
       get the start of the injector pulse scheduled in. 
       This is with the assumption that that the injector duty cycle never goes beyond something like 80-85%.
    */
    for (index = 0; index < num_injectors; index++)
    {
        injector_output_st * const injector = injectors[index];
        float const injector_close_to_schedule_delay = 0.0;

        float const injector_scheduling_angle = normalise_engine_cycle_angle(injector_close_angle_get(injector)
                                                                             + injector_close_to_schedule_delay);
        /* Temp debug add in some slack to help prevent event overlap. 
           Means that injector duty is limited to 100  * (720-20) / 720%. 
         */

        trigger_36_1_register_callback(trigger_wheel,
                                       normalise_engine_cycle_angle(injector_scheduling_angle),
                                       injector_pulse_callback,
                                       injector);
    }
}

void injection_initialise(trigger_wheel_36_1_context_st * const trigger_wheel_in)
{
    trigger_wheel = trigger_wheel_in;

    get_injector_outputs();
    setup_injector_scheduling(trigger_wheel);
}

