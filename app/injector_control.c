#include "injector_control.h"
#include "injector_output.h"
#include "pulser.h"
#include "utils.h"
#include "main.h"
#include "main_input_timer.h"

#include <math.h>
#include <stdio.h>
#include <inttypes.h>

typedef struct injector_control_st
{
    size_t number;
    float close_angle;
    float scheduling_angle; /* Desired scheduling angle. */
    float latest_scheduling_angle; /* Actual scheduling angle. Will always be after the desired angle due to latency in the system. */
    uint32_t debug_scheduling_timestamp;

    pulser_st * pulser;

    /* TODO: - This should be a list of the outputs to control 
       with the pulser.*/
    injector_output_st * output; /* The injector output to control */
    float debug_engine_cycle_angle; 
    float degrees_to_closing_time; /* Debug */
    uint32_t debug_injector_us_until_open;
    uint32_t debug_injector_pulse_width_us;
    float debug_time_to_next_injector_close;
    uint32_t debug_latency;
    uint32_t debug_timer_base_count; 
    uint32_t close_timestamp;
    uint32_t open_timestamp; 

} injector_control_st;

static injector_control_st injector_controls[MAX_INJECTORS];

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
    return 1500;
}

#define TEST_INJECTOR_DEAD_TIME_US 500UL
uint32_t get_injector_dead_time_us(void)
{
    /* TODO - Calculate from battery voltage and configuration table. */
    return TEST_INJECTOR_DEAD_TIME_US;
}

void print_injector_debug(size_t const index)
{
    injector_control_st * const injector_control = &injector_controls[index];

    printf("inj %d time %"PRIu32" scheduling_angle %f desired %f actual close %f error %f\r\n",
           (int)index,
           injector_control->debug_scheduling_timestamp,
           injector_control->latest_scheduling_angle,
           injector_control->close_angle,
           injector_control->debug_engine_cycle_angle,
           injector_control->debug_engine_cycle_angle - injector_control->close_angle
           ); 
    printf("\r\ntime between scheduled and closing %"PRId32"\r\n",
           injector_control->close_timestamp - injector_control->debug_scheduling_timestamp);
    printf("delay %"PRIu32" width %"PRIu32"\r\n", 
           injector_control->debug_injector_us_until_open, 
           injector_control->debug_injector_pulse_width_us);
    printf("time until closing %f degrees %f\r\n",
           injector_control->debug_time_to_next_injector_close,
           injector_control->degrees_to_closing_time);
    printf("latency %"PRIu32" base %"PRIu32"\r\n\r\n", injector_control->debug_latency, injector_control->debug_timer_base_count);
    printf("pulse width %"PRIu32"\r\n", injector_control->close_timestamp - injector_control->open_timestamp);
}

static void pulser_active_callback(void * const arg)
{
    injector_control_st * const injector_control = arg;
    injector_output_st * const injector = injector_control->output;

    injector_set_active(injector);
    injector_control->open_timestamp = main_input_timer_count_get();

}

static void pulser_inactive_callback(void * const arg)
{
    injector_control_st * const injector_control = arg;
    injector_output_st * const injector = injector_control->output;

    injector_set_inactive(injector);

    injector_control->debug_engine_cycle_angle = current_engine_cycle_angle_get(); 
    injector_control->close_timestamp = main_input_timer_count_get();
}

static void injector_pulse_callback(float const crank_angle,
                                    uint32_t timestamp,
                                    void * const user_arg)
{
    injector_control_st * const injector_control = user_arg;

    UNUSED(crank_angle);
    UNUSED(timestamp);

#if 1
    float const injector_close_angle = injector_control->close_angle; /* Angle we want the injector closed. */
    float const injector_scheduling_angle = trigger_36_1_engine_cycle_angle_get(trigger_wheel); /* Current engine angle. */
    /* Determine how long it will take to rotate this many degrees. */
    injector_control->degrees_to_closing_time = (float)get_engine_cycle_degrees() - normalise_engine_cycle_angle(injector_scheduling_angle - injector_close_angle);
    float const time_to_next_injector_close = trigger_36_1_rotation_time_get(trigger_wheel, injector_control->degrees_to_closing_time);
    /* The injector pulse width must include the time taken to open the injector (dead time). */
    uint32_t const injector_pulse_width_us = get_injector_pulse_width_us() + get_injector_dead_time_us();
    uint32_t injector_us_until_open = lrintf(time_to_next_injector_close * TIMER_FREQUENCY) - injector_pulse_width_us;
    uint32_t const current_timestamp = main_input_timer_count_get();
    uint32_t const latency = current_timestamp - timestamp;
    uint32_t const injector_timer_count = pulser_timer_count_get(injector_control->pulser);
    uint32_t const timer_base_count = injector_timer_count; /* This is the time from which we base the injector event. */

    /* Remove the measured latency from the time delay before 
     * opening the injector. 
     */
    injector_us_until_open -= latency;

    if ((int)injector_us_until_open < 0)
    {
        /* This seems to occur once at startup time. Why? */
        /* XXX - TODO - Update a statistic? */
        goto done;
    }

    injector_control->latest_scheduling_angle = injector_scheduling_angle;
    injector_control->debug_latency = latency;
    injector_control->debug_timer_base_count = timer_base_count;
    injector_control->debug_time_to_next_injector_close = time_to_next_injector_close;
    injector_control->debug_injector_us_until_open = injector_us_until_open;
    injector_control->debug_injector_pulse_width_us = injector_pulse_width_us;
    injector_control->debug_scheduling_timestamp = current_timestamp;

#else
    uint32_t const timer_base_count = injector_timer_count_get(injector); /* This is the time from which we base the injector event. */
    uint32_t const injector_pulse_width_us = get_injector_pulse_width_us();
    uint32_t const injector_us_until_open = 100;

#endif
    /* Called roughly 360 degrees before the injector is due to close. */
    {
        pulser_schedule_st const pulser_schedule =
        {
            .initial_delay_us = injector_us_until_open,
            .pulse_width_us = injector_pulse_width_us,
            .programmed_at = current_timestamp
        };
        pulser_schedule_pulse(injector_control->pulser, &pulser_schedule);
    }

done:
    return;
}

static void get_injector_outputs(void)
{
    unsigned int const num_injectors = num_injectors_get();
    unsigned int const degrees_per_engine_cycle = get_engine_cycle_degrees();
    float const degrees_per_cylinder_injection = (float)degrees_per_engine_cycle / num_injectors;
    float const injector_close_angle = get_config_injector_close_angle();
    size_t index;

    for (index = 0; index < num_injectors_get(); index++)
    {
        injector_control_st * const injector_control = &injector_controls[index];

        injector_control->number = index;
        injector_control->close_angle = normalise_engine_cycle_angle(injector_close_angle + (degrees_per_cylinder_injection * index));
        injector_control->pulser = pulser_get(pulser_active_callback, 
                                              pulser_inactive_callback, 
                                              injector_control);
        
        injector_control->output = injector_output_get();
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
        injector_control_st * const injector_control = &injector_controls[index];
        float const injector_close_to_scheduling_angle = 0.0;

        injector_control->scheduling_angle = normalise_engine_cycle_angle(injector_control->close_angle
                                                                          + injector_close_to_scheduling_angle);

        trigger_36_1_register_callback(trigger_wheel,
                                       injector_control->scheduling_angle,
                                       injector_pulse_callback,
                                       injector_control);
    }
}

void injection_initialise(trigger_wheel_36_1_context_st * const trigger_wheel_in)
{
    trigger_wheel = trigger_wheel_in;

    get_injector_outputs();
    setup_injector_scheduling(trigger_wheel);
}

