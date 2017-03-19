#include "ignition_control.h"
#include "ignition_output.h"
#include "main.h"
#include "hi_res_timer.h"
#include "utils.h"
#include "pulser.h"

#include <stdio.h>
#include <math.h>

typedef struct ignition_control_st
{
    size_t number;
    float spark_angle;

    float scheduling_angle; /* Desired scheduling angle. */
    float latest_scheduling_angle; /* Actual scheduling angle. Will always be after the desired angle due to latency in the system. */

    pulser_st * pulser;

    /* TODO: - This should be a list of the outputs to control 
       with the pulser.*/
    ignition_output_st * output; /* The ignition output to control */
    float debug_engine_cycle_angle; /* engine angle when the latest spark occured. */

} ignition_control_st;

static ignition_control_st ignition_controls[MAX_IGNITIONS];

/* temp debug. Don't have multiple copies of this 
 * trigger_wheel pointer lying around the place. 
 */
static trigger_wheel_36_1_context_st * trigger_wheel;

static unsigned int num_ignition_outputs_get(void)
{
    /* XXX - Get the value from the configuration. */
    return 4;
}

float debug_desired_spark_angle;

void print_ignition_debug(size_t const index)
{
    ignition_control_st * const ignition_control = &ignition_controls[index];
    float const actual_spark_angle = ignition_control->debug_engine_cycle_angle;
    float const desired_spark_angle = ignition_control->spark_angle;

    /* Note that the desired and actual values don't match up 
     * because of the delay between scheduling the pulse and when 
     * it actually happens. In the meantime, another pulse gets 
     * scheduled which mucks things up. Some better debug would be 
     * nice. 
     */
    printf("ignition_scheduling_angle # %u %f desired %f actual spark %f error %f\r\n",
           (int)index,
           ignition_control->latest_scheduling_angle,
           desired_spark_angle,
           actual_spark_angle,
           actual_spark_angle - desired_spark_angle
           );
}

static void pulser_active_callback(void * const arg)
{
    ignition_control_st * const ignition_control = arg;
    ignition_output_st * const ignition = ignition_control->output;

    /* TODO: support configurations where multiple outputs may 
     * change state at the same times. 
    */
    ignition_set_active(ignition);
}

static void pulser_inactive_callback(void * const arg)
{
    ignition_control_st * const ignition_control = arg;
    ignition_output_st * const ignition = ignition_control->output;

    /* TODO: support configurations where multiple outputs may 
     * change state at the same times. 
    */
    ignition_set_inactive(ignition);

    ignition_control->debug_engine_cycle_angle = current_engine_cycle_angle_get();
}

void ignition_pulse_callback(float const crank_angle,
                             uint32_t timestamp,
                             void * const user_arg)
{
    ignition_control_st * const ignition_control = user_arg;

    UNUSED(crank_angle);
    UNUSED(timestamp);

#if 1
    unsigned int const num_ignitions = num_ignition_outputs_get();
    unsigned int degrees_per_engine_cycle = get_engine_cycle_degrees();
    float const degrees_per_cylinder_ignition = (float)degrees_per_engine_cycle / num_ignitions;
    float const ignition_spark_angle = normalise_engine_cycle_angle((degrees_per_cylinder_ignition * ignition_control->number) - get_ignition_advance());
    float const ignition_scheduling_angle = trigger_36_1_engine_cycle_angle_get(trigger_wheel); /* Current engine angle. */
    /* Determine how long it will take to rotate this many degrees. */
    float const time_to_next_spark = trigger_36_1_rotation_time_get(trigger_wheel,
                                                                    (float)degrees_per_engine_cycle - normalise_crank_angle(ignition_scheduling_angle - ignition_spark_angle));
    /* The injector pulse width must include the time taken to open the injector (dead time). */
    uint32_t const ignition_pulse_width_us = get_ignition_dwell_us();
    uint32_t const ignition_us_until_open = lrintf(time_to_next_spark * TIMER_FREQUENCY) - ignition_pulse_width_us;
    uint32_t const latency = hi_res_counter_val() - timestamp;
    uint32_t const ignition_timer_count = pulser_timer_count_get(ignition_control->pulser);
    uint32_t const timer_base_count = (ignition_timer_count - latency) & 0xffff; /* This is the time from which we base the injector event. */

    ignition_control->latest_scheduling_angle = ignition_scheduling_angle;
    ignition_control->spark_angle = ignition_spark_angle;

    if ((int)ignition_us_until_open < 0)
    {
        /* XXX - TODO - Update a statistic? */
        goto done;
    }

#else
    (void)crank_angle;
    (void)timestamp;
    uint32_t const ignition_pulse_width_us = 2000;
    uint32_t const ignition_us_until_open = 100;
    uint32_t const injector_timer_count = injector_timer_count_get(injector);
    uint32_t const timer_base_count = injector_timer_count; /* This is the time from which we base the injector event. */

#endif
    {
        pulser_schedule_st const pulser_schedule =
        {
            .base_time = timer_base_count,
            .initial_delay_us = ignition_us_until_open,
            .pulse_width_us = ignition_pulse_width_us
        };
        pulser_schedule_pulse(ignition_control->pulser, &pulser_schedule);
    }

done:
    return;
}

static void get_ignition_outputs(void)
{
#if 1
    unsigned int const num_ignitions = num_ignition_outputs_get();
    size_t index;

    /* TODO: Ingition advance obviously varies. This is all just debug. */
    for (index = 0; index < num_ignitions; index++)
    {
        ignition_control_st * ignition_control = &ignition_controls[index];

        ignition_control->number = index;
        ignition_control->pulser = pulser_get(pulser_active_callback,
                                              pulser_inactive_callback,
                                              ignition_control);

        ignition_control->output = ignition_output_get();

    }
#else
    size_t index;
    float const ignition_spark_angle = get_ignition_advance();

    for (index = 0; index < num_ignition_outputs_get(); index++)
    {
        ignitions[index] = ignition_output_get(index, ignition_spark_angle);
    }
#endif
}

static void setup_ignition_scheduling(trigger_wheel_36_1_context_st * const trigger_wheel)
{
    unsigned int const num_ignitions = num_ignition_outputs_get();
    unsigned int degrees_per_engine_cycle = get_engine_cycle_degrees();
    float const degrees_per_cylinder_ignition = (float)degrees_per_engine_cycle / num_ignitions;
    float const maximum_advance = get_ignition_maximum_advance();
    float const tolerance = 10.0;
    float const ignition_scheduling_angle = normalise_engine_cycle_angle((float)degrees_per_engine_cycle - (360.0 + maximum_advance + tolerance));
    size_t index;

    /* By doing the scheduling 1 revolution before the spark there should be enough time to get the start 
     * of the ignition pulse scheduled in at all realistic RPMs. 
     */
    for (index = 0; index < num_ignitions; index++)
    {
        ignition_control_st * const ignition_control = &ignition_controls[index];

        ignition_control->scheduling_angle = normalise_engine_cycle_angle(ignition_scheduling_angle + (degrees_per_cylinder_ignition * index));

        trigger_36_1_register_callback(trigger_wheel,
                                       ignition_control->scheduling_angle,
                                       ignition_pulse_callback,
                                       ignition_control);
    }
}

void ignition_initialise(trigger_wheel_36_1_context_st * const trigger_wheel_in)
{
    trigger_wheel = trigger_wheel_in;

    get_ignition_outputs();
    setup_ignition_scheduling(trigger_wheel);
}

