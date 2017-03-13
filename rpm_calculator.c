#include "rpm_calculator.h"

#include <stdlib.h>
#include <stdbool.h>
#include <math.h>

#define RPM_TO_DEGREES_PER_SECOND_FACTOR 6.0

struct rpm_calculator_st
{
    bool had_sample;
    float smoothing_factor;

    float raw_rpm;
    float smoothed_rpm;
    float smoothed_degrees_per_second;
    float degrees_per_second_acceleration;
}; 

static rpm_calculator_st rpm_calculator_context;

void rpm_calculator_init(rpm_calculator_st * rpm_calculator, 
                         float const smoothing_factor)
{
    rpm_calculator->had_sample = false;
    rpm_calculator->smoothing_factor = smoothing_factor;
    rpm_calculator->raw_rpm = 0.0;
    rpm_calculator->smoothed_rpm = 0.0;
    rpm_calculator->smoothed_degrees_per_second = 0.0;
    rpm_calculator->degrees_per_second_acceleration = 0.0;
}

rpm_calculator_st * rpm_calculator_get(float const smoothing_factor)
{
    rpm_calculator_st * const rpm_calculator = &rpm_calculator_context;

    rpm_calculator_init(rpm_calculator, smoothing_factor);

    return rpm_calculator;
}

void rpm_calculator_smoothing_factor_set(rpm_calculator_st * rpm_calculator, float const smoothing_factor)
{
    rpm_calculator->smoothing_factor = smoothing_factor;
}

float rpm_calculator_update(rpm_calculator_st * rpm_calculator, float const degrees_of_rotation, float const delta_seconds)
{
    float const degrees_per_second = degrees_of_rotation / delta_seconds;

    rpm_calculator->raw_rpm = degrees_per_second / RPM_TO_DEGREES_PER_SECOND_FACTOR;

    if (rpm_calculator->had_sample)
    {
        float const smoothed_degrees_per_second = rpm_calculator->smoothed_degrees_per_second;

        /* Merge this rate into the current rate. Note that the 
         * smoothing will be affected by the time between updates.
         */
        rpm_calculator->smoothed_degrees_per_second = (rpm_calculator->smoothing_factor * degrees_per_second) +
            ((1.0 - rpm_calculator->smoothing_factor) * rpm_calculator->smoothed_degrees_per_second);

        rpm_calculator->smoothed_rpm = rpm_calculator->smoothed_degrees_per_second / RPM_TO_DEGREES_PER_SECOND_FACTOR;

        rpm_calculator->degrees_per_second_acceleration = (rpm_calculator->smoothed_degrees_per_second - smoothed_degrees_per_second) / delta_seconds;
    }
    else
    {
        rpm_calculator->smoothed_rpm = 0.0;
        rpm_calculator->smoothed_degrees_per_second = 0.0;
        rpm_calculator->degrees_per_second_acceleration = 0.0;
        rpm_calculator->had_sample = true;
    }

    /* TODO: Factor in the rate of change of the RPM. */
    return rpm_calculator->smoothed_rpm;
}

float rpm_calculator_rpm_get(rpm_calculator_st * rpm_calculator)
{
    return rpm_calculator->raw_rpm;
}

float rpm_calculator_smoothed_rpm_get(rpm_calculator_st * rpm_calculator)
{
    return rpm_calculator->smoothed_rpm;
}

float rpm_calcuator_get_degrees_turned(rpm_calculator_st * const rpm_calculator, float const seconds)
{
    /* Given an amount of time, calculate how many degrees of 
     * rotation that amounts to. 
     * angle = degrees/second * seconds + 0.5 * degrees/s^2 * 
     * seconds * seconds. 
     */
    float degrees_of_rotation;

    degrees_of_rotation = (rpm_calculator->smoothed_degrees_per_second * seconds)
        + (0.5 * rpm_calculator->degrees_per_second_acceleration * seconds * seconds);

    return degrees_of_rotation;
}

#define MINIMUM_VALID_ROTATION_SPEED_DEGREES_PER_SECOND 20.0

float rpm_calcuator_get_time_to_rotate_angle(rpm_calculator_st * const rpm_calculator, float const degrees)
{
    /* Given an amount of rotation (hopefully small so that 
     * acceleration won't change much in that time), calculate how 
     * much time it will take to rotate this many degrees of 
     * rotation. time = degrees / degrees_per_second. XXX 
     * - Need to take into account acceleration. 
     */
    float seconds;

    if (rpm_calculator->smoothed_degrees_per_second >= MINIMUM_VALID_ROTATION_SPEED_DEGREES_PER_SECOND)
    {
        seconds = degrees / rpm_calculator->smoothed_degrees_per_second;
    }
    else
    {
        seconds = NAN;
    }

    return seconds;
}
