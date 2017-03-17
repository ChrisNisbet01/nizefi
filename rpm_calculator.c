#include "rpm_calculator.h"
#include "utils.h"

#include <stdlib.h>
#include <stdbool.h>
#include <math.h>

#define RPM_TO_DEGREES_PER_SECOND_FACTOR 6.0
#define MIN_SECONDS_BETWEEN_UPDATES 0.1

typedef float (* rpm_calculator_update_fn)(rpm_calculator_st * const rpm_calculator);

struct rpm_calculator_st
{
    float smoothing_factor;

    float raw_rpm;
    float smoothed_rpm;
    float smoothed_degrees_per_second;
    float degrees_per_second_acceleration;

    float accumulated_degrees;
    float accumulated_seconds;


    rpm_calculator_update_fn update_fn;
}; 

static rpm_calculator_st rpm_calculator_context;

void rpm_calculator_smoothing_factor_set(rpm_calculator_st * rpm_calculator, float const smoothing_factor)
{
    rpm_calculator->smoothing_factor = smoothing_factor;
}

static void rpm_calculator_calculate_rpm(rpm_calculator_st * const rpm_calculator, 
                                         float const degrees_of_rotation, 
                                         float const delta_seconds)
{
    float const degrees_per_second = degrees_of_rotation / delta_seconds;
    float const previous_smoothed_degrees_per_second = rpm_calculator->smoothed_degrees_per_second;

    rpm_calculator->raw_rpm = degrees_per_second / RPM_TO_DEGREES_PER_SECOND_FACTOR;

    /* Merge this rate into the current rate. Note that the 
     * smoothing will be affected by the time between updates.
     */
    rpm_calculator->smoothed_degrees_per_second = (rpm_calculator->smoothing_factor * degrees_per_second) +
        ((1.0 - rpm_calculator->smoothing_factor) * rpm_calculator->smoothed_degrees_per_second);

    rpm_calculator->smoothed_rpm = rpm_calculator->smoothed_degrees_per_second / RPM_TO_DEGREES_PER_SECOND_FACTOR;

    rpm_calculator->degrees_per_second_acceleration = (rpm_calculator->smoothed_degrees_per_second - previous_smoothed_degrees_per_second) 
                                                        / delta_seconds;
    /* TODO: Factor in the rate of change of the RPM. */
}

static float rpm_calculator_update_had_two_samples(rpm_calculator_st * const rpm_calculator)
{
    if (rpm_calculator->accumulated_seconds >= MIN_SECONDS_BETWEEN_UPDATES)
    {
        rpm_calculator_calculate_rpm(rpm_calculator, rpm_calculator->accumulated_degrees, rpm_calculator->accumulated_seconds);
        rpm_calculator->accumulated_degrees = 0.0;
        rpm_calculator->accumulated_seconds = 0.0;
    }

    return rpm_calculator->smoothed_rpm;
}

static float rpm_calculator_update_no_samples(rpm_calculator_st * const rpm_calculator)
{
    rpm_calculator->smoothed_rpm = 0.0;
    rpm_calculator->smoothed_degrees_per_second = 0.0;
    rpm_calculator->degrees_per_second_acceleration = 0.0;
    rpm_calculator->update_fn = rpm_calculator_update_had_two_samples;

    return rpm_calculator->smoothed_rpm;
}

float rpm_calculator_update(rpm_calculator_st * const rpm_calculator, 
                            float const degrees_of_rotation, 
                            float const delta_seconds)
{
    rpm_calculator->accumulated_degrees += degrees_of_rotation;
    rpm_calculator->accumulated_seconds += delta_seconds;

    return rpm_calculator->update_fn(rpm_calculator);
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
     *   if factoring in acceleration we do this...
     *   v2 ^ 2 = v1 ^ 2 + 2as, where v2 is the final velocity
     *   (degrees/sec), v1 is the initial velocity (degrees/sec),
     *   'a' is the acceleration (degrees/sec ^ 2), 's' the distance
     *   (degrees).
     *   calculate v2  = sqrt(v1 ^ 2 + 2as)
     *   - find average velocity.
     *   - time = angle / average velocity.
     */
    float seconds;
    float temp;

    temp = (rpm_calculator->smoothed_degrees_per_second * rpm_calculator->smoothed_degrees_per_second);
    temp += 2.0 * rpm_calculator->degrees_per_second_acceleration * degrees;
    if (temp < 0.0)
    {
        seconds = NAN;
        goto done;
    }
    temp = sqrtf(temp); /* This is the average velocity. */

    if (temp < MINIMUM_VALID_ROTATION_SPEED_DEGREES_PER_SECOND)
    {
        seconds = NAN;
        goto done;
    }

    if (rpm_calculator->smoothed_degrees_per_second >= MINIMUM_VALID_ROTATION_SPEED_DEGREES_PER_SECOND)
    {
        seconds = degrees / temp;
    }
    else
    {
        seconds = NAN;
    }

done:

    return seconds;
}

void rpm_calculator_init(rpm_calculator_st * rpm_calculator,
                         float const smoothing_factor)
{
    rpm_calculator->smoothing_factor = smoothing_factor;
    rpm_calculator->raw_rpm = 0.0;
    rpm_calculator->smoothed_rpm = 0.0;
    rpm_calculator->smoothed_degrees_per_second = 0.0;
    rpm_calculator->degrees_per_second_acceleration = 0.0;
    rpm_calculator->accumulated_degrees = 0.0;
    rpm_calculator->accumulated_seconds = 0.0; 

    rpm_calculator->update_fn = rpm_calculator_update_no_samples;
}

rpm_calculator_st * rpm_calculator_get(float const smoothing_factor)
{
    rpm_calculator_st * const rpm_calculator = &rpm_calculator_context;

    rpm_calculator_init(rpm_calculator, smoothing_factor);

    return rpm_calculator;
}

