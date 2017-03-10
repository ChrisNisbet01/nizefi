#include "rpm_calculator.h"

#include <stdlib.h>
#include <stdbool.h>

#define SECONDS_PER_MINUTE 60.0

struct rpm_calculator_st
{
    bool had_sample;
    float smoothing_factor;

    float raw_rpm;
    float smoothed_rpm;

}; 

static rpm_calculator_st rpm_calculator_context;

static void rpm_calculator_init(rpm_calculator_st * rpm_calculator)
{
    rpm_calculator->had_sample = false;
    rpm_calculator->smoothing_factor = 1.0;
    rpm_calculator->raw_rpm = 0.0;
    rpm_calculator->smoothed_rpm = 0.0;
}

rpm_calculator_st * rpm_calculator_get(void)
{
    rpm_calculator_st * const rpm_calculator = &rpm_calculator_context;

    rpm_calculator_init(rpm_calculator);

    return rpm_calculator;
}

void rpm_calculator_smoothing_factor_set(rpm_calculator_st * rpm_calculator, float const smoothing_factor)
{
    rpm_calculator->smoothing_factor = smoothing_factor;
}

float rpm_calculator_update(rpm_calculator_st * rpm_calculator, float const delta_seconds, float const revolutions)
{
    float const interval = delta_seconds;
    float const rpm = SECONDS_PER_MINUTE * revolutions / interval;

    rpm_calculator->raw_rpm = rpm;

    if (rpm_calculator->had_sample)
    {
        /* Merge this new rpm into the current rpm. The amount of effect
         * from the new sample is proportional to the amount of time 
         * between samples. 
         */
        rpm_calculator->smoothed_rpm = (rpm_calculator->smoothing_factor * rpm_calculator->raw_rpm) +
            ((1.0 - rpm_calculator->smoothing_factor) * rpm_calculator->smoothed_rpm);
    }
    else
    {
        rpm_calculator->smoothed_rpm = 0.0;
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
