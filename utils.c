#include "utils.h"

static float normalise_angle(float const angle_in, float const maximum_angle)
{
    float angle = angle_in;

    while (angle < 0.0)
    {
        angle += maximum_angle;
    }
    while (angle >= maximum_angle)
    {
        angle -= maximum_angle;
    }

    return angle;
}

float normalise_crank_angle(float const crank_angle)
{
    return normalise_angle(crank_angle, 360.0);
}

float normalise_engine_cycle_angle(float const crank_angle)
{
    return normalise_angle(crank_angle, 720.0);
}

