#ifndef __RPM_CALCULATOR_H__
#define __RPM_CALCULATOR_H__

typedef struct rpm_calculator_st rpm_calculator_st;

rpm_calculator_st * rpm_calculator_get(float const smoothing_factor);
void rpm_calculator_init(rpm_calculator_st * rpm_calculator,
                         float const smoothing_factor);

void rpm_calculator_smoothing_factor_set(rpm_calculator_st * rpm_calculator, float const smoothing_factor);
float rpm_calculator_update(rpm_calculator_st * const rpm_calculator, 
                            float const degrees_of_rotation, 
                            float const delta_seconds);
float rpm_calculator_rpm_get(rpm_calculator_st * rpm_calculator);
float rpm_calculator_smoothed_rpm_get(rpm_calculator_st * rpm_calculator); 
float rpm_calcuator_get_degrees_turned(rpm_calculator_st * const rpm_calculator, float const seconds);
float rpm_calcuator_get_time_to_rotate_angle(rpm_calculator_st * const rpm_calculator, float const degrees);

#endif /* __RPM_CALCULATOR_H__ */
