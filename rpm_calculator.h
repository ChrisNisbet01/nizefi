#ifndef __RPM_CALCULATOR_H__
#define __RPM_CALCULATOR_H__

typedef struct rpm_calculator_st rpm_calculator_st;

rpm_calculator_st * rpm_calculator_get(void);
void rpm_calculator_smoothing_factor_set(rpm_calculator_st * rpm_calculator, float const smoothing_factor);
float rpm_calculator_update(rpm_calculator_st * rpm_calculator, float const delta_seconds, float const revolutions);
float rpm_calculator_rpm_get(rpm_calculator_st * rpm_calculator);
float rpm_calculator_smoothed_rpm_get(rpm_calculator_st * rpm_calculator); 

#endif /* __RPM_CALCULATOR_H__ */
