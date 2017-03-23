#ifndef __MAIN_H__
#define __MAIN_H__

/* Debug file. Shouldn't be any calls into functions in this 
 * file. 
 */

unsigned int get_engine_cycle_degrees(void);
float get_config_injector_close_angle(void);
float current_engine_cycle_angle_get(void);
float get_ignition_advance(void);
float get_ignition_maximum_advance(void);
uint32_t get_ignition_dwell_us(void);

#endif /* __MAIN_H__ */
