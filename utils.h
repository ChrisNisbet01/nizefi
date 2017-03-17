#ifndef __UTILS_H__
#define __UTILS_H__

#define ARRAY_SIZE(a) (sizeof((a)) / sizeof((a)[0]))
#define UNUSED(x) ((void)(x))

float normalise_crank_angle(float const crank_angle);
float normalise_engine_cycle_angle(float const angle);

#endif /* __UTILS_H__ */
