#ifndef __UTILS_H__
#define __UTILS_H__

#define ARRAY_SIZE(a) (sizeof((a)) / sizeof((a)[0]))
#define UNUSED(x) ((void)(x))

#define container_of(ptr, type, member) ({                      \
        const typeof( ((type *)0)->member ) *__mptr = (ptr);    \
        (type *)( (char *)__mptr - offsetof(type,member) );})

float normalise_crank_angle(float const crank_angle);
float normalise_engine_cycle_angle(float const angle);

#endif /* __UTILS_H__ */
