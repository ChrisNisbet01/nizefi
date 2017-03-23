#ifndef __INJECTOR_OUTPUT_H__
#define __INJECTOR_OUTPUT_H__

#include <stdint.h>
#include <stddef.h>

#define MAX_INJECTORS 8

typedef struct injector_output_st injector_output_st;

injector_output_st * injector_output_get(void);

void injector_set_active(injector_output_st * const injector_output);
void injector_set_inactive(injector_output_st * const injector_output);


#endif /* __INJECTOR_OUTPUT_H__ */
