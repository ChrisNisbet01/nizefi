#ifndef __IGNITION_OUTPUT_H__
#define __IGNITION_OUTPUT_H__

#include <stdint.h>
#include <stddef.h>

#define MAX_IGNITIONS 8

typedef struct ignition_output_st ignition_output_st;

ignition_output_st * ignition_output_get();

void ignition_set_active(ignition_output_st * const ignition_output);
void ignition_set_inactive(ignition_output_st * const ignition_output);

#endif /* __IGNITION_OUTPUT_H__ */
