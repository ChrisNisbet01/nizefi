#ifndef __SERIAL_TASK_H__
#define __SERIAL_TASK_H__

#include <stdbool.h>
#include <stddef.h>

bool set_debug_port(int port);
int debug_put_block(void * data, size_t len);

void serial_task_init(void);

#endif /* __SERIAL_TASK_H__ */

