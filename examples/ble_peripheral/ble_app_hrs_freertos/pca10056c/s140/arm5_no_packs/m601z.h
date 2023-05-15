#ifndef APP_M601Z_H
#define APP_M601Z_H

// #include "app_error.h"

#define TSK_M601Z_STACK_SIZE					(1024)
#define TSK_M601Z_PRIORITY						(1)

/*
 * This file defines m601z interface.
 */
 
void app_m601z_freertos_init(void);

#endif // APP_M601Z_H
