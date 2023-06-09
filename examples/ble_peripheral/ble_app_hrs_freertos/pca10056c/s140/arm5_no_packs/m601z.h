#ifndef APP_M601Z_H
#define APP_M601Z_H

// #include "app_error.h"

#define TSK_M601Z_STACK_SIZE          (64)	// the unit is word, 4-bytes
#define TSK_M601Z_PRIORITY            (1)

/*
 * This file defines m601z interface.
 */
void app_m601z_freertos_init(void);

#endif // APP_M601Z_H
