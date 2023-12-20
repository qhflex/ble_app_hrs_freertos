#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "mem_manager.h"
#include "SEGGER_RTT.h"
#include "FreeRTOS.h"

#include "circular_array.h"
#include "dynamic_array.h"
#include "macro.h"
#include "type.h"

#include "peak_finding.h"
#include "feature.h"

#include "pressure.h"

static void *hijack_malloc(size_t size);
static void hijack_free(void *ptr);

// static int count = 0;

static void *hijack_malloc(size_t size)
{
    void* p = malloc(size);
    if (p == NULL) 
    {
        volatile int i = 0;
        SEGGER_RTT_printf(0, "malloc %d failed\r\n", size);
        for (;;)
        {
            i++;
        }
    }
    else
    {
        // count++;
        // SEGGER_RTT_printf(0, "-> %p %d\r\n", p, count);
    }
    return p;
}

static void hijack_free(void *ptr)
{
    // count--;
    free(ptr);
    // SEGGER_RTT_printf(0, "<- %p %d\r\n", ptr, count);
}

#define malloc  hijack_malloc //nrf_malloc
#define free    hijack_free

// extern int watchout;

#include "circular_array.c"

#include "dynamic_array.c"

#include "peak_finding.c"

#include "feature.c"

#include "pressure.c"




