#include <stdio.h>
#include <sleep.h> // For usleep, if needed (though busy-wait loop is used)
#include "platform.h"
#include "xil_printf.h"
#include "xil_io.h"
#include "xil_mmu.h"
#include "xpseudo_asm.h"
#include "xil_exception.h"
#include "xil_cache.h" // Needed for cache operations, if any
#include "shared_print.h"

int main() {

    Xil_SetTlbAttributes(SHARED_MEM_BASE, NORM_NONCACHE_SHARED); // Critical for coherency!

    init_platform(); // Initialize platform for Core 1

    xil_printf("Core 1 awake!!\r\n");

    // init_print_buffer(); // Since this core starts SECOND, we should do this in the other core!

    print_handler_loop();

    cleanup_platform(); // Clean up platform resources
    return 0;
}
