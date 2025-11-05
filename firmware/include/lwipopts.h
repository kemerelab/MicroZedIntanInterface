/**
 * Custom LWIP Options Override
 *
 * This file overrides default Xilinx LWIP configuration to enable
 * link status callbacks for network cable hotplug detection.
 *
 * This file is found BEFORE the BSP's xlwipconfig.h due to the -I flag
 * added in create_vitis_project.py. We define our overrides, then include
 * the base Xilinx configuration which will respect our settings due to
 * #ifndef guards.
 */

#ifndef __LWIPOPTS_H__
#define __LWIPOPTS_H__

/* ========================================================================
 * CUSTOM OVERRIDES
 * ======================================================================== */

/**
 * LWIP_NETIF_LINK_CALLBACK: Support a callback function from an interface
 * whenever the link changes (i.e., link down)
 */
#define LWIP_NETIF_LINK_CALLBACK        1

/**
 * Optional: Enable status callbacks as well (for IP address changes, etc.)
 * Uncomment if needed:
 */
/* #define LWIP_NETIF_STATUS_CALLBACK      1 */

/* ========================================================================
 * Include base Xilinx LWIP configuration
 *
 * The BSP provides xlwipconfig.h which contains all the standard Xilinx
 * LWIP settings. It uses #ifndef guards, so our overrides above take
 * precedence.
 * ======================================================================== */
#include "xlwipconfig.h"

#endif /* __LWIPOPTS_H__ */
