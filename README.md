# IEEE 1588 PTPD FOR EMBEDDED MICROCONTROLLERS STM32H7

IEEE 1588 PTP daemon for STM32H7 v2 supporting FreeRTOS, lwip, and hardware timestamping. The software is tested to work with LWIP 2.x and FreeRTOS 10.3.1. The operating system interface is utilizing CMSIS-RTOS version 1.02.

## Usage
1. Make sure lwip is working properly and is stalbe with FreeRTOS.
2. Enable IGMP in LWIP and make sure it is working. Typically this is done by defining LWIP_IGMP to 1 in _lwipopts.h_. You typically need to enable IGMP in your network interface as well (for example netif->flags |= NETIF_FLAG_IGMP).
3. Go through the files in helper_files folder and compare them with the files in your project.
4. In the software initilization call function ptpd_init().
