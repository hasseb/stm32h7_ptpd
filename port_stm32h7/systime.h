/*
 * systime.h
 *
 *  Created on: Mar 13, 2025
 *      Author: hasseb
 */

#ifndef PTPD_SRC_SYSTIME_H_
#define PTPD_SRC_SYSTIME_H_
#include "cmsis_os.h"
#include <stdio.h>

static inline void systime_str(char* buffer, size_t size){snprintf(buffer, size, "%u ticks", (unsigned int)osKernelSysTick());}

#endif /* PTPD_SRC_SYSTIME_H_ */
