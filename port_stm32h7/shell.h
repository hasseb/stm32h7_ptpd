/*
 * shell.h
 *
 *  Created on: Mar 13, 2025
 *      Author: hasseb
 */

#ifndef PTPD_SRC_SHELL_H_
#define PTPD_SRC_SHELL_H_

#include "user_functions.h"

#define shell_printf debugPrintf
#define shell_puts debugPrintf

static inline void shell_add_command(const char*, void*){};

#endif /* PTPD_SRC_SHELL_H_ */
