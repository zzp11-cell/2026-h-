#ifndef __BSP_PRINTF_H
#define __BSP_PRINTF_H

#include "ti_msp_dl_config.h"

//任意串口printf
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

void any_printf(UART_Regs *uart,char *format,...);

#endif
