/* C++ detection */
#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx.h"
#include <stdio.h>

#pragma import(__use_no_semihosting_swi)

struct __FILE { int handle; };  // Required by Keil

// DO NOT define __stdout / __stdin / __stderr

int fputc(int ch, FILE *f) {
    (void)f;
    ITM_SendChar(ch);  // SWO output
    return ch;
}

void _sys_exit(int return_code) {
    (void)return_code;
    while (1);
}

/* C++ detection */
#ifdef __cplusplus
}
#endif