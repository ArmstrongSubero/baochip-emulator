/*
 * main.c: UART2 echo test for baoemu --interactive
 *
 * Copyright (c) 2026 Armstrong Subero
 * SPDX-License-Identifier: Apache-2.0
 */

#include "bao.h"

int main(void)
{
    bao_init();
    mini_printf("echo ready\r\n");
    while (1) {
        if (uart_is_readable(2)) {
            char c = uart_getc(2);
            mini_printf("[%c]", c);
        }
    }
}
