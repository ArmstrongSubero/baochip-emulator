/*
 * main.c: software reset test for baoemu
 *
 * Copyright (c) 2026 Armstrong Subero
 * SPDX-License-Identifier: Apache-2.0
 */

#include "bao.h"
#include "bao_reset.h"

int main(void)
{
    bao_init();
    mini_printf("sw_reset boot, resetting in 5 ms\r\n");
    delay_ms(5);
    bao_software_reset();
    return 0;
}
