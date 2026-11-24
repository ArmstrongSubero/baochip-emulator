/*
 * main.c: GPIO interrupt test for baoemu, PB3 falling edge on IOX channel 0
 *
 * Copyright (c) 2026 Armstrong Subero
 * SPDX-License-Identifier: Apache-2.0
 */

#include "bao.h"
#include "hardware/irq.h"

static volatile uint32_t hits;

static void on_iox(uint32_t ev)
{
    (void)ev;
    REG32(0x5012F120) = 0x80;
    hits++;
}

int main(void)
{
    bao_init();
    gpio_init(GPIO_PORT_B, 3);
    gpio_set_dir(GPIO_PORT_B, 3, false);
    gpio_pull_up(GPIO_PORT_B, 3);
    REG32(0x5012F100) = (1u << 9) | (1u << 7) | (16 + 3);
    irq_init();
    irq_set_handler(10, on_iox);
    irq_enable_events(10, 1);
    uint32_t last = 0;
    while (1) {
        if (hits != last) {
            last = hits;
            mini_printf("falling edges: %u\r\n", last);
        }
    }
}
