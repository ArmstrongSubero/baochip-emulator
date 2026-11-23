/*
 * main.c: register readback test for baoemu, IOX, UDMA and PWM
 *
 * Copyright (c) 2026 Armstrong Subero
 * SPDX-License-Identifier: Apache-2.0
 */

#include "bao.h"

#define IOX   0x5012F000u
#define UDMA  0x50100000u
#define UART3 0x50104000u
#define SPIM0 0x50105000u
#define I2C0  0x50109000u
#define PWM   0x50120000u

static int pass, total;

static void check(const char *what, uint32_t got, uint32_t want)
{
    total++;
    if (got == want)
        pass++;
    mini_printf("%s: 0x%08x %s\r\n", what, got, got == want ? "PASS" : "FAIL");
}

static uint32_t ones(uint32_t addr)
{
    uint32_t old = REG32(addr), v;
    REG32(addr) = 0xFFFFFFFFu;
    v = REG32(addr);
    REG32(addr) = old;
    return v;
}

static void iox(void)
{
    uint32_t old;

    check("IOX AFSEL port F 16 bits", ones(IOX + 0x028), 0xFFFF);
    check("IOX GPIOOUT port F 16 bits", ones(IOX + 0x144), 0xFFFF);
    check("IOX DRVSEL port F 32 bits", ones(IOX + 0x274), 0xFFFFFFFFu);
    check("IOX PU port F reset", REG32(IOX + 0x174), 0xFFFF);
    check("IOX undecoded 0x030", ones(IOX + 0x030), 0);
    check("IOX undecoded 0x190", ones(IOX + 0x190), 0);

    /* channel 7 with every bit but inten, so it never fires */
    old = REG32(IOX + 0x11C);
    REG32(IOX + 0x11C) = 0xFFFFFDFFu;
    check("IOX INTCR 11 bits", REG32(IOX + 0x11C), 0x5FF);
    REG32(IOX + 0x11C) = old;

    old = REG32(IOX + 0x18C);
    REG32(IOX + 0x18C) = ~old;
    check("IOX GPIOIN port F read only", REG32(IOX + 0x18C), old);
}

static void udma(void)
{
    uint32_t cg = REG32(UDMA + 0x00), v;

    REG32(UDMA + 0x00) = cg | 0xFFF00000u;
    v = REG32(UDMA + 0x00);
    REG32(UDMA + 0x00) = cg;
    check("UDMA CG 20 bits", v, cg & 0xFFFFF);
    check("UDMA CFG_EVT 32 bits", ones(UDMA + 0x04), 0xFFFFFFFFu);
    check("UDMA undecoded 0x0C", ones(UDMA + 0x0C), 0);

    /* with its CG bit clear the UART3 reg_if has no clock: writes drop */
    REG32(UDMA + 0x00) = cg & ~(1u << 3);
    REG32(UART3 + 0x2C) = 3;
    check("UART3 IRQ_EN write with CG off", REG32(UART3 + 0x2C), 0);

    /* clocks on for UART3 (bit 3), SPIM0 (4) and I2C0 (8): their reg_if
     * runs on the gated clock */
    REG32(UDMA + 0x00) = cg | (1u << 3) | (1u << 4) | (1u << 8);

    check("UART3 IRQ_EN 2 bits", ones(UART3 + 0x2C), 3);
    check("UART3 RX INTCFG undecoded", ones(UART3 + 0x0C), 0);
    check("UART3 undecoded 0x38", ones(UART3 + 0x38), 0);

    check("SPIM0 CMD CFG reads words", REG32(SPIM0 + 0x28), 0x4);
    check("SPIM0 RX CFG reset datasize", REG32(SPIM0 + 0x08), 0x4);
    check("SPIM0 TX CFG reads 0 datasize", REG32(SPIM0 + 0x18), 0);

    REG32(I2C0 + 0x34) = 1;
    check("I2C0 SETUP not decoded", REG32(I2C0 + 0x34), 0);
    REG32(I2C0 + 0x34) = 0;
    check("I2C0 CMD CFG reads words", REG32(I2C0 + 0x28), 0x4);
    check("I2C0 ACK idle", REG32(I2C0 + 0x38), 0);
    REG32(UDMA + 0x00) = cg;
}

static void pwm(void)
{
    uint32_t en = REG32(PWM + 0x104), a, b;

    check("PWM slice 3 CFG", ones(PWM + 0x0C4), 0x00FF1FFFu);
    check("PWM slice 3 CH0 TH", ones(PWM + 0x0CC), 0x7FFFF);
    check("PWM slice 3 CH0 LUT", ones(PWM + 0x0DC), 0x3FFFF);
    check("PWM slice 3 CMD reads 0", REG32(PWM + 0x0C0), 0);
    check("PWM PREFD3 10 bits", ones(PWM + 0x14C), 0x3FF);
    check("PWM undecoded 0x0F0", ones(PWM + 0x0F0), 0);

    REG32(PWM + 0x0C8) = 0xFFFF0000u;
    REG32(PWM + 0x0C4) = 1u << 12;
    REG32(PWM + 0x104) = en & ~8u;
    REG32(PWM + 0x0C0) = 1;
    delay_us(50);
    REG32(PWM + 0x104) = en | 8u;
    a = REG32(PWM + 0x0EC);
    delay_us(50);
    b = REG32(PWM + 0x0EC);
    check("PWM start while gated is lost", b - a, 0);

    REG32(PWM + 0x0C0) = 1;
    delay_us(20);
    REG32(PWM + 0x104) = en & ~8u;
    a = REG32(PWM + 0x0EC);
    delay_us(50);
    b = REG32(PWM + 0x0EC);
    total++;
    if (a != 0 && a == b)
        pass++;
    mini_printf("PWM count holds while gated: %s\r\n", a != 0 && a == b ? "PASS" : "FAIL");

    REG32(PWM + 0x104) = en | 8u;
    delay_us(20);
    b = REG32(PWM + 0x0EC);
    total++;
    if (b > a)
        pass++;
    mini_printf("PWM count goes on after: %s\r\n", b > a ? "PASS" : "FAIL");

    REG32(PWM + 0x0C0) = 2;
    REG32(PWM + 0x0C0) = 8;
    REG32(PWM + 0x104) = en;
}

int main(void)
{
    bao_init();
    mini_printf("periph_regs\r\n");
    iox();
    udma();
    pwm();
    mini_printf("[Done] periph_regs %d of %d passed\r\n", pass, total);
    while (1) {
    }
}
