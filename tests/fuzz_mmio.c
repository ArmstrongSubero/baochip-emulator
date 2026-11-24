/*
 * fuzz_mmio.c: random register accesses, instructions and host events
 * against baoemu, for a build with the address and undefined behaviour
 * sanitizers
 *
 * Copyright (c) 2026 Armstrong Subero
 * SPDX-License-Identifier: Apache-2.0
 *
 * Build and run:
 *   gcc -O1 -g -std=gnu99 -fsanitize=address,undefined \
 *       -fno-sanitize-recover=undefined -o build/fuzz_mmio \
 *       src/core.c src/cpu.c src/bus.c src/udma.c src/sce.c src/devices.c \
 *       src/bio.c src/pke.c src/pwm.c src/mdma.c src/mbox.c src/usb.c \
 *       tests/fuzz_mmio.c
 *   build/fuzz_mmio <first seed> <seeds> [steps per seed] [program.elf]
 *
 * With a program given, accesses are slipped in between its instructions.
 * A sanitizer report or a step over 20 s of host time stops the run.
 */

#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "../src/baoemu.h"
#include "../src/bao_int.h"

#define STUB 0x61100000u
#define TRAP 0x61110000u
#define NOP  0x00000013u

static uint64_t rng_state;
static unsigned long cur_seed, cur_step;
static unsigned long n_access, n_done, n_loads_nonzero, n_resets;
static int with_program;
static uint32_t saved_x[32], saved_pc, saved_mstatus, saved_priv;
static int saved_wfi, saved_gate;

static void save_cpu(bao_t *b);
static void restore_cpu(bao_t *b);

static uint32_t rnd(void)
{
    rng_state ^= rng_state << 13;
    rng_state ^= rng_state >> 7;
    rng_state ^= rng_state << 17;
    return (uint32_t)(rng_state >> 16);
}

static void quiet(void *user, int level, const char *msg)
{
    (void)user;
    (void)level;
    (void)msg;
}

static void on_alarm(int sig)
{
    (void)sig;
    fprintf(stderr, "\nHANG: seed %lu step %lu took more than 20 s\n", cur_seed, cur_step);
    _exit(3);
}

static uint32_t pick_addr(void)
{
    static const uint32_t win[][2] = {
        { 0x40000000u, 0x00100000u }, { 0x50100000u, 0x00030000u },
        { 0x50202000u, 0x00003000u }, { 0xE0000000u, 0x00100000u },
        { 0x50000000u, 0x00040000u }, { 0x40020000u, 0x00010000u },
        { 0x50120000u, 0x00010000u }, { 0x60000000u, 0x00400000u },
        { 0x61000000u, 0x00200000u },
    };
    uint32_t r = rnd() % 100, a;
    if (r < 88) {
        uint32_t w = rnd() % 7;
        a = win[w][0] + rnd() % win[w][1];
        if (rnd() % 4)
            a = (a & ~0xFFFu) | (rnd() % 0x100u);
    } else if (r < 94) {
        uint32_t w = 7 + rnd() % 2;
        a = win[w][0] + rnd() % win[w][1];
    } else {
        a = rnd();
    }
    if (rnd() % 10)
        a &= ~3u;
    return a;
}

static uint32_t pick_val(void)
{
    switch (rnd() % 8) {
    case 0: return 0;
    case 1: return 0xFFFFFFFFu;
    case 2: return 1u << (rnd() % 32);
    case 3: return rnd() & 0xFFu;
    case 4: return 0x50000000u + (rnd() % 0x40000u);
    case 5: return 0x61000000u + (rnd() % 0x200000u);
    default: return rnd();
    }
}

static void put32(bao_t *b, uint32_t addr, uint32_t v)
{
    uint8_t w[4] = { (uint8_t)v, (uint8_t)(v >> 8), (uint8_t)(v >> 16), (uint8_t)(v >> 24) };
    bao_mem_write(b, addr, w, 4);
}

static void one_access(bao_t *b)
{
    static const uint32_t ops[6] = {
        0x00B52023u,    /* sw  a1, 0(a0) */
        0x00B51023u,    /* sh  a1, 0(a0) */
        0x00B50023u,    /* sb  a1, 0(a0) */
        0x00052603u,    /* lw  a2, 0(a0) */
        0x00055603u,    /* lhu a2, 0(a0) */
        0x00054603u,    /* lbu a2, 0(a0) */
    };
    save_cpu(b);
    put32(b, STUB, ops[rnd() % 6]);
    bao_set_reg(b, 10, pick_addr());
    bao_set_reg(b, 11, pick_val());
    bao_set_reg(b, 12, 0);
    bao_set_pc(b, STUB);
    bao_run(b, 12);
    n_access++;
    if (bao_get_pc(b) > STUB && bao_get_pc(b) < STUB + 0x100u)
        n_done++;
    if (bao_get_reg(b, 12) != 0)
        n_loads_nonzero++;
    restore_cpu(b);
}

static void save_cpu(bao_t *b)
{
    int i;
    if (!with_program)
        return;
    for (i = 1; i < 32; i++)
        saved_x[i] = bao_get_reg(b, i);
    saved_pc = bao_get_pc(b);
    saved_mstatus = b->mstatus;
    saved_priv = b->priv;
    saved_wfi = b->wfi;
    saved_gate = b->wfi_gate;
    /* wake a sleeping core and hold interrupts off for the one access */
    if (b->wfi)
        cpu_wfi_leave(b);
    b->mstatus &= ~0x8u;
    b->priv = 3;
}

/* Put the program back where it was, unless the access trapped into it */
static void restore_cpu(bao_t *b)
{
    int i;
    uint32_t pc = bao_get_pc(b);
    if (!with_program || pc < STUB || pc >= STUB + 0x400u)
        return;
    for (i = 1; i < 32; i++)
        bao_set_reg(b, i, saved_x[i]);
    bao_set_pc(b, saved_pc);
    b->mstatus = saved_mstatus;
    b->priv = saved_priv;
    if (saved_wfi) {
        b->wfi = 1;
        b->wfi_since = b->cycles;
        b->wfi_gate = saved_gate;
    }
}

static void instructions(bao_t *b)
{
    int n = 1 + (int)(rnd() % 24), i;
    for (i = 0; i < n; i++) {
        uint32_t w = rnd();
        if (rnd() % 3 == 0)
            w = (w & ~0x7Fu) | (uint32_t[]){ 0x03, 0x13, 0x23, 0x33, 0x63, 0x73, 0x2F, 0x0F }[rnd() % 8];
        put32(b, STUB + 0x100u + 4u * (uint32_t)i, w);
    }
    for (i = n; i < n + 8; i++)
        put32(b, STUB + 0x100u + 4u * (uint32_t)i, NOP);
    for (i = 1; i < 32; i++)
        if (rnd() % 4 == 0)
            bao_set_reg(b, i, rnd() % 2 ? pick_addr() : pick_val());
    bao_set_pc(b, STUB + 0x100u + (rnd() % 8 ? 0 : 2));
    bao_run(b, 40 + rnd() % 200);
}

/* nop runs after the stub and at the trap vector keep the chip running; a stopped chip is reset */
static void setup(bao_t *b)
{
    uint32_t i;
    bao_on_log(b, quiet, NULL);
    bao_set_guard(b, 0);
    bao_set_stop_on_exception(b, 0);
    for (i = 0; i < 0x400u; i += 4) {
        put32(b, STUB + i, NOP);
        put32(b, TRAP + i, NOP);
    }
    if (with_program) {
        /* boot the program and let it set its devices up */
        bao_run(b, 20000000u + rnd() % 200000000u);
        return;
    }
    put32(b, STUB, 0x30551073u);    /* csrw mtvec, a0 */
    bao_set_reg(b, 10, TRAP);
    bao_set_pc(b, STUB);
    bao_run(b, 8);
}

static void recover(bao_t *b)
{
    int why = bao_stop_reason(b);
    if (why != BAO_RUNNING && why != BAO_STOP_LIMIT && why != BAO_STOP_USER &&
        why != BAO_STOP_IDLE) {
        n_resets++;
        bao_reset(b);
        setup(b);
    }
}

static void host_event(bao_t *b)
{
    uint8_t buf[600];
    uint32_t i, n;
    switch (rnd() % 12) {
    case 0:
        n = 1 + rnd() % 40;
        for (i = 0; i < n; i++) buf[i] = (uint8_t)rnd();
        bao_uart_rx(b, (int)(rnd() % 4), buf, (int)n);
        break;
    case 1: bao_gpio_set_input(b, (int)(rnd() % 6), (int)(rnd() % 16), (int)(rnd() % 2)); break;
    case 2: bao_adc_set_mv(b, (int)(rnd() % 5), rnd() % 4000); break;
    case 3: bao_usb_attach(b, (int)(rnd() % 2)); break;
    case 4:
        n = 1 + rnd() % sizeof(buf);
        for (i = 0; i < n; i++) buf[i] = (uint8_t)rnd();
        bao_usb_serial_rx(b, buf, (int)n);
        break;
    case 5: bao_usb_disk_read(b, rnd() % 300, 1 + rnd() % 8); break;
    case 6:
        memset(buf, (int)rnd(), 512);
        bao_usb_disk_write(b, rnd() % 300, 1, buf);
        break;
    case 7: bao_touch(b, (int)(rnd() % 2), (int)(rnd() % 2), (int)(rnd() % 500), (int)(rnd() % 500)); break;
    case 8: bao_mpu6050_set(b, (int16_t)rnd(), (int16_t)rnd(), (int16_t)rnd(), (int16_t)rnd(),
                            (int16_t)rnd(), (int16_t)rnd()); break;
    case 9: {
        uint32_t p, h;
        bao_pwm_get(b, (int)(rnd() % 5), (int)(rnd() % 5), &p, &h);
        bao_usb_disk_data(b, buf, rnd() % sizeof(buf));
        bao_usb_serial_open(b);
        bao_millis(b);
        break;
    }
    case 10:
        if (rnd() % 8 == 0) {
            bao_reset(b);
            setup(b);
        }
        break;
    default: bao_set_fast_forward(b, (int)(rnd() % 2)); bao_set_uart_instant(b, (int)(rnd() % 2)); break;
    }
}

int main(int argc, char **argv)
{
    unsigned long first = argc > 1 ? strtoul(argv[1], NULL, 0) : 1;
    unsigned long seeds = argc > 2 ? strtoul(argv[2], NULL, 0) : 10;
    unsigned long steps = argc > 3 ? strtoul(argv[3], NULL, 0) : 20000;
    const char *elf = argc > 4 ? argv[4] : NULL;
    unsigned long s, k;

    signal(SIGALRM, on_alarm);
    for (s = first; s < first + seeds; s++) {
        bao_t *b = bao_new();
        static const char *const parts[] = { "w25q", "mpu6050", "st7796", "ft6336" };
        cur_seed = s;
        rng_state = 0x9E3779B97F4A7C15ull * (s + 1) + 0x1234567ull;
        with_program = elf != NULL;
        if (elf && bao_load_file(b, elf) != 0) {
            fprintf(stderr, "cannot load %s\n", elf);
            return 2;
        }
        setup(b);
        if (rnd() % 2)
            bao_attach(b, parts[rnd() % 4], (int)(rnd() % 4), (int)(rnd() % 4), NULL);
        if (rnd() % 2)
            bao_usb_attach(b, 1);
        for (k = 0; k < steps; k++) {
            uint32_t r = rnd() % 100;
            cur_step = k;
            alarm(20);
            if (r < 70) one_access(b);
            else if (r < 82 && !with_program) instructions(b);
            else if (r < 92) bao_run(b, 1 + rnd() % (rnd() % 8 ? 2000u : 2000000u));
            else host_event(b);
            recover(b);
            if (elf) {
                bao_run(b, 1 + rnd() % (rnd() % 16 ? 3000u : 3000000u));
                recover(b);
            }
        }
        alarm(0);
        bao_free(b);
        printf("seed %lu ok\n", s);
        fflush(stdout);
    }
    printf("accesses %lu, completed %lu, loads that read non zero %lu, resets after a stop %lu\n",
           n_access, n_done, n_loads_nonzero, n_resets);
    printf("RESULT: PASS (%lu seeds, %lu steps each)\n", seeds, steps);
    return 0;
}
