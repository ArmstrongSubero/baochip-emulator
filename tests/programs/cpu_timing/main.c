/*
 * main.c: VexRiscv instruction timing for baoemu
 *
 * Copyright (c) 2026 Armstrong Subero
 * SPDX-License-Identifier: Apache-2.0
 */

#include "bao.h"

/* each block is timed with mcycle, t3 holds an uncached address */
#define BLOCK(fn, body)                                                        \
    static uint32_t fn(void)                                                   \
    {                                                                          \
        uint32_t d, n;                                                         \
        for (n = 0; n < 2; n++)                                                \
            __asm__ volatile (                                                 \
                ".option push\n"                                               \
                ".option norvc\n"                                              \
                "li t3, 0xE000F010\n"                                          \
                "li t4, 7\n"                                                   \
                "li t5, 3\n"                                                   \
                ".option rvc\n"                                                \
                ".balign 4\n"                                                  \
                ".option norvc\n"                                              \
                "csrr t0, mcycle\n"                                            \
                body                                                           \
                ".option norvc\n"                                              \
                "nop\n nop\n nop\n nop\n"                                      \
                "csrr t1, mcycle\n"                                            \
                "sub %0, t1, t0\n"                                             \
                ".option pop\n"                                                \
                : "=r"(d) : : "t0", "t1", "t2", "t3", "t4", "t5", "t6", "memory"); \
        return d;                                                              \
    }

BLOCK(t_empty, "")
BLOCK(t_alu, "add t2, t4, t5\n add t2, t2, t5\n add t2, t2, t5\n add t2, t2, t5\n"
             "add t2, t2, t5\n add t2, t2, t5\n add t2, t2, t5\n add t2, t2, t5\n")
BLOCK(t_load_use, "lw t2, 0(sp)\n addi t6, t2, 1\n")
BLOCK(t_load_gap1, "lw t2, 0(sp)\n nop\n addi t6, t2, 1\n")
BLOCK(t_load_gap2, "lw t2, 0(sp)\n nop\n nop\n addi t6, t2, 1\n")
BLOCK(t_load_store, "lw t2, 0(sp)\n sw t2, 0(sp)\n")
BLOCK(t_load_rewritten, "lw t2, 0(sp)\n li t2, 5\n addi t6, t2, 1\n")
BLOCK(t_shift_use, "slli t2, t4, 1\n addi t6, t2, 1\n")
BLOCK(t_shift_free, "slli t2, t4, 1\n addi t6, t4, 1\n")
BLOCK(t_mul_use, "mul t2, t4, t5\n addi t6, t2, 1\n")
BLOCK(t_amo, "amoor.w zero, zero, (sp)\n")
BLOCK(t_div_free, "div t2, t4, t5\n addi t6, t4, 1\n")
BLOCK(t_div_use, "divu t2, t4, t5\n addi t6, t2, 1\n")
BLOCK(t_jal, "j 1f\n 1:\n")
BLOCK(t_fwd_not, "beq zero, sp, 1f\n 1:\n")
BLOCK(t_fwd_taken, "beq zero, zero, 1f\n 1:\n")
BLOCK(t_loop, "li t2, 4\n 1: addi t2, t2, -1\n bnez t2, 1b\n")
BLOCK(t_jalr, "la t2, 1f\n jr t2\n 1:\n")
BLOCK(t_csr, "csrr t2, mscratch\n")
BLOCK(t_csr_use, "csrr t2, mscratch\n addi t6, t2, 1\n")
BLOCK(t_csr_gap, "nop\n nop\n nop\n csrr t2, mscratch\n")
BLOCK(t_io_load, "lw t2, 0(t3)\n")
BLOCK(t_io_use, "lw t2, 0(t3)\n addi t6, t2, 1\n")
BLOCK(t_split, ".option rvc\n c.j 1f\n .option norvc\n 1: addi t2, t2, 1\n")
BLOCK(t_nosplit, ".option rvc\n c.j 1f\n 1: c.addi t2, 1\n")
BLOCK(t_sfence, "sfence.vma\n")
BLOCK(t_fence_i, "fence.i\n")
BLOCK(t_dflush_line, ".word 0x0001500F\n")
BLOCK(t_dflush, ".word 0x0000500F\n")
BLOCK(t_dflush_store, "sw t2, 0(sp)\n .word 0x0000500F\n")

BLOCK(t_ecall, "ecall\n")

/* trap handler for t_ecall: step over the ecall and return */
extern char timing_trap[];
__asm__(
"   .section .text.timing_trap, \"ax\"\n"
"   .option push\n"
"   .option norvc\n"
"   .balign 4\n"
"   .globl timing_trap\n"
"timing_trap:\n"
"   csrr t6, mepc\n"
"   addi t6, t6, 4\n"
"   csrw mepc, t6\n"
"   mret\n"
"   .option pop\n"
"   .text\n"
);

static int passed, total;

static void report(const char *name, uint32_t got, uint32_t base, uint32_t want)
{
    uint32_t c = got - base;
    total++;
    if (c == want) {
        passed++;
        mini_printf("%s: %d cycles PASS\r\n", name, (int)c);
    } else {
        mini_printf("%s: %d cycles FAIL want %d\r\n", name, (int)c, (int)want);
    }
}

int main(void)
{
    uint32_t base;

    bao_init();
    mini_printf("\r\ncpu_timing: VexRiscv instruction timing\r\n");
    base = t_empty();
    mini_printf("empty block: %d cycles\r\n", (int)base);

    report("8 dependent adds", t_alu(), base, 8);
    report("load then use", t_load_use(), base, 4);
    report("load, one between, use", t_load_gap1(), base, 4);
    report("load, two between, use", t_load_gap2(), base, 4);
    report("load then store of it", t_load_store(), base, 4);
    report("load, li same rd, use", t_load_rewritten(), base, 4);
    report("shift then use", t_shift_use(), base, 3);
    report("shift then other", t_shift_free(), base, 2);
    report("mul then use", t_mul_use(), base, 4);
    report("amoor.w", t_amo(), base, 2);
    report("div then other", t_div_free(), base, 35);
    report("divu then use", t_div_use(), base, 36);
    report("jal", t_jal(), base, 2);
    report("forward branch not taken", t_fwd_not(), base, 1);
    report("forward branch taken", t_fwd_taken(), base, 4);
    report("loop of 4", t_loop(), base, 15);
    report("la and jr", t_jalr(), base, 6);
    report("csrr", t_csr(), base, 3);
    report("csrr then use", t_csr_use(), base, 5);
    report("three nops then csrr", t_csr_gap(), base, 6);
    report("uncached load", t_io_load(), base, 4);
    report("uncached load then use", t_io_use(), base, 7);
    report("c.j to a split 32 bit addi", t_split(), base, 4);
    report("c.j to c.addi", t_nosplit(), base, 3);
    report("sfence.vma", t_sfence(), base, 5);
    report("fence.i", t_fence_i(), base, 133);
    report("data cache flush of one line", t_dflush_line(), base, 4);
    report("data cache flush", t_dflush(), base, 131);
    report("store then data cache flush", t_dflush_store(), base, 134);
    {
        uint32_t saved, d;
        __asm__ volatile ("csrr %0, mtvec" : "=r"(saved));
        __asm__ volatile ("csrw mtvec, %0" :: "r"(timing_trap));
        d = t_ecall();
        __asm__ volatile ("csrw mtvec, %0" :: "r"(saved));
        report("ecall, handler, mret", d, base, 17);
    }

    mini_printf("[Done] cpu_timing %d of %d match\r\n", passed, total);
    for (;;) {
    }
}
