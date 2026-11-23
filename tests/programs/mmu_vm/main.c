/*
 * main.c: Sv32 translation, page faults, SUM, MXR, MPRV and sfence.vma
 *
 * Copyright (c) 2026 Armstrong Subero
 * SPDX-License-Identifier: Apache-2.0
 */

#include "bao.h"
#include "hardware/timer.h"
#include "hardware/regs/timer.h"

#define U 0u
#define S 1u

#define CSRR(c) ({ uint32_t v_; __asm__ volatile ("csrr %0, " #c : "=r"(v_)); v_; })
#define CSRW(c, v) __asm__ volatile ("csrw " #c ", %0" :: "r"((uint32_t)(v)) : "memory")

#define PV 0x01u
#define PR 0x02u
#define PW 0x04u
#define PX 0x08u
#define PU 0x10u
#define PA 0x40u
#define PD 0x80u
#define PTE(pa, f) ((((uint32_t)(pa) >> 12) << 10) | (f))

#define VA_PAGES  0x10000000u
#define VA_MISSUP 0x10400000u
#define VA_SUPER  0x10800000u
#define VA_L1WNR  0x10C00000u
#define VA_UCODE  0x20000000u
#define VA_NONE   0x30000000u
#define PG(n)     (VA_PAGES + 0x1000u * (n))

/*
 * run_lower(mode, entry, a0, a1) drops to U (0) or S (1) and returns a0.
 * Record words: 0 cause, 1 tval, 2 epc, 3 status, 4 exception count,
 * 5 to 7 interrupt cause, epc and count, 8 to 15 first four exceptions.
 */
volatile uint32_t mrec[20], srec[20];
uint32_t mctx[16];
uint8_t m_stack[512] __attribute__((aligned(16)));
uint8_t s_stack[512] __attribute__((aligned(16)));

extern uint32_t run_lower(uint32_t mode, uint32_t entry, uint32_t a0, uint32_t a1);
extern char m_trap[], s_trap[];

__asm__(
"   .section .text.lowerharness, \"ax\"\n"
"   .option push\n"
"   .option norvc\n"
"   .globl run_lower\n"
"   .balign 4\n"
"run_lower:\n"
"   la t0, mctx\n"
"   sw sp, 0(t0)\n  sw ra, 4(t0)\n  sw s0, 8(t0)\n  sw s1, 12(t0)\n"
"   sw s2, 16(t0)\n sw s3, 20(t0)\n sw s4, 24(t0)\n sw s5, 28(t0)\n"
"   sw s6, 32(t0)\n sw s7, 36(t0)\n sw s8, 40(t0)\n sw s9, 44(t0)\n"
"   sw s10, 48(t0)\n sw s11, 52(t0)\n sw gp, 56(t0)\n sw tp, 60(t0)\n"
"   la t1, m_stack + 512\n"
"   csrw mscratch, t1\n"
"   la t1, s_stack + 512\n"
"   csrw sscratch, t1\n"
"   csrw mepc, a1\n"
"   li t1, 0x1800\n"
"   csrc mstatus, t1\n"
"   slli t1, a0, 11\n"
"   csrs mstatus, t1\n"
"   mv a0, a2\n"
"   mv a1, a3\n"
"   mret\n"
"   .globl m_trap\n"
"   .balign 64\n"
"m_trap:\n"
"   csrrw sp, mscratch, sp\n"
"   addi sp, sp, -16\n"
"   sw t0, 0(sp)\n  sw t1, 4(sp)\n  sw a0, 8(sp)\n"
"   la t0, mrec\n"
"   csrr t1, mcause\n"
"   bltz t1, 3f\n"
"   sw t1, 0(t0)\n"
"   lw a0, 16(t0)\n  sltiu t1, a0, 4\n  beqz t1, 5f\n"
"   slli a0, a0, 3\n  add a0, a0, t0\n"
"   csrr t1, mcause\n  sw t1, 32(a0)\n"
"   csrr t1, mtval\n  sw t1, 36(a0)\n"
"5: lw a0, 8(sp)\n"
"   csrr t1, mtval\n  sw t1, 4(t0)\n"
"   csrr t1, mepc\n   sw t1, 8(t0)\n"
"   csrr t1, mstatus\n sw t1, 12(t0)\n"
"   lw t1, 16(t0)\n   addi t1, t1, 1\n  sw t1, 16(t0)\n"
"   csrr t1, mcause\n"
"   li t0, 8\n  beq t1, t0, 4f\n"
"   li t0, 9\n  beq t1, t0, 4f\n"
"   li t0, 12\n beq t1, t0, 4f\n"
"   li t0, 1\n  beq t1, t0, 4f\n"
"   csrr t0, mepc\n  addi t0, t0, 4\n  csrw mepc, t0\n"
"2: lw t0, 0(sp)\n  lw t1, 4(sp)\n  lw a0, 8(sp)\n"
"   addi sp, sp, 16\n"
"   csrrw sp, mscratch, sp\n"
"   mret\n"
"3: sw t1, 20(t0)\n"
"   csrr t1, mepc\n   sw t1, 24(t0)\n"
"   lw t1, 28(t0)\n   addi t1, t1, 1\n  sw t1, 28(t0)\n"
"   csrr t1, 0xFC0\n  sw t1, 64(t0)\n"
"   csrr t1, mtval\n  sw t1, 68(t0)\n"
"   lw t1, 72(t0)\n  beqz t1, 6f\n  sw t1, 0(t1)\n"
"6: csrw mip, zero\n"
"   csrw 0xBC0, zero\n"
"   csrw 0x9C0, zero\n"
"   j 2b\n"
"4: lw a0, 8(sp)\n"
"   la t0, mctx\n"
"   lw sp, 0(t0)\n  lw ra, 4(t0)\n  lw s0, 8(t0)\n  lw s1, 12(t0)\n"
"   lw s2, 16(t0)\n lw s3, 20(t0)\n lw s4, 24(t0)\n lw s5, 28(t0)\n"
"   lw s6, 32(t0)\n lw s7, 36(t0)\n lw s8, 40(t0)\n lw s9, 44(t0)\n"
"   lw s10, 48(t0)\n lw s11, 52(t0)\n lw gp, 56(t0)\n lw tp, 60(t0)\n"
"   ret\n"
"   .globl s_trap\n"
"   .balign 64\n"
"s_trap:\n"
"   csrrw sp, sscratch, sp\n"
"   addi sp, sp, -16\n"
"   sw t0, 0(sp)\n  sw t1, 4(sp)\n  sw a0, 8(sp)\n"
"   la t0, srec\n"
"   csrr t1, scause\n"
"   bltz t1, 3f\n"
"   sw t1, 0(t0)\n"
"   lw a0, 16(t0)\n  sltiu t1, a0, 4\n  beqz t1, 5f\n"
"   slli a0, a0, 3\n  add a0, a0, t0\n"
"   csrr t1, scause\n  sw t1, 32(a0)\n"
"   csrr t1, stval\n  sw t1, 36(a0)\n"
"5: lw a0, 8(sp)\n"
"   csrr t1, stval\n  sw t1, 4(t0)\n"
"   csrr t1, sepc\n   sw t1, 8(t0)\n"
"   csrr t1, sstatus\n sw t1, 12(t0)\n"
"   lw t1, 16(t0)\n   addi t1, t1, 1\n  sw t1, 16(t0)\n"
"   csrr t1, scause\n"
"   li t0, 8\n  beq t1, t0, 4f\n"
"   li t0, 12\n beq t1, t0, 4f\n"
"   li t0, 1\n  beq t1, t0, 4f\n"
"   csrr t0, sepc\n  addi t0, t0, 4\n  csrw sepc, t0\n"
"2: lw t0, 0(sp)\n  lw t1, 4(sp)\n  lw a0, 8(sp)\n"
"   addi sp, sp, 16\n"
"   csrrw sp, sscratch, sp\n"
"   sret\n"
"3: sw t1, 20(t0)\n"
"   csrr t1, sepc\n   sw t1, 24(t0)\n"
"   lw t1, 28(t0)\n   addi t1, t1, 1\n  sw t1, 28(t0)\n"
"   csrr t1, 0xDC0\n  sw t1, 64(t0)\n"
"   csrr t1, stval\n  sw t1, 68(t0)\n"
"   csrw sip, zero\n"
"   csrw 0x9C0, zero\n"
"   j 2b\n"
"4: lw t0, 0(sp)\n  lw t1, 4(sp)\n  lw a0, 8(sp)\n"
"   addi sp, sp, 16\n"
"   csrrw sp, sscratch, sp\n"
"   ecall\n"
"   .option pop\n"
"   .text\n"
);

#define HC(r, i) ((r)[8 + 2 * (i)])
#define HT(r, i) ((r)[9 + 2 * (i)])

static uint32_t pt_root[1024] __attribute__((aligned(4096)));
static uint32_t pt_l0[1024] __attribute__((aligned(4096)));
static uint32_t pages[8][1024] __attribute__((aligned(4096)));
volatile uint32_t fixval;
static volatile uint32_t marker = 0x5A5A1234u;

extern char sn_lw[], sn_sw[], sn_jr[], sn_amo[], sn_lr[], sn_sc[], sn_lw_mis[];
extern char sn_sum_lw[], sn_mxr_lw[], sn_sum_jr[], sn_ecall2[], sn_stale[], sn_satp_flush[];
extern char sn_drop[], sn_lr_other[], sn_lr_trap_sc[], sn_lr_sw_sc[], sn_poll[], sn_wfi[];
extern char sn_after_wfi[];

__asm__(
"   .section .text.snippets, \"ax\"\n"
"   .option push\n"
"   .option norvc\n"
"   .balign 4\n"
"   .globl sn_lw\n sn_lw: lw a0, 0(a0)\n ecall\n"
"   .globl sn_sw\n sn_sw: sw a1, 0(a0)\n ecall\n"
"   .globl sn_jr\n sn_jr: jr a0\n"
"   .globl sn_amo\n sn_amo: amoadd.w a0, a1, (a0)\n ecall\n"
"   .globl sn_lr\n sn_lr: lr.w a0, (a0)\n ecall\n"
"   .globl sn_sc\n sn_sc: li t0, 7\n sc.w t0, a1, (a0)\n mv a0, t0\n ecall\n"
"   .globl sn_lw_mis\n sn_lw_mis: lw a0, 1(a0)\n ecall\n"
"   .globl sn_sum_lw\n sn_sum_lw: li t0, 0x40000\n csrs sstatus, t0\n lw a0, 0(a0)\n csrc sstatus, t0\n ecall\n"
"   .globl sn_mxr_lw\n sn_mxr_lw: li t0, 0x80000\n csrs sstatus, t0\n lw a0, 0(a0)\n csrc sstatus, t0\n ecall\n"
"   .globl sn_sum_jr\n sn_sum_jr: li t0, 0x40000\n csrs sstatus, t0\n jr a0\n"
"   .globl sn_ecall2\n sn_ecall2: ecall\n"
/* a0 = VA of the read only page, a1 = its PTE through the identity map */
"   .globl sn_stale\n"
"sn_stale:\n"
"   li t0, 0xAB\n"
"   sw t0, 0(a0)\n"
"   lw t1, 0(a1)\n"
"   ori t1, t1, 4\n"
"   sw t1, 0(a1)\n"
"   sw t0, 0(a0)\n"
"   sfence.vma\n"
"   sw t0, 0(a0)\n"
"   lw a0, 0(a0)\n"
"   ecall\n"
"   .globl sn_satp_flush\n"
"sn_satp_flush:\n"
"   li t0, 0xCD\n"
"   sw t0, 0(a0)\n"
"   lw t1, 0(a1)\n"
"   ori t1, t1, 4\n"
"   sw t1, 0(a1)\n"
"   csrr t1, satp\n"
"   csrw satp, t1\n"
"   sw t0, 0(a0)\n"
"   lw a0, 0(a0)\n"
"   ecall\n"
/* a0 = VA of an invalid page, a1 = its PTE, fixval = the valid PTE */
"   .globl sn_drop\n"
"sn_drop:\n"
"   lw t0, 0(a0)\n"
"   la t2, fixval\n"
"   lw t1, 0(t2)\n"
"   sw t1, 0(a1)\n"
"   lw a0, 0(a0)\n"
"   ecall\n"
"   .globl sn_lr_other\n sn_lr_other: lr.w t0, (a0)\n addi t1, a0, 8\n sc.w a0, a1, (t1)\n ecall\n"
"   .globl sn_lr_trap_sc\n sn_lr_trap_sc: lr.w t0, (a0)\n csrr t1, 0x7C0\n sc.w a0, a1, (a0)\n ecall\n"
"   .globl sn_poll\n sn_poll: lw t0, 0(a0)\n beqz t0, sn_poll\n mv a0, t0\n ecall\n"
"   .globl sn_wfi\n sn_wfi: wfi\n"
"   .globl sn_after_wfi\n sn_after_wfi: ecall\n"
"   .globl sn_lr_sw_sc\n sn_lr_sw_sc: lr.w t0, (a0)\n sw a1, 4(a0)\n sc.w a0, a1, (a0)\n ecall\n"
"   .option pop\n"
"   .text\n"
);

static int passed, total;

static void check(const char *name, int ok, uint32_t got, uint32_t want)
{
    total++;
    if (ok) {
        passed++;
        mini_printf("%s: PASS\r\n", name);
    } else {
        mini_printf("%s: FAIL got %08x want %08x\r\n", name, got, want);
    }
}

static void eq(const char *name, uint32_t got, uint32_t want)
{
    check(name, got == want, got, want);
}

static void clear_recs(void)
{
    int i;
    for (i = 0; i < 20; i++)
        mrec[i] = srec[i] = 0;
}

static uint32_t ualias(const char *f, uint32_t code_mp)
{
    return (uint32_t)f - code_mp + VA_UCODE;
}

static uint32_t mprv_lw(uint32_t mpp, uint32_t va)
{
    uint32_t v = 0xEEEEEEEEu, set = 0x20000u | (mpp << 11);
    __asm__ volatile (
        ".option push\n .option norvc\n"
        "li t0, 0x1800\n"
        "csrc mstatus, t0\n"
        "csrs mstatus, %2\n"
        "lw %0, 0(%1)\n"
        "csrc mstatus, %2\n"
        "csrs mstatus, t0\n"
        ".option pop\n"
        : "+r"(v) : "r"(va), "r"(set) : "t0", "memory");
    return v;
}

static void mprv_sw(uint32_t mpp, uint32_t va, uint32_t val)
{
    uint32_t set = 0x20000u | (mpp << 11);
    __asm__ volatile (
        ".option push\n .option norvc\n"
        "li t0, 0x1800\n"
        "csrc mstatus, t0\n"
        "csrs mstatus, %2\n"
        "sw %1, 0(%0)\n"
        "csrc mstatus, %2\n"
        "csrs mstatus, t0\n"
        ".option pop\n"
        :: "r"(va), "r"(val), "r"(set) : "t0", "memory");
}

/* Timer0 stops while the core sleeps in WFI, the TickTimer does not */
static void tick_alarm(uint64_t tick)
{
    TICKTIMER_EV_ENABLE = 0;
    TICKTIMER_MSLEEP_TARGET1 = (uint32_t)(tick >> 32);
    TICKTIMER_MSLEEP_TARGET0 = (uint32_t)tick;
    TICKTIMER_EV_PENDING = 1;
    __asm__ volatile ("fence" ::: "memory");
    TICKTIMER_EV_ENABLE = 1;
}

int main(void)
{
    uint32_t saved_mtvec, code_mp, sram_mp, r, i;

    bao_init();
    mini_printf("\r\nmmu_vm: Sv32 checks\r\n");

    code_mp = (uint32_t)sn_lw & 0xFFC00000u;
    sram_mp = (uint32_t)pt_root & 0xFFC00000u;
    for (i = 0; i < 1024; i++)
        pt_root[i] = pt_l0[i] = 0;
    for (i = 0; i < 8; i++) {
        uint32_t k;
        for (k = 0; k < 1024; k++)
            pages[i][k] = (i << 24) | k;
    }
    pt_root[code_mp >> 22] = PTE(code_mp, PV | PR | PX | PA);
    pt_root[VA_UCODE >> 22] = PTE(code_mp, PV | PR | PX | PU | PA);
    pt_root[sram_mp >> 22] = PTE(sram_mp, PV | PR | PW | PA | PD);
    pt_root[VA_PAGES >> 22] = PTE(pt_l0, PV);
    pt_root[VA_MISSUP >> 22] = PTE(sram_mp + 0x1000u, PV | PR | PW | PA | PD);
    pt_root[VA_SUPER >> 22] = PTE(sram_mp, PV | PR | PW | PA | PD);
    pt_root[VA_L1WNR >> 22] = PTE(sram_mp, PV | PW);
    pt_l0[0] = PTE(pages[0], PV | PR | PW | PA | PD);
    pt_l0[1] = PTE(pages[1], PV | PR | PA);
    pt_l0[2] = PTE(pages[2], PV | PX | PA);
    pt_l0[3] = 0;
    pt_l0[4] = PTE(pages[4], PV | PW | PA | PD);
    pt_l0[5] = PTE(pages[5], PV | PR | PW | PU | PA | PD);
    pt_l0[6] = PTE(pages[6], PV | PR | PW);
    pt_l0[7] = PTE(pages[7], PR | PW | PX | PA | PD);
    /* code in the execute only page: ecall at 0, and addi a0, a0, 1 split
     * across its last half word and the invalid page after it */
    pages[2][0] = 0x00000073u;
    pages[2][1023] = 0x05130000u;

    saved_mtvec = CSRR(mtvec);
    CSRW(mtvec, (uint32_t)m_trap);
    CSRW(stvec, (uint32_t)s_trap);
    CSRW(medeleg, 0);
    CSRW(satp, 0x80000000u | ((uint32_t)pt_root >> 12));

    /* plain translation */
    clear_recs();
    r = run_lower(S, (uint32_t)sn_lw, PG(0) + 8, 0);
    eq("S load through L0 page", r, 0x00000002u);
    clear_recs();
    run_lower(S, (uint32_t)sn_sw, PG(0) + 12, 0x11112222u);
    eq("S store through L0 page", pages[0][3], 0x11112222u);
    clear_recs();
    r = run_lower(S, (uint32_t)sn_lw, VA_SUPER + ((uint32_t)&marker - sram_mp), 0);
    eq("superpage load", r, 0x5A5A1234u);
    clear_recs();
    r = run_lower(U, ualias(sn_lw, code_mp), PG(5) + 4, 0);
    eq("U code from the U alias, load from U page", r, 0x05000001u);

    /* load and store page faults */
    clear_recs();
    run_lower(S, (uint32_t)sn_sw, PG(1) + 4, 1);
    eq("store to read only: cause", HC(mrec, 0), 15);
    eq("store to read only: tval", HT(mrec, 0), PG(1) + 4);
    eq("store to read only: memory kept", pages[1][1], 0x01000001u);
    clear_recs();
    r = run_lower(S, (uint32_t)sn_lw, PG(1) + 4, 0);
    eq("load from read only", r, 0x01000001u);
    clear_recs();
    run_lower(S, (uint32_t)sn_lw, PG(3) + 0x10, 0);
    eq("load from invalid: cause", HC(mrec, 0), 13);
    eq("load from invalid: tval", HT(mrec, 0), PG(3) + 0x10);
    clear_recs();
    run_lower(S, (uint32_t)sn_sw, PG(3), 0);
    eq("store to invalid", HC(mrec, 0), 15);
    clear_recs();
    run_lower(S, (uint32_t)sn_lw, PG(7), 0);
    eq("V 0 with R W X set", HC(mrec, 0), 13);
    clear_recs();
    run_lower(S, (uint32_t)sn_lw, PG(4), 0);
    eq("load from W without R", HC(mrec, 0), 13);
    clear_recs();
    run_lower(S, (uint32_t)sn_sw, PG(4), 0);
    eq("store to W without R", HC(mrec, 0), 15);
    clear_recs();
    run_lower(S, (uint32_t)sn_lw, VA_L1WNR + 0x20, 0);
    eq("L1 PTE W without R", HC(mrec, 0), 13);
    eq("L1 PTE W without R tval", HT(mrec, 0), VA_L1WNR + 0x20);
    clear_recs();
    run_lower(S, (uint32_t)sn_lw, VA_MISSUP + 0x44, 0);
    eq("misaligned superpage", HC(mrec, 0), 13);
    eq("misaligned superpage tval", HT(mrec, 0), VA_MISSUP + 0x44);
    clear_recs();
    run_lower(S, (uint32_t)sn_sw, VA_NONE + 0x100, 0);
    eq("store to unmapped L1", HC(mrec, 0), 15);
    eq("store to unmapped L1 tval", HT(mrec, 0), VA_NONE + 0x100);
    clear_recs();
    run_lower(S, (uint32_t)sn_lw_mis, PG(3), 0);
    eq("misaligned before page fault", HC(mrec, 0), 4);
    eq("misaligned tval", HT(mrec, 0), PG(3) + 1);

    /* execute only page and MXR */
    clear_recs();
    run_lower(S, (uint32_t)sn_lw, PG(2), 0);
    eq("load from execute only, MXR 0", HC(mrec, 0), 13);
    clear_recs();
    r = run_lower(S, (uint32_t)sn_mxr_lw, PG(2), 0);
    eq("load from execute only, MXR 1", r, 0x00000073u);
    eq("MXR 1 took no fault", mrec[4], 1);
    clear_recs();
    run_lower(S, (uint32_t)sn_sw, PG(2), 0);
    eq("store to execute only", HC(mrec, 0), 15);
    clear_recs();
    run_lower(S, (uint32_t)sn_jr, PG(2), 0);
    eq("S runs code in execute only page", mrec[0], 9);
    eq("its epc", mrec[2], PG(2));

    /* instruction page faults */
    clear_recs();
    run_lower(S, (uint32_t)sn_jr, PG(0) + 0x40, 0);
    eq("fetch from page without X: cause", mrec[0], 12);
    eq("fetch from page without X: tval", mrec[1], PG(0) + 0x40);
    eq("fetch from page without X: epc", mrec[2], PG(0) + 0x40);
    clear_recs();
    run_lower(S, (uint32_t)sn_jr, PG(3) + 0x22, 0);
    eq("fetch at pc+2 of invalid page: cause", mrec[0], 12);
    eq("fetch fault tval is the word address", mrec[1], PG(3) + 0x20);
    eq("fetch fault epc is the pc", mrec[2], PG(3) + 0x22);
    clear_recs();
    r = run_lower(S, (uint32_t)sn_jr, PG(2) + 0xFFE, 5);
    eq("instruction across into invalid page: cause", mrec[0], 12);
    eq("its tval is the second word", mrec[1], PG(3));
    eq("its epc is the instruction", mrec[2], PG(2) + 0xFFE);

    /* U bit and SUM */
    clear_recs();
    run_lower(U, ualias(sn_lw, code_mp), PG(0), 0);
    eq("U load from S page", HC(mrec, 0), 13);
    eq("U load from S page tval", HT(mrec, 0), PG(0));
    clear_recs();
    run_lower(U, ualias(sn_sw, code_mp), PG(5) + 8, 0x7777u);
    eq("U store to U page", pages[5][2], 0x7777u);
    clear_recs();
    run_lower(U, ualias(sn_jr, code_mp), (uint32_t)sn_ecall2, 0);
    eq("U fetch from S page", mrec[0], 12);
    eq("U fetch from S page tval", mrec[1], (uint32_t)sn_ecall2);
    clear_recs();
    run_lower(S, (uint32_t)sn_lw, PG(5), 0);
    eq("S load from U page, SUM 0", HC(mrec, 0), 13);
    clear_recs();
    r = run_lower(S, (uint32_t)sn_sum_lw, PG(5) + 0x10, 0);
    eq("S load from U page, SUM 1", r, 0x05000004u);
    clear_recs();
    run_lower(S, (uint32_t)sn_jr, ualias(sn_ecall2, code_mp), 0);
    eq("S fetch from U page, SUM 0", mrec[0], 12);
    clear_recs();
    run_lower(S, (uint32_t)sn_sum_jr, ualias(sn_ecall2, code_mp), 0);
    eq("S fetch from U page, SUM 1 runs it", mrec[0], 9);
    eq("its epc is the U alias", mrec[2], ualias(sn_ecall2, code_mp));

    /* A and D are not checked or written */
    clear_recs();
    run_lower(S, (uint32_t)sn_sw, PG(6) + 4, 0x6666u);
    eq("store to page with A and D clear", pages[6][1], 0x6666u);
    eq("PTE A and D still clear", pt_l0[6] & (PA | PD), 0);

    /* AMO, LR and SC */
    clear_recs();
    r = run_lower(S, (uint32_t)sn_amo, PG(0) + 0x20, 5);
    eq("amoadd through MMU", (r == 0x00000008u && pages[0][8] == 13u) ? 1 : 0, 1);
    clear_recs();
    run_lower(S, (uint32_t)sn_amo, PG(1), 5);
    eq("amoadd on read only", HC(mrec, 0), 15);
    clear_recs();
    run_lower(S, (uint32_t)sn_amo, PG(2), 5);
    eq("amoadd on execute only", HC(mrec, 0), 15);
    clear_recs();
    run_lower(S, (uint32_t)sn_lr, PG(3), 0);
    eq("lr on invalid", HC(mrec, 0), 13);
    clear_recs();
    run_lower(S, (uint32_t)sn_sc, PG(1), 0);
    eq("sc on read only without reservation", HC(mrec, 0), 15);
    clear_recs();
    r = run_lower(S, (uint32_t)sn_lr_other, PG(0) + 0x40, 0x99u);
    eq("sc to another address after lr succeeds", (r == 0 && pages[0][18] == 0x99u) ? 1 : 0, 1);
    clear_recs();
    r = run_lower(S, (uint32_t)sn_lr_trap_sc, PG(0) + 0x60, 0x98u);
    eq("stores in the trap handler drop the reservation", HC(mrec, 0) == 2 ? r : 9, 1);
    clear_recs();
    r = run_lower(S, (uint32_t)sn_lr_sw_sc, PG(0) + 0x80, 0x97u);
    eq("a store drops the reservation", r, 1);

    /* TLB: permission lines stay until sfence.vma or satp, fault lines go */
    pt_l0[1] = PTE(pages[1], PV | PR | PA);
    clear_recs();
    r = run_lower(S, (uint32_t)sn_stale, PG(1) + 0x30, (uint32_t)&pt_l0[1]);
    eq("stale line: first store faults", HC(mrec, 0), 15);
    eq("stale line: second store faults", HC(mrec, 1), 15);
    eq("after sfence.vma the store lands", r, 0xABu);
    pt_l0[1] = PTE(pages[1], PV | PR | PA);
    __asm__ volatile ("sfence.vma" ::: "memory");
    clear_recs();
    r = run_lower(S, (uint32_t)sn_satp_flush, PG(1) + 0x34, (uint32_t)&pt_l0[1]);
    eq("satp write flushes: store lands", r, 0xCDu);
    eq("satp write flushes: one fault", HC(mrec, 0) == 15 ? mrec[4] : 0, 2);
    pt_l0[1] = PTE(pages[1], PV | PR | PA);
    pt_l0[3] = 0;
    __asm__ volatile ("sfence.vma" ::: "memory");
    fixval = PTE(pages[3], PV | PR | PW | PA | PD);
    clear_recs();
    r = run_lower(S, (uint32_t)sn_drop, PG(3) + 0x14, (uint32_t)&pt_l0[3]);
    eq("faulting walk is not kept", (HC(mrec, 0) == 13 && r == 0x03000005u) ? 1 : 0, 1);
    pt_l0[3] = 0;
    __asm__ volatile ("sfence.vma" ::: "memory");

    /* MPRV from machine mode */
    eq("MPRV with MPP S translates", mprv_lw(1, PG(0) + 4), 0x00000001u);
    eq("MPRV with MPP U skips the U check", mprv_lw(0, PG(0) + 4), 0x00000001u);
    eq("MPRV with MPP M is physical", mprv_lw(3, (uint32_t)&marker), 0x5A5A1234u);
    CSRW(medeleg, (1u << 13) | (1u << 15));
    clear_recs();
    mprv_sw(1, PG(1) + 8, 0x1234u);
    eq("MPRV store fault stays in M", HC(mrec, 0), 15);
    eq("MPRV store fault tval", HT(mrec, 0), PG(1) + 8);
    eq("MPRV store fault S saw nothing", srec[4], 0);

    /* a translated user loop waiting on a machine interrupt, then wfi in S */
    pages[5][100] = 0;
    mrec[18] = (uint32_t)&pages[5][100];
    timer0_start_periodic_ms(2);
    __asm__ volatile ("csrci mstatus, 8" ::: "memory");
    clear_recs();
    mrec[18] = (uint32_t)&pages[5][100];
    r = run_lower(U, ualias(sn_poll, code_mp), PG(5) + 400, 0);
    eq("U loop on a U page saw the timer flag", r, (uint32_t)&pages[5][100]);
    eq("timer interrupt from U", mrec[5], 0x8000000Bu);
    eq("taken inside the loop", (mrec[6] >= ualias(sn_poll, code_mp) && mrec[6] < ualias(sn_poll, code_mp) + 8) ? 1 : 0, 1);
    timer0_stop();
    TIMER0_EV_PENDING = 1;
    tick_alarm(millis() + 2);
    CSRW(0xBC0, 1u << TICKTIMER_IRQ);
    clear_recs();
    run_lower(S, (uint32_t)sn_wfi, 0, 0);
    eq("wfi in S woken by the timer", mrec[5], 0x8000000Bu);
    eq("interrupt epc after the wfi", mrec[6], (uint32_t)sn_after_wfi);
    TICKTIMER_EV_ENABLE = 0;
    TICKTIMER_EV_PENDING = 1;
    CSRW(0xBC0, 0);
    CSRW(mie, 0);
    mrec[18] = 0;

    /* page faults delegated to S */
    CSRW(medeleg, (1u << 12) | (1u << 13) | (1u << 15));
    clear_recs();
    run_lower(U, ualias(sn_lw, code_mp), PG(0) + 0x18, 0);
    eq("U load fault to S: cause", HC(srec, 0), 13);
    eq("U load fault to S: stval", HT(srec, 0), PG(0) + 0x18);
    eq("U load fault to S: sepc", srec[2], ualias(sn_lw, code_mp));
    eq("U load fault to S: M saw only the exit", mrec[0] == 8 && mrec[4] == 1 ? 1 : 0, 1);
    clear_recs();
    run_lower(U, ualias(sn_jr, code_mp), PG(3) + 6, 0);
    eq("U fetch fault to S: cause", HC(srec, 0), 12);
    eq("U fetch fault to S: stval", HT(srec, 0), PG(3) + 4);
    eq("U fetch fault to S: then exit from S", mrec[0], 9);
    clear_recs();
    run_lower(S, (uint32_t)sn_sw, PG(1), 0);
    eq("S store fault to S", HC(srec, 0), 15);
    eq("S store fault SPP", srec[3] & 0x100u, 0x100u);
    CSRW(medeleg, 0);

    CSRW(satp, 0);
    CSRW(mtvec, saved_mtvec);
    mini_printf("[Done] mmu_vm %d of %d passed\r\n", passed, total);
    for (;;) {
    }
}
