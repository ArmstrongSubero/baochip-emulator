/*
 * main.c: privilege modes, CSRs, delegation and interrupts for baoemu
 *
 * Copyright (c) 2026 Armstrong Subero
 * SPDX-License-Identifier: Apache-2.0
 */

#include "bao.h"

#define U 0u
#define S 1u

#define CSRR(c) ({ uint32_t v_; __asm__ volatile ("csrr %0, " #c : "=r"(v_)); v_; })
#define CSRW(c, v) __asm__ volatile ("csrw " #c ", %0" :: "r"((uint32_t)(v)) : "memory")

#define IRQ19_SOFT    (*(volatile uint32_t *)0xE000F000u)
#define IRQ19_PENDING (*(volatile uint32_t *)0xE000F010u)
#define IRQ19_ENABLE  (*(volatile uint32_t *)0xE000F014u)

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

volatile uint32_t vals[4];

extern char sn_ecall[], sn_ebreak[], sn_csrr_sstatus[], sn_csrr_cycle[], sn_csrw_cycle[];
extern char sn_csrrs_cycle[], sn_mret[], sn_sret[], sn_csrr_mstatus[], sn_csrr_sim[];
extern char sn_csrw_sip_arr[], sn_csrrs_sip_arr[], sn_sfence[], sn_sret_test[];
extern char sn_illegal[], sn_sie_wait[], sn_nops[], sn_after_csrsi[];

__asm__(
"   .section .text.snippets, \"ax\"\n"
"   .option push\n"
"   .option norvc\n"
"   .balign 4\n"
"   .globl sn_ecall\n sn_ecall: ecall\n"
"   .globl sn_ebreak\n sn_ebreak: ebreak\n ecall\n"
"   .globl sn_csrr_sstatus\n sn_csrr_sstatus: csrr a0, sstatus\n ecall\n"
"   .globl sn_csrr_cycle\n sn_csrr_cycle: li a0, 0\n csrr a0, cycle\n ecall\n"
"   .globl sn_csrw_cycle\n sn_csrw_cycle: csrw cycle, a0\n ecall\n"
"   .globl sn_csrrs_cycle\n sn_csrrs_cycle: csrrs a0, cycle, a1\n ecall\n"
"   .globl sn_mret\n sn_mret: mret\n ecall\n"
"   .globl sn_sret\n sn_sret: sret\n ecall\n"
"   .globl sn_csrr_mstatus\n sn_csrr_mstatus: csrr a0, mstatus\n ecall\n"
"   .globl sn_csrr_sim\n sn_csrr_sim: csrr a0, 0x9C0\n ecall\n"
"   .globl sn_csrw_sip_arr\n sn_csrw_sip_arr: csrw 0xDC0, a0\n ecall\n"
"   .globl sn_csrrs_sip_arr\n sn_csrrs_sip_arr: csrrs a0, 0xDC0, a1\n ecall\n"
"   .globl sn_sfence\n sn_sfence: sfence.vma\n li a0, 1\n ecall\n"
"   .globl sn_illegal\n sn_illegal: csrr a0, 0x7C0\n ecall\n"
"   .globl sn_nops\n sn_nops: nop\n nop\n ecall\n"
"   .globl sn_sret_test\n"
"sn_sret_test:\n"
"   li t0, 0x20100\n"
"   csrs sstatus, t0\n"
"   csrr a1, sstatus\n"
"   la t1, vals\n"
"   sw a1, 0(t1)\n"
"   la t0, 1f\n"
"   csrw sepc, t0\n"
"   sret\n"
"1: csrr a0, sstatus\n"
"   ecall\n"
"   .globl sn_sie_wait\n"
"sn_sie_wait:\n"
"   csrr a0, sip\n"
"   csrsi sstatus, 2\n"
"   .globl sn_after_csrsi\n"
"sn_after_csrsi:\n"
"   csrci sstatus, 2\n"
"   ecall\n"
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

static uint32_t mret_to_m(uint32_t set)
{
    uint32_t v;
    __asm__ volatile (
        "csrs mstatus, %1\n"
        "la t0, 1f\n"
        "csrw mepc, t0\n"
        "li t0, 0x1800\n"
        "csrs mstatus, t0\n"
        "mret\n"
        "1: csrr %0, mstatus\n"
        : "=r"(v) : "r"(set) : "t0", "memory");
    return v;
}

int main(void)
{
    uint32_t saved_mtvec, v, r;

    bao_init();
    mini_printf("\r\nmmu_priv: privilege and CSR checks\r\n");

    saved_mtvec = CSRR(mtvec);
    CSRW(mtvec, (uint32_t)m_trap);
    CSRW(stvec, (uint32_t)s_trap);
    CSRW(satp, 0);

    /* identity and info CSRs */
    eq("misa reads MXL 1 and no extensions", CSRR(misa), 1u << 30);
    eq("mvendorid 1", CSRR(mvendorid), 1);
    eq("marchid 2", CSRR(marchid), 2);
    eq("mimpid 3", CSRR(mimpid), 3);
    eq("mhartid 0", CSRR(mhartid), 0);
    eq("0xCC0 data cache info", CSRR(0xCC0), 0x02001000u);

    /* mstatus and its supervisor view */
    CSRW(mstatus, 0xFFFFFFF7u);
    eq("mstatus writable bits", CSRR(mstatus), 0x000E19A2u);
    eq("sstatus view", CSRR(sstatus), 0x000E0122u);
    CSRW(mstatus, 0x00001000u);
    eq("MPP 2 keeps the old value", CSRR(mstatus) & 0x1800u, 0x1800u);
    CSRW(mstatus, 0x00000800u);
    eq("MPP 1 is kept", CSRR(mstatus) & 0x1800u, 0x0800u);
    CSRW(sstatus, 0xFFFFFFFFu);
    eq("sstatus write reaches MPRV", CSRR(mstatus), 0x000E0922u);
    CSRW(mstatus, 0x1800u);

    /* delegation, enables and pendings */
    CSRW(medeleg, 0xFFFFFFFFu);
    eq("medeleg bits", CSRR(medeleg), 0x0000B3F7u);
    CSRW(medeleg, 0);
    CSRW(mideleg, 0xFFFFFFFFu);
    eq("mideleg bits", CSRR(mideleg), 0x00000222u);
    CSRW(mideleg, 0);
    CSRW(mie, 0xFFFFFFFFu);
    eq("mie bits", CSRR(mie), 0x00000AAAu);
    eq("sie view", CSRR(sie), 0x00000222u);
    CSRW(sie, 0);
    eq("sie write leaves M bits", CSRR(mie), 0x00000888u);
    CSRW(mie, 0);
    CSRW(mip, 0xFFFFFFFFu);
    eq("mip soft bits", CSRR(mip), 0x00000222u);
    eq("sip view", CSRR(sip), 0x00000222u);
    CSRW(sip, 0);
    eq("sip write clears", CSRR(mip), 0);
    CSRW(scause, 0xFFFFFFFFu);
    eq("scause keeps 31 and 3:0", CSRR(scause), 0x8000000Fu);
    CSRW(mcause, 0xFFFFFFFFu);
    eq("mcause keeps 31 and 3:0", CSRR(mcause), 0x8000000Fu);
    CSRW(stvec, 0x61000003u);
    eq("stvec keeps mode bits", CSRR(stvec), 0x61000003u);
    CSRW(stvec, (uint32_t)s_trap);
    CSRW(satp, 0x7FFFFFFFu);
    eq("satp bare with all other bits", CSRR(satp), 0x7FFFFFFFu);
    CSRW(satp, 0);
    CSRW(mepc, 0x12345671u);
    eq("mepc keeps bit 0", CSRR(mepc), 0x12345671u);

    /* mret back to M: MPP to U, MPIE to 1, MPRV kept */
    v = mret_to_m(0x20000u);
    eq("mret to M sets MPP to U", v & 0x1880u, 0x0080u);
    eq("mret to M keeps MPRV", v & 0x20000u, 0x20000u);
    CSRW(mstatus, 0x1800u);

    /* ecall and ebreak from each privilege */
    clear_recs();
    run_lower(U, (uint32_t)sn_ecall, 0, 0);
    eq("ecall from U cause", mrec[0], 8);
    eq("ecall from U tval", mrec[1], 0x00000073u);
    eq("ecall from U epc", mrec[2], (uint32_t)sn_ecall);
    eq("ecall from U MPP", mrec[3] & 0x1800u, 0);
    CSRW(mstatus, 0x1800u | 0x20000u);
    clear_recs();
    run_lower(S, (uint32_t)sn_ecall, 0, 0);
    eq("ecall from S cause", mrec[0], 9);
    eq("ecall from S MPP", mrec[3] & 0x1800u, 0x0800u);
    eq("mret to S clears MPRV", mrec[3] & 0x20000u, 0);
    clear_recs();
    __asm__ volatile (".option push\n .option norvc\n ecall\n .option pop" ::: "memory");
    eq("ecall from M cause", mrec[0], 11);
    eq("ecall from M tval", mrec[1], 0x00000073u);
    clear_recs();
    run_lower(U, (uint32_t)sn_ebreak, 0, 0);
    eq("ebreak from U cause", HC(mrec, 0), 3);
    eq("ebreak tval is the instruction", HT(mrec, 0), 0x00100073u);
    eq("ebreak epc", mrec[4] == 2 ? 1 : 0, 1);

    /* privilege checks on CSRs and xret */
    clear_recs();
    r = run_lower(U, (uint32_t)sn_csrr_sstatus, 0x1234u, 0);
    eq("U reads sstatus: illegal", HC(mrec, 0), 2);
    eq("illegal CSR tval is the instruction", HT(mrec, 0), 0x10002573u);
    eq("illegal CSR leaves rd", r, 0x1234u);
    clear_recs();
    r = run_lower(U, (uint32_t)sn_csrr_cycle, 0, 0);
    eq("U reads cycle", (r != 0 && mrec[4] == 1) ? 1 : 0, 1);
    clear_recs();
    run_lower(U, (uint32_t)sn_csrw_cycle, 5, 0);
    eq("csrw cycle is illegal", HC(mrec, 0), 2);
    clear_recs();
    r = run_lower(U, (uint32_t)sn_csrrs_cycle, 0, 1);
    eq("csrrs cycle with rs1 is legal", (r != 0 && mrec[4] == 1) ? 1 : 0, 1);
    clear_recs();
    run_lower(U, (uint32_t)sn_mret, 0, 0);
    eq("mret from U illegal", HC(mrec, 0), 2);
    eq("mret tval", HT(mrec, 0), 0x30200073u);
    clear_recs();
    run_lower(U, (uint32_t)sn_sret, 0, 0);
    eq("sret from U illegal", HC(mrec, 0), 2);
    clear_recs();
    run_lower(S, (uint32_t)sn_mret, 0, 0);
    eq("mret from S illegal", HC(mrec, 0), 2);
    clear_recs();
    run_lower(S, (uint32_t)sn_csrr_mstatus, 0, 0);
    eq("S reads mstatus: illegal", HC(mrec, 0), 2);
    CSRW(0x9C0, 0x00080000u);
    clear_recs();
    r = run_lower(S, (uint32_t)sn_csrr_sim, 0, 0);
    eq("S reads 0x9C0", mrec[4] == 1 ? r : 0, 0x00080000u);
    CSRW(0x9C0, 0);
    clear_recs();
    run_lower(S, (uint32_t)sn_csrw_sip_arr, 1, 0);
    eq("csrw 0xDC0 is illegal", HC(mrec, 0), 2);
    clear_recs();
    run_lower(S, (uint32_t)sn_csrrs_sip_arr, 0, 1);
    eq("csrrs 0xDC0 with rs1 is legal", mrec[4], 1);
    clear_recs();
    r = run_lower(U, (uint32_t)sn_sfence, 0, 0);
    eq("sfence.vma legal in U", mrec[4] == 1 ? r : 0, 1);
    clear_recs();
    r = run_lower(S, (uint32_t)sn_sret_test, 0, 0);
    eq("sstatus sets MPRV from S", vals[0] & 0x20100u, 0x20100u);
    eq("sret clears SPP and MPRV, SPIE to 1", r & 0x20120u, 0x00020u);
    eq("sret to S stays in S", mrec[0], 9);

    /* exception delegation */
    CSRW(medeleg, (1u << 2) | (1u << 8));
    clear_recs();
    run_lower(U, (uint32_t)sn_illegal, 0, 0);
    eq("illegal from U goes to S", HC(srec, 0), 2);
    eq("delegated illegal stval", HT(srec, 0), 0x7C002573u);
    eq("ecall from U goes to S", HC(srec, 1), 8);
    eq("sepc of the ecall", srec[2], (uint32_t)sn_illegal + 4);
    eq("SPP 0 after trap from U", srec[3] & 0x100u, 0);
    eq("then ecall from S to M", mrec[0], 9);
    eq("M saw only that", mrec[4], 1);
    clear_recs();
    run_lower(S, (uint32_t)sn_illegal, 0, 0);
    eq("illegal from S stays in S", HC(srec, 0), 2);
    eq("SPP 1 after trap from S", srec[3] & 0x100u, 0x100u);
    eq("ecall from S not delegated", mrec[0], 9);
    clear_recs();
    __asm__ volatile (".option push\n .option norvc\n csrr a0, 0x7C0\n .option pop" ::: "a0", "memory");
    eq("illegal in M never delegated", mrec[0], 2);
    eq("S saw nothing from M", srec[4], 0);
    /* everything but ecall from S, which the S handler uses to leave */
    CSRW(medeleg, ~(1u << 9));
    clear_recs();
    run_lower(U, (uint32_t)sn_ebreak, 0, 0);
    eq("ebreak is not delegable", HC(mrec, 0), 3);
    eq("ecall from U delegated", HC(srec, 0), 8);
    CSRW(medeleg, 0);

    /* interrupts: soft pendings and delegation */
    CSRW(mideleg, 0x2u);
    CSRW(mie, 0x2u);
    CSRW(mip, 0x2u);
    clear_recs();
    run_lower(U, (uint32_t)sn_nops, 0, 0);
    eq("SSIP delegated, taken in U", srec[5], 0x80000001u);
    eq("taken before the first instruction", srec[6], (uint32_t)sn_nops);
    eq("M saw no interrupt", mrec[7], 0);
    CSRW(mip, 0x2u);
    CSRW(stval, 0x5555u);
    clear_recs();
    r = run_lower(S, (uint32_t)sn_sie_wait, 0, 0);
    eq("S with SIE 0 holds it, sip shows it", r & 2u, 2u);
    eq("taken after csrsi sstatus", srec[5], 0x80000001u);
    eq("epc is the next instruction", srec[6], (uint32_t)sn_after_csrsi);
    eq("interrupt leaves stval", srec[17], 0x5555u);
    CSRW(mideleg, 0);
    CSRW(mip, 0x2u);
    CSRW(mtval, 0x6666u);
    clear_recs();
    run_lower(S, (uint32_t)sn_nops, 0, 0);
    eq("SSIP not delegated goes to M from S", mrec[5], 0x80000001u);
    eq("interrupt leaves mtval", mrec[17], 0x6666u);
    CSRW(mideleg, 0x222u);
    CSRW(mie, 0x222u);
    CSRW(mip, 0x20u);
    clear_recs();
    run_lower(U, (uint32_t)sn_nops, 0, 0);
    eq("STIP delegated", srec[5], 0x80000005u);
    CSRW(mip, 0x200u);
    clear_recs();
    run_lower(U, (uint32_t)sn_nops, 0, 0);
    eq("SEIP soft delegated", srec[5], 0x80000009u);
    CSRW(mideleg, 0);
    CSRW(mip, 0x222u);
    clear_recs();
    run_lower(S, (uint32_t)sn_nops, 0, 0);
    eq("M priority: SEIP before SSIP and STIP", mrec[5], 0x80000009u);
    CSRW(mip, 0x22u);
    clear_recs();
    run_lower(S, (uint32_t)sn_nops, 0, 0);
    eq("M priority: SSIP before STIP", mrec[5], 0x80000001u);
    CSRW(mideleg, 0x2u);
    CSRW(mip, 0x22u);
    clear_recs();
    run_lower(U, (uint32_t)sn_nops, 0, 0);
    eq("M level STIP before delegated SSIP", mrec[5], 0x80000005u);
    eq("S saw none of it", srec[7], 0);
    CSRW(mip, 0);

    /* supervisor external interrupt from irq array 19 */
    IRQ19_PENDING = 0xFFFF;
    IRQ19_ENABLE = 1;
    CSRW(mideleg, 0x200u);
    CSRW(mie, 0x200u);
    CSRW(0x9C0, 1u << 19);
    IRQ19_SOFT = 1;
    eq("mip SEIP from the line", CSRR(mip), 0x200u);
    eq("0xDC0 shows line 19", CSRR(0xDC0), 1u << 19);
    eq("0xFC0 masked by MIM", CSRR(0xFC0), 0);
    clear_recs();
    run_lower(U, (uint32_t)sn_nops, 0, 0);
    eq("SEIP from 0x9C0 delegated", srec[5], 0x80000009u);
    eq("handler read 0xDC0", srec[16], 1u << 19);
    CSRW(mideleg, 0);
    CSRW(0x9C0, 1u << 19);
    clear_recs();
    run_lower(U, (uint32_t)sn_nops, 0, 0);
    eq("SEIP from 0x9C0 to M when not delegated", mrec[5], 0x80000009u);
    CSRW(0x9C0, 0);
    CSRW(0xBC0, 1u << 19);
    CSRW(mie, 0x800u);
    clear_recs();
    run_lower(U, (uint32_t)sn_nops, 0, 0);
    eq("MEIP from 0xBC0 in U", mrec[5], 0x8000000Bu);
    eq("handler read 0xFC0", mrec[16], 1u << 19);
    IRQ19_PENDING = 0xFFFF;
    IRQ19_ENABLE = 0;
    CSRW(0x9C0, 0);
    CSRW(0xBC0, 0);
    CSRW(mie, 0);

    /* mtvec mode bits do not vector */
    CSRW(mtvec, (uint32_t)m_trap | 1u);
    CSRW(mie, 0x2u);
    CSRW(mip, 0x2u);
    clear_recs();
    run_lower(U, (uint32_t)sn_nops, 0, 0);
    eq("mtvec mode 1 traps to base", (mrec[5] == 0x80000001u && mrec[0] == 8) ? 1 : 0, 1);
    CSRW(mtvec, (uint32_t)m_trap);

    /* WFI wakes on a pending enabled interrupt even with MIE 0 */
    CSRW(mip, 0x2u);
    __asm__ volatile ("csrci mstatus, 8\n wfi" ::: "memory");
    eq("wfi woke with MIE 0", CSRR(mip) & 2u, 2u);
    CSRW(mip, 0);
    CSRW(mie, 0);

    CSRW(mtvec, saved_mtvec);
    mini_printf("[Done] mmu_priv %d of %d passed\r\n", passed, total);
    for (;;) {
    }
}
