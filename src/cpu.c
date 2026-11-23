/*
 * cpu.c: RV32IMAC interpreter for the Baochip-1x VexRiscv core
 *
 * Copyright (c) 2026 Armstrong Subero
 * SPDX-License-Identifier: Apache-2.0
 *
 * M, S and U privilege, Sv32 MMU, interrupt arrays and the AesZknPlugin
 * scalar AES instructions, with VexRiscv pipeline timing.
 */

#include <string.h>
#include <stdio.h>
#include "bao_int.h"

#define CSR_MIM 0xBC0
#define CSR_MIP 0xFC0
#define CSR_SIM 0x9C0
#define CSR_SIP 0xDC0

#define SSTATUS_MASK  (MSTATUS_SIE | MSTATUS_SPIE | MSTATUS_SPP | MSTATUS_MPRV | \
                       MSTATUS_SUM | MSTATUS_MXR)
#define MSTATUS_MASK  (SSTATUS_MASK | MSTATUS_MIE | MSTATUS_MPIE | MSTATUS_MPP)
#define S_INTS        (MIP_SSIP | MIP_STIP | MIP_SEIP)
#define MEDELEG_MASK  0x0000B3F7u
#define XCAUSE_MASK   0x8000000Fu

enum { ACC_FETCH, ACC_LOAD, ACC_STORE, ACC_AMO };

/*
 * Instruction timing in ACLK cycles, from VexRiscv_CramSoC.v (RTL lines in
 * brackets). One slot per instruction unless something below holds it.
 *
 * Result latency, in slots from leaving decode until a dependent
 * instruction may leave decode (hazard check, 5640 to 5716):
 *   ALU, LUI, AUIPC, JAL, JALR        1   bypass from execute
 *   shifts, CSR reads, DIV, REM       2   bypass from memory
 *   loads, LR, SC, AMO, MUL, AES32    3   bypass from writeback only
 * Next instruction in decode, in slots after a redirect:
 *   JAL, backward branch taken        2   predicted in decode (4969)
 *   JALR, any wrong guess             4   jump from memory (3886, 6559)
 *   mret, sret                        5   jump from writeback (6186)
 *   exception                         6   writeback, then hadException (7909)
 *   interrupt                         5   liberator drains 3 stages (6123)
 *   satp write, sfence.vma            3   redo from decode's PC (5956)
 *   fence.i                         133   instruction cache clears 128 lines (12109)
 *   target is a 32 bit instruction
 *   in the upper half of a word      +1   second fetch word (4909)
 * Held in execute:
 *   CSR, sfence.vma                   leave decode 3 slots after the one before (6193)
 *   data cache flush 0x500F         130   after loads and stores drain (11467);
 *                                     3   for one line when rs1 is not x0 (5180)
 * Freezes that stop the whole front of the pipe (7084):
 *   DIV, REM                         33   in memory (5757 to 5790)
 *   uncached load                     3   cmd register and response register (5078, 7826)
 *   AMO in the cached range           1   writeback waits for the result register
 *                                         (11264, 11432)
 */
#define LAT_MEM        2
#define LAT_WB         3
#define JUMP_DECODE    2
#define JUMP_MEMORY    4
#define JUMP_XRET      5
#define JUMP_TRAP      6
#define JUMP_IRQ       5
#define JUMP_REDO      3
#define JUMP_SPLIT     1
#define FENCE_I        133
#define CSR_DRAIN      3
#define DFLUSH_ALL     130
#define DFLUSH_LINE    3
#define FREEZE_DIV     33
#define FREEZE_IO_LOAD 3
#define FREEZE_AMO     1

#define NO_RD 32

/* MmuPlugin ioRange: top nibble 4, 5 or A to F bypasses the data cache */
static inline int io_range(uint32_t pa)
{
    return (0xFC30u >> (pa >> 28)) & 1;
}

static inline uint64_t later(uint64_t a, uint64_t b)
{
    return a > b ? a : b;
}

static void pipe_reset(bao_t *b)
{
    b->slot = b->fetch_ok = 8;
    b->last_issue = b->last_mem = 0;
    b->hz_rd[0] = b->hz_rd[1] = NO_RD;
    b->hz_ready[0] = b->hz_ready[1] = 0;
    b->redirected = 1;
    b->wb_hold = 0;
}

/* WFI gates clkvex, so mcycle and Timer0 stop (vexsys.sv 124) */
#define VEX_GATE_DELAY 64u

void cpu_wfi_enter(bao_t *b)
{
    b->wfi = 1;
    b->wfi_since = b->cycles;
    b->wfi_gate = soc_irq_lines(b) == 0;
}

static uint64_t wfi_gated_so_far(const bao_t *b)
{
    uint64_t from = b->wfi_since + VEX_GATE_DELAY;
    if (!b->wfi || !b->wfi_gate || b->cycles <= from)
        return 0;
    return b->cycles - from;
}

void cpu_wfi_leave(bao_t *b)
{
    uint64_t g = wfi_gated_so_far(b);
    b->vex_gated += g;
    b->wfi = 0;
    if (g)
        bao_kick(b);
}

uint64_t vex_now(bao_t *b)
{
    return b->cycles - b->vex_gated - wfi_gated_so_far(b);
}

/* the core sat in WFI: everything ahead of it has drained */
void cpu_idle(bao_t *b)
{
    uint64_t s = later(b->slot, b->fetch_ok);
    b->slot = b->fetch_ok = s;
    b->hz_rd[0] = b->hz_rd[1] = NO_RD;
}

static inline uint64_t ahead(uint64_t t, uint64_t now)
{
    return t > now ? t - now : 0;
}

/* everything that decides future stalls, relative to the next slot */
uint64_t cpu_pipe_state(const bao_t *b)
{
    uint64_t s = b->slot, r0 = ahead(b->hz_ready[0], s), r1 = ahead(b->hz_ready[1], s);
    return ahead(b->fetch_ok, s) | ahead(b->last_issue + CSR_DRAIN, s) << 8 |
           ahead(b->last_mem + CSR_DRAIN, s) << 16 | r0 << 24 |
           (uint64_t)(r0 ? b->hz_rd[0] : NO_RD) << 32 | r1 << 40 |
           (uint64_t)(r1 ? b->hz_rd[1] : NO_RD) << 48 | (uint64_t)b->redirected << 56;
}

static const char *cause_name(uint32_t c)
{
    switch (c) {
    case 0: return "instruction address misaligned";
    case 1: return "instruction access fault";
    case 2: return "illegal instruction";
    case 3: return "breakpoint";
    case 4: return "load address misaligned";
    case 5: return "load access fault";
    case 6: return "store address misaligned";
    case 7: return "store access fault";
    case 8: return "ecall from U mode";
    case 9: return "ecall from S mode";
    case 11: return "ecall from M mode";
    case 12: return "instruction page fault";
    case 13: return "load page fault";
    case 15: return "store page fault";
    default: return "exception";
    }
}

/* a change ends the block so the run loop picks the matching interpreter */
static void vm_update(bao_t *b)
{
    int on = (int)(b->satp >> 31), i, d;
    i = on && b->priv != PRIV_M;
    d = on && !(b->priv == PRIV_M && (!(b->mstatus & MSTATUS_MPRV) ||
                                       (b->mstatus & MSTATUS_MPP) == MSTATUS_MPP));
    if (i != b->vm_i || d != b->vm_d)
        bao_kick(b);
    b->vm_i = i;
    b->vm_d = d;
}

static void tlb_flush(bao_t *b)
{
    int p, i;
    for (p = 0; p < 2; p++)
        for (i = 0; i < 8; i++)
            b->tlb[p][i].valid = 0;
}

/* a trap or xret drops the lines that hold a faulting walk */
static void tlb_drop_faults(bao_t *b)
{
    int p, i;
    for (p = 0; p < 2; p++)
        for (i = 0; i < 8; i++)
            if (b->tlb[p][i].exc)
                b->tlb[p][i].valid = 0;
}

void cpu_reset(bao_t *b)
{
    memset(b->x, 0, sizeof(b->x));
    b->pc = b->entry;
    b->priv = PRIV_M;
    b->mstatus = MSTATUS_MPP | MSTATUS_SPP;
    b->mie = 0;
    b->mtvec = 0x60000000u;
    b->mepc = b->mcause = b->mtval = b->mscratch = 0;
    b->misa = 0x40000000u;
    b->medeleg = b->mideleg = b->mip_soft = 0;
    b->stvec = b->sscratch = b->sepc = b->scause = b->stval = 0;
    b->mim = b->sim = 0;
    b->satp = 0;
    tlb_flush(b);
    b->tlb_next[0] = b->tlb_next[1] = 0;
    vm_update(b);
    cpu_wfi_leave(b);
    b->reserved_valid = 0;
    pipe_reset(b);
}

/* mip as the core sees it; MTIP and MSIP are tied low in cram_axi.sv */
static uint32_t mip_read(bao_t *b)
{
    uint32_t lines = soc_irq_lines(b), v = b->mip_soft;
    if (lines & b->mim) v |= MIP_MEIP;
    if (lines & b->sim) v |= MIP_SEIP;
    return v;
}

int cpu_wake(bao_t *b)
{
    return b->mie && (mip_read(b) & b->mie) != 0;
}

int cpu_irq_pending(bao_t *b, int *target)
{
    static const int mprio[] = { 11, 3, 7, 9, 1, 5 };
    static const int sprio[] = { 9, 1, 5 };
    int m_en = (b->mstatus & MSTATUS_MIE) || b->priv < PRIV_M;
    int s_en = b->priv == PRIV_U || (b->priv == PRIV_S && (b->mstatus & MSTATUS_SIE));
    uint32_t pend;
    int i;

    if ((!m_en && !s_en) || !b->mie)
        return -1;
    pend = mip_read(b) & b->mie;
    if (m_en && (pend & ~b->mideleg))
        for (i = 0; i < 6; i++)
            if (pend & ~b->mideleg & (1u << mprio[i])) {
                *target = PRIV_M;
                return mprio[i];
            }
    if (s_en && (pend & b->mideleg))
        for (i = 0; i < 3; i++)
            if (pend & b->mideleg & (1u << sprio[i])) {
                *target = PRIV_S;
                return sprio[i];
            }
    return -1;
}

static void enter_trap(bao_t *b, uint32_t cause, uint32_t tval, int irq, uint32_t target)
{
    b->side++;
    if (target == PRIV_S) {
        b->sepc = b->pc;
        b->scause = cause | (irq ? 0x80000000u : 0);
        if (!irq)
            b->stval = tval;
        b->mstatus &= ~(MSTATUS_SPIE | MSTATUS_SPP);
        if (b->mstatus & MSTATUS_SIE)
            b->mstatus |= MSTATUS_SPIE;
        if (b->priv & 1)
            b->mstatus |= MSTATUS_SPP;
        b->mstatus &= ~MSTATUS_SIE;
        b->pc = b->stvec & ~3u;
    } else {
        b->mepc = b->pc;
        b->mcause = cause | (irq ? 0x80000000u : 0);
        if (!irq)
            b->mtval = tval;
        b->mstatus &= ~(MSTATUS_MPIE | MSTATUS_MPP);
        if (b->mstatus & MSTATUS_MIE)
            b->mstatus |= MSTATUS_MPIE;
        b->mstatus |= b->priv << 11;
        b->mstatus &= ~MSTATUS_MIE;
        b->pc = b->mtvec & ~3u;
    }
    b->priv = target;
    tlb_drop_faults(b);
    vm_update(b);
    bao_kick(b);
}

static void take_trap(bao_t *b, uint32_t cause, uint32_t tval)
{
    uint32_t target = PRIV_M;
    if (b->stop_on_exception) {
        bao_halt(b, cause == 3 ? BAO_STOP_BREAK : BAO_STOP_EXCEPTION,
                 "%s at %s, tval 0x%08X", cause_name(cause),
                 bao_where(b, b->pc), tval);
        return;
    }
    if (b->priv != PRIV_M && ((b->medeleg >> cause) & 1))
        target = PRIV_S;
    enter_trap(b, cause, tval, 0, target);
}

void cpu_take_irq(bao_t *b, int code, int target)
{
    b->fetch_ok = later(b->slot, b->fetch_ok) + JUMP_IRQ;
    b->redirected = 1;
    cpu_wfi_leave(b);
    enter_trap(b, (uint32_t)code, 0, 1, (uint32_t)target);
}

static void xret(bao_t *b, int to_s)
{
    uint32_t back;
    if (to_s) {
        back = (b->mstatus & MSTATUS_SPP) ? PRIV_S : PRIV_U;
        b->mstatus &= ~(MSTATUS_SPP | MSTATUS_SIE | MSTATUS_MPRV);
        if (b->mstatus & MSTATUS_SPIE)
            b->mstatus |= MSTATUS_SIE;
        b->mstatus |= MSTATUS_SPIE;
    } else {
        back = (b->mstatus & MSTATUS_MPP) >> 11;
        b->mstatus &= ~(MSTATUS_MPP | MSTATUS_MIE);
        if (back != PRIV_M)
            b->mstatus &= ~MSTATUS_MPRV;
        if (b->mstatus & MSTATUS_MPIE)
            b->mstatus |= MSTATUS_MIE;
        b->mstatus |= MSTATUS_MPIE;
    }
    b->priv = back;
    b->side++;
    tlb_drop_faults(b);
    vm_update(b);
    bao_kick(b);
}

/* read only CSRs are illegal for an instruction that does not read them */
static int csr_read_only(uint32_t n)
{
    switch (n) {
    case 0xCC0: case 0xF11: case 0xF12: case 0xF13: case 0xF14:
    case 0xC00: case 0xC80: case 0xC02: case 0xC82:
    case CSR_MIP: case CSR_SIP:
        return 1;
    default:
        return 0;
    }
}

static int csr_read(bao_t *b, uint32_t n, uint32_t *v)
{
    switch (n) {
    case 0x300: *v = b->mstatus; return 0;
    case 0x301: *v = b->misa; return 0;
    case 0x302: *v = b->medeleg; return 0;
    case 0x303: *v = b->mideleg; return 0;
    case 0x304: *v = b->mie; return 0;
    case 0x305: *v = b->mtvec; return 0;
    case 0x340: *v = b->mscratch; return 0;
    case 0x341: *v = b->mepc; return 0;
    case 0x342: *v = b->mcause; return 0;
    case 0x343: *v = b->mtval; return 0;
    case 0x344: *v = mip_read(b); return 0;
    case 0x100: *v = b->mstatus & SSTATUS_MASK; return 0;
    case 0x104: *v = b->mie & S_INTS; return 0;
    case 0x105: *v = b->stvec; return 0;
    case 0x140: *v = b->sscratch; return 0;
    case 0x141: *v = b->sepc; return 0;
    case 0x142: *v = b->scause; return 0;
    case 0x143: *v = b->stval; return 0;
    case 0x144: *v = mip_read(b) & S_INTS; return 0;
    case 0x180: *v = b->satp; return 0;
    case 0xB00: case 0xC00: *v = (uint32_t)vex_now(b); return 0;
    case 0xB80: case 0xC80: *v = (uint32_t)(vex_now(b) >> 32); return 0;
    case 0xB02: case 0xC02: *v = (uint32_t)b->instret; return 0;
    case 0xB82: case 0xC82: *v = (uint32_t)(b->instret >> 32); return 0;
    case 0xF11: *v = 1; return 0;
    case 0xF12: *v = 2; return 0;
    case 0xF13: *v = 3; return 0;
    case 0xF14: *v = 0; return 0;
    case 0xCC0: *v = 0x02001000u; return 0;
    case CSR_MIM: *v = b->mim; return 0;
    case CSR_MIP: *v = soc_irq_lines(b) & b->mim; return 0;
    case CSR_SIM: *v = b->sim; return 0;
    case CSR_SIP: *v = soc_irq_lines(b) & b->sim; return 0;
    default: return -1;
    }
}

static void csr_write(bao_t *b, uint32_t n, uint32_t v)
{
    switch (n) {
    case 0x300:
        if ((v & MSTATUS_MPP) == 0x1000u)
            v = (v & ~MSTATUS_MPP) | (b->mstatus & MSTATUS_MPP);
        b->mstatus = v & MSTATUS_MASK;
        vm_update(b);
        bao_kick(b);
        break;
    case 0x100:
        b->mstatus = (b->mstatus & ~SSTATUS_MASK) | (v & SSTATUS_MASK);
        vm_update(b);
        bao_kick(b);
        break;
    case 0x301: b->misa = v & 0xC3FFFFFFu; break;
    case 0x302: b->medeleg = v & MEDELEG_MASK; break;
    case 0x303: b->mideleg = v & S_INTS; bao_kick(b); break;
    case 0x304: b->mie = v & 0x00000AAAu; bao_kick(b); break;
    case 0x104: b->mie = (b->mie & ~S_INTS) | (v & S_INTS); bao_kick(b); break;
    case 0x344: case 0x144: b->mip_soft = v & S_INTS; bao_kick(b); break;
    case 0x305: b->mtvec = v; break;
    case 0x105: b->stvec = v; break;
    case 0x340: b->mscratch = v; break;
    case 0x341: b->mepc = v; break;
    case 0x342: b->mcause = v & XCAUSE_MASK; break;
    case 0x343: b->mtval = v; break;
    case 0x140: b->sscratch = v; break;
    case 0x141: b->sepc = v; break;
    case 0x142: b->scause = v & XCAUSE_MASK; break;
    case 0x143: b->stval = v; break;
    case 0x180:
        b->satp = v;
        tlb_flush(b);
        vm_update(b);
        break;
    case CSR_MIM: b->mim = v; bao_kick(b); break;
    case CSR_SIM: b->sim = v; bao_kick(b); break;
    default: break;
    }
}

static int pte_read(bao_t *b, uint32_t a, uint32_t *v)
{
    uint8_t *p = b->rdmap[a >> 16];
    if (p) {
        memcpy(v, p + (a & 0xFFFF), 4);
        return 0;
    }
    *v = 0;
    return bus_read(b, a, 4, v) != 0;
}

#define PTE_V(p) ((p) & 1)
#define PTE_R(p) (((p) >> 1) & 1)
#define PTE_W(p) (((p) >> 2) & 1)
#define PTE_X(p) (((p) >> 3) & 1)

static int pte_fault(uint32_t pte, int err)
{
    return !PTE_V(pte) || (!PTE_R(pte) && PTE_W(pte)) || err;
}

/* the walker only reads: A and D are neither checked nor set */
static int tlb_refill(bao_t *b, int port, uint32_t va)
{
    uint32_t vpn1 = va >> 22, vpn0 = (va >> 12) & 0x3FF, pte, a;
    int err, super = 1, exc;
    tlb_t *t;

    a = ((b->satp & 0xFFFFFu) << 12) | (vpn1 << 2);
    err = pte_read(b, a, &pte);
    if (b->stop)
        return -1;
    exc = pte_fault(pte, err);
    if (!exc && !PTE_R(pte) && !PTE_X(pte)) {
        a = (((pte >> 20) & 0x3FFu) << 22) | (((pte >> 10) & 0x3FFu) << 12) | (vpn0 << 2);
        err = pte_read(b, a, &pte);
        if (b->stop)
            return -1;
        super = 0;
        exc = pte_fault(pte, err);
        if (!exc && !PTE_R(pte) && !PTE_X(pte)) {
            bao_halt(b, BAO_STOP_HALT, "page walk for 0x%08X at %s never ends: level 0 PTE 0x%08X at 0x%08X is a pointer",
                     va, bao_where(b, b->pc), pte, a);
            return -1;
        }
    }
    t = &b->tlb[port][b->tlb_next[port]];
    b->tlb_next[port] = (b->tlb_next[port] + 1) & 7;
    t->valid = 1;
    t->super = (uint8_t)super;
    t->exc = (uint8_t)(exc || (super && ((pte >> 10) & 0x3FFu)));
    t->vpn1 = (uint16_t)vpn1;
    t->vpn0 = (uint16_t)vpn0;
    t->ppn0 = (uint16_t)((pte >> 10) & 0x3FFu);
    t->ppn1 = (uint16_t)((pte >> 20) & 0x3FFu);
    t->r = (uint8_t)PTE_R(pte);
    t->w = (uint8_t)PTE_W(pte);
    t->x = (uint8_t)PTE_X(pte);
    t->u = (uint8_t)((pte >> 4) & 1);
    b->side++;
    return 0;
}

/* the line used is the one at the OR of every hit index (MmuPlugin cacheLine) */
static int tlb_lookup(bao_t *b, int port, uint32_t va)
{
    uint32_t vpn1 = va >> 22, vpn0 = (va >> 12) & 0x3FF;
    int i, idx = 0, hit = 0;
    for (i = 0; i < 8; i++) {
        const tlb_t *t = &b->tlb[port][i];
        if (t->valid && t->vpn1 == vpn1 && (t->super || t->vpn0 == vpn0)) {
            idx |= i;
            hit = 1;
        }
    }
    return hit ? idx : -1;
}

/* returns 0 with the physical address, or the page fault cause */
static int xlate(bao_t *b, uint32_t va, int acc, uint32_t *pa)
{
    static const int fault[4] = { 12, 13, 15, 15 };
    int port = acc != ACC_FETCH, i, rd;
    const tlb_t *t;

    i = tlb_lookup(b, port, va);
    if (i < 0) {
        if (tlb_refill(b, port, va))
            return fault[acc];
        i = tlb_lookup(b, port, va);
    }
    t = &b->tlb[port][i];
    if (t->exc || (t->u && b->priv == PRIV_S && !(b->mstatus & MSTATUS_SUM)) ||
        (!t->u && b->priv == PRIV_U))
        return fault[acc];
    rd = t->r || ((b->mstatus & MSTATUS_MXR) && t->x);
    switch (acc) {
    case ACC_FETCH: if (!t->x) return fault[acc]; break;
    case ACC_LOAD:  if (!rd) return fault[acc]; break;
    case ACC_STORE: if (!t->w) return fault[acc]; break;
    default:        if (!t->w || !rd) return fault[acc]; break;
    }
    *pa = ((uint32_t)t->ppn1 << 22) | ((t->super ? (va >> 12) & 0x3FFu : t->ppn0) << 12) |
          (va & 0xFFFu);
    return 0;
}

static inline int load_phys(bao_t *b, uint32_t a, int size, uint32_t *v)
{
    uint8_t *p = b->rdmap[a >> 16];
    int e;
    if (p) {
        p += a & 0xFFFF;
        if (size == 4) { uint32_t t; memcpy(&t, p, 4); *v = t; }
        else if (size == 2) { uint16_t t; memcpy(&t, p, 2); *v = t; }
        else *v = *p;
        return 0;
    }
    e = bus_read(b, a, size, v) ? 5 : 0;
    if (io_range(a)) {
        b->cycles += FREEZE_IO_LOAD;
        b->wb_hold = 1;
    }
    return e;
}

static inline int store_phys(bao_t *b, uint32_t a, int size, uint32_t v)
{
    uint8_t *p = b->wrmap[a >> 16];
    int e;
    if (p) {
        p += a & 0xFFFF;
        b->reserved_valid = 0;
        if (size == 4) {
            uint32_t o; memcpy(&o, p, 4);
            if (o != v) { memcpy(p, &v, 4); b->side++; }
        } else if (size == 2) {
            uint16_t t = (uint16_t)v, o; memcpy(&o, p, 2);
            if (o != t) { memcpy(p, &t, 2); b->side++; }
        } else if (*p != (uint8_t)v) {
            *p = (uint8_t)v; b->side++;
        }
        return 0;
    }
    b->side++;
    e = bus_write(b, a, size, v) ? 7 : 0;
    if (!e)
        b->reserved_valid = 0;
    return e;
}

static inline int load_bare(bao_t *b, uint32_t a, int size, uint32_t *v)
{
    if (a & (uint32_t)(size - 1))
        return 4;
    return load_phys(b, a, size, v);
}

static inline int store_bare(bao_t *b, uint32_t a, int size, uint32_t v)
{
    if (a & (uint32_t)(size - 1))
        return 6;
    return store_phys(b, a, size, v);
}

static inline int load(bao_t *b, uint32_t a, int size, uint32_t *v)
{
    if (a & (uint32_t)(size - 1))
        return 4;
    if (b->vm_d) {
        int e = xlate(b, a, ACC_LOAD, &a);
        if (e)
            return e;
    }
    return load_phys(b, a, size, v);
}

static inline int store(bao_t *b, uint32_t a, int size, uint32_t v)
{
    if (a & (uint32_t)(size - 1))
        return 6;
    if (b->vm_d) {
        int e = xlate(b, a, ACC_STORE, &a);
        if (e)
            return e;
    }
    return store_phys(b, a, size, v);
}

static inline int fetch16(bao_t *b, uint32_t a, uint32_t *h)
{
    uint8_t *p = b->rdmap[a >> 16];
    uint16_t t;
    if (!p)
        return -1;
    memcpy(&t, p + (a & 0xFFFF), 2);
    *h = t;
    return 0;
}

/* translated fetch, faults report the virtual word address */
static int fetch_paged(bao_t *b, uint32_t pc, uint32_t *lo, uint32_t *hi, uint32_t *tval)
{
    uint32_t pa;
    int e = xlate(b, pc, ACC_FETCH, &pa);
    *tval = pc & ~3u;
    if (e)
        return e;
    if (fetch16(b, pa, lo))
        return 1;
    if ((*lo & 3) != 3)
        return 0;
    if (pc & 2) {
        *tval = pc + 2;
        e = xlate(b, pc + 2, ACC_FETCH, &pa);
        if (e)
            return e;
    } else {
        pa += 2;
    }
    return fetch16(b, pa, hi) ? 1 : 0;
}

#define RD(i)  (((i) >> 7) & 31)
#define RS1(i) (((i) >> 15) & 31)
#define RS2(i) (((i) >> 20) & 31)
#define F3(i)  (((i) >> 12) & 7)
#define F7(i)  ((i) >> 25)

static inline uint32_t enc_i(int32_t imm, uint32_t rs1, uint32_t f3, uint32_t rd, uint32_t op)
{
    return (((uint32_t)imm & 0xFFF) << 20) | (rs1 << 15) | (f3 << 12) | (rd << 7) | op;
}

static inline uint32_t enc_s(int32_t imm, uint32_t rs2, uint32_t rs1, uint32_t f3, uint32_t op)
{
    uint32_t u = (uint32_t)imm;
    return (((u >> 5) & 0x7F) << 25) | (rs2 << 20) | (rs1 << 15) | (f3 << 12) | ((u & 0x1F) << 7) | op;
}

static inline uint32_t enc_b(int32_t imm, uint32_t rs2, uint32_t rs1, uint32_t f3)
{
    uint32_t u = (uint32_t)imm;
    return (((u >> 12) & 1) << 31) | (((u >> 5) & 0x3F) << 25) | (rs2 << 20) | (rs1 << 15) |
           (f3 << 12) | (((u >> 1) & 0xF) << 8) | (((u >> 11) & 1) << 7) | 0x63;
}

static inline uint32_t enc_j(int32_t imm, uint32_t rd)
{
    uint32_t u = (uint32_t)imm;
    return (((u >> 20) & 1) << 31) | (((u >> 1) & 0x3FF) << 21) | (((u >> 11) & 1) << 20) |
           (((u >> 12) & 0xFF) << 12) | (rd << 7) | 0x6F;
}

static inline uint32_t enc_r(uint32_t f7, uint32_t rs2, uint32_t rs1, uint32_t f3, uint32_t rd, uint32_t op)
{
    return (f7 << 25) | (rs2 << 20) | (rs1 << 15) | (f3 << 12) | (rd << 7) | op;
}

static inline int32_t sext(uint32_t v, int bits)
{
    uint32_t m = 1u << (bits - 1);
    v &= (1u << bits) - 1;
    return (int32_t)((v ^ m) - m);
}

uint32_t bao_expand_c(uint32_t c)
{
    uint32_t q = c & 3, f = (c >> 13) & 7;
    uint32_t rd = (c >> 7) & 31, rs2 = (c >> 2) & 31;
    uint32_t rdp = ((c >> 2) & 7) + 8, rs1p = ((c >> 7) & 7) + 8;
    int32_t imm;

    if (q == 0) {
        switch (f) {
        case 0:
            imm = (int32_t)((((c >> 11) & 3) << 4) | (((c >> 7) & 0xF) << 6) |
                            (((c >> 6) & 1) << 2) | (((c >> 5) & 1) << 3));
            if (imm == 0) return 0;
            return enc_i(imm, 2, 0, rdp, 0x13);
        case 2:
            imm = (int32_t)((((c >> 10) & 7) << 3) | (((c >> 6) & 1) << 2) | (((c >> 5) & 1) << 6));
            return enc_i(imm, rs1p, 2, rdp, 0x03);
        case 6:
            imm = (int32_t)((((c >> 10) & 7) << 3) | (((c >> 6) & 1) << 2) | (((c >> 5) & 1) << 6));
            return enc_s(imm, rdp, rs1p, 2, 0x23);
        default:
            return 0;
        }
    }
    if (q == 1) {
        switch (f) {
        case 0:
            imm = sext((((c >> 12) & 1) << 5) | ((c >> 2) & 31), 6);
            return enc_i(imm, rd, 0, rd, 0x13);
        case 1:
        case 5:
            imm = sext((((c >> 12) & 1) << 11) | (((c >> 11) & 1) << 4) | (((c >> 9) & 3) << 8) |
                       (((c >> 8) & 1) << 10) | (((c >> 7) & 1) << 6) | (((c >> 6) & 1) << 7) |
                       (((c >> 3) & 7) << 1) | (((c >> 2) & 1) << 5), 12);
            return enc_j(imm, f == 1 ? 1 : 0);
        case 2:
            imm = sext((((c >> 12) & 1) << 5) | ((c >> 2) & 31), 6);
            return enc_i(imm, 0, 0, rd, 0x13);
        case 3:
            if (rd == 2) {
                imm = sext((((c >> 12) & 1) << 9) | (((c >> 6) & 1) << 4) | (((c >> 5) & 1) << 6) |
                           (((c >> 3) & 3) << 7) | (((c >> 2) & 1) << 5), 10);
                if (imm == 0) return 0;
                return enc_i(imm, 2, 0, 2, 0x13);
            }
            imm = sext((((c >> 12) & 1) << 17) | (((c >> 2) & 31) << 12), 18);
            if (imm == 0) return 0;
            return ((uint32_t)imm & 0xFFFFF000u) | (rd << 7) | 0x37;
        case 4: {
            uint32_t f2 = (c >> 10) & 3;
            if (f2 == 0 || f2 == 1) {
                if ((c >> 12) & 1) return 0;
                return enc_i((int32_t)(((c >> 2) & 31) | (f2 == 1 ? 0x400 : 0)), rs1p, 5, rs1p, 0x13);
            }
            if (f2 == 2) {
                imm = sext((((c >> 12) & 1) << 5) | ((c >> 2) & 31), 6);
                return enc_i(imm, rs1p, 7, rs1p, 0x13);
            }
            if ((c >> 12) & 1) return 0;
            switch ((c >> 5) & 3) {
            case 0: return enc_r(0x20, rdp, rs1p, 0, rs1p, 0x33);
            case 1: return enc_r(0, rdp, rs1p, 4, rs1p, 0x33);
            case 2: return enc_r(0, rdp, rs1p, 6, rs1p, 0x33);
            default: return enc_r(0, rdp, rs1p, 7, rs1p, 0x33);
            }
        }
        case 6:
        case 7:
            imm = sext((((c >> 12) & 1) << 8) | (((c >> 10) & 3) << 3) | (((c >> 5) & 3) << 6) |
                       (((c >> 3) & 3) << 1) | (((c >> 2) & 1) << 5), 9);
            return enc_b(imm, 0, rs1p, f == 6 ? 0 : 1);
        }
    }
    if (q == 2) {
        switch (f) {
        case 0:
            if ((c >> 12) & 1) return 0;
            return enc_i((int32_t)((c >> 2) & 31), rd, 1, rd, 0x13);
        case 2:
            if (rd == 0) return 0;
            imm = (int32_t)((((c >> 12) & 1) << 5) | (((c >> 4) & 7) << 2) | (((c >> 2) & 3) << 6));
            return enc_i(imm, 2, 2, rd, 0x03);
        case 4:
            if (!((c >> 12) & 1)) {
                if (rs2 == 0) {
                    if (rd == 0) return 0;
                    return enc_i(0, rd, 0, 0, 0x67);
                }
                return enc_r(0, rs2, 0, 0, rd, 0x33);
            }
            if (rd == 0 && rs2 == 0) return 0x00100073u;
            if (rs2 == 0) return enc_i(0, rd, 0, 1, 0x67);
            return enc_r(0, rs2, rd, 0, rd, 0x33);
        case 6:
            imm = (int32_t)((((c >> 9) & 0xF) << 2) | (((c >> 7) & 3) << 6));
            return enc_s(imm, rs2, 2, 2, 0x23);
        default:
            return 0;
        }
    }
    return 0;
}

/* scalar AES (AesZknPlugin, VexRiscv_CramSoC.v 5807 to 5946) */
static uint32_t aes_rom(uint32_t addr)
{
    uint8_t x = (uint8_t)addr;
    if (addr & 0x100) {
        uint8_t i = bao_aes_inv_sbox[x];
        return (uint32_t)bao_gmul(i, 14) | (uint32_t)bao_gmul(i, 9) << 8 |
               (uint32_t)bao_gmul(i, 13) << 16 | (uint32_t)bao_gmul(i, 11) << 24;
    }
    {
        uint8_t s = bao_aes_sbox[x];
        return (uint32_t)s | (uint32_t)bao_gmul(s, 2) << 8 | (uint32_t)bao_gmul(s, 3) << 16 |
               (uint32_t)bao_aes_inv_sbox[x] << 24;
    }
}

static uint32_t aes_rom_addr(uint32_t ins, uint32_t rs2)
{
    return (((ins >> 27) & (ins >> 26) & 1) << 8) | ((rs2 >> (8 * (ins >> 30))) & 0xFF);
}

/* memory stage: pick and rotate the ROM bytes, keep one for the final rounds */
static uint32_t aes_mix(uint32_t ins, uint32_t word, uint32_t rs1)
{
    static const uint8_t remap[4][4] = { { 2, 0, 0, 1 }, { 0, 0, 0, 0 }, { 3, 2, 1, 0 }, { 3, 3, 3, 3 } };
    const uint8_t *out = remap[((ins >> 26) & 2) | (~ins >> 26 & 1)];
    uint32_t bs = ins >> 30, r = 0, j;
    for (j = 0; j < 4; j++) {
        uint32_t v = (word >> (8 * out[(bs + 3 - j) & 3])) & 0xFF;
        if (((ins >> 26) & 1) || j == bs)
            r |= v << (8 * j);
    }
    return r ^ rs1;
}

/* look at a halfword the fetch port already holds a translation for, without a refill */
static int peek16(bao_t *b, uint32_t va, uint32_t *h)
{
    if (b->vm_i) {
        const tlb_t *t;
        int i = tlb_lookup(b, 0, va);
        if (i < 0)
            return -1;
        t = &b->tlb[0][i];
        if (t->exc || !t->x)
            return -1;
        va = ((uint32_t)t->ppn1 << 22) | ((t->super ? (va >> 12) & 0x3FFu : t->ppn0) << 12) |
             (va & 0xFFFu);
    }
    return fetch16(b, va, h);
}

/* the ROM is read every cycle, so a held AES takes the next instruction's word */
static uint32_t aes32(bao_t *b, uint32_t ins, uint32_t pc, uint32_t rs1, uint32_t rs2, uint64_t at)
{
    uint32_t e = ins, v = rs2, lo, hi;
    if (b->wb_hold && at == b->wb_hold && !peek16(b, pc + 4, &lo)) {
        if ((lo & 3) != 3)
            e = bao_expand_c(lo);
        else if (!peek16(b, pc + 6, &hi))
            e = lo | hi << 16;
        else
            e = 0;
        if (e) {
            uint32_t r = RS2(e);
            v = r == b->hold_rd ? b->hold_old : b->x[r];
        } else {
            e = ins;
        }
    }
    return aes_mix(ins, aes_rom(aes_rom_addr(e, v)), rs1);
}

static uint32_t amo(uint32_t f5, uint32_t a, uint32_t b)
{
    switch (f5) {
    case 0x00: return a + b;
    case 0x01: return b;
    case 0x04: return a ^ b;
    case 0x0C: return a & b;
    case 0x08: return a | b;
    case 0x10: return (int32_t)a < (int32_t)b ? a : b;
    case 0x14: return (int32_t)a > (int32_t)b ? a : b;
    case 0x18: return a < b ? a : b;
    case 0x1C: return a > b ? a : b;
    default: return a;
    }
}

#define WR(r, v) do { uint32_t _v = (v); if (r) x[r] = _v; } while (0)
#define TRAP(c, t) do { take_trap(b, (c), (t)); bub = JUMP_TRAP; goto next; } while (0)

/* wait in decode for a source register still in flight */
#define USE(r) do { uint32_t r_ = (r);                                         \
        if (r_ == b->hz_rd[0] && b->hz_ready[0] > at) at = b->hz_ready[0];     \
        if (r_ == b->hz_rd[1] && b->hz_ready[1] > at) at = b->hz_ready[1];     \
    } while (0)
/* rd is ready l slots after this instruction leaves decode */
#define LATE(r, l) do { if (r) { wrd = (r); lat = (l); } } while (0)
/* a load or AMO that held writeback: note its slot and the rd it replaces */
#define HELD(r) do { if (b->wb_hold == 1) { b->wb_hold = at + 1;               \
            b->hold_rd = (uint8_t)(r); b->hold_old = x[r]; } } while (0)
/* count the stall before a side effect, so timers and mcycle see it */
#define GO() do { b->cycles += at - b->slot; b->slot = at; } while (0)

#define LOOP_NAME  step_bare
#define LOOP_PAGED 0
#define LOOP_LOAD  load_bare
#define LOOP_STORE store_bare
#include "cpu_loop.h"
#undef LOOP_NAME
#undef LOOP_PAGED
#undef LOOP_LOAD
#undef LOOP_STORE

#define LOOP_NAME  step_paged
#define LOOP_PAGED 1
#define LOOP_LOAD  load
#define LOOP_STORE store
#include "cpu_loop.h"

int cpu_step_block(bao_t *b)
{
    if (b->vm_i | b->vm_d)
        return step_paged(b);
    return step_bare(b);
}
