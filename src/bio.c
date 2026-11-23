/*
 * bio.c: BIO coprocessor model (four PicoRV32 cores)
 *
 * Copyright (c) 2026 Armstrong Subero
 * SPDX-License-Identifier: Apache-2.0
 *
 * Each core is stepped one aclk cycle at a time through the PicoRV32 state
 * machine as bio_bdma.sv configures it. x16 to x31 are the BIO extensions
 * (FIFOs, quantum halt, masked GPIO, events, core id and clock count).
 */

#include <stdlib.h>
#include <string.h>
#include "bao_int.h"

#define BIO_BASE   0x50124000u
#define IMEM_BASE  0x50125000u
#define ALIAS_BASE 0x50129000u
#define TNEVER     (~(uint64_t)0)
#define HIST       512

/* BIO DMA completion latency in aclk after grant, fitted to the Dabao */
#define DMA_RD_CYCLES 3
#define DMA_WR_CYCLES 4

enum { S_FETCH, S_LDRS1, S_EXEC, S_LDMEM, S_STMEM, S_TRAP };
enum { I_ILL, I_LUI, I_AUIPC, I_JAL, I_JALR, I_BR, I_LD, I_ST, I_ALUI, I_SHI, I_ALU, I_MUL, I_FENCE, I_ECALL };

typedef struct {
    int     cls, compr, rd, rs1, rs2, f3, alt, noren2;
    int32_t imm;
} dec_t;

/* pio_divider state during cycle t0; later cycles follow in closed form */
typedef struct {
    uint64_t t0;
    uint32_t c, div;
    int      pen, old, pe;
} qdiv_t;

/* regfifo: q is the storage (level0), r1 and dat1 the output register */
typedef struct {
    uint32_t q[8], dat1, awd;
    int      qh, qn, r1, are, awe, aprio, lvl_prev;
    uint64_t ft, at, lvl_t;
    uint64_t hold;   /* held in reset by a_restart: writes before this cycle are lost */
} fifo_t;

typedef struct {
    uint32_t x[16];
    int      en, blocked, st, trap;
    uint64_t t, en_at, last_l;
    /* en_sync is high for cycles on_from <= t < on_until; a restart holds the
     * core in reset from rst_at up to rst_end (bio_bdma.sv 447, 1842) */
    uint64_t on_from, on_until, rst_at, rst_end;
    uint32_t last_lpc;
    uint32_t pc, npc, op1, op2, out, aluq;
    int      lstore, lbranch, lstalu, lcompr, lrd, lis, dtrig;
    dec_t    dec, cur;
    int      mstate, mvalid, mwsize, mwstrb, do_pf, do_ri, do_rd, do_wd;
    uint32_t maddr, mwdata, rdq, ramq, pbufh;
    int      secondword, fwreg, lastmv, pbuf, clrq;
    int      mvd, dead, ext;
    int      pvalid, mul0, mul1, pcnt, ptimeout;
    uint32_t mrs1, mrs2, mrd;
    uint32_t mask, evmask;
} bcore_t;

typedef struct { uint64_t t; uint32_t v; } hent_t;

typedef struct {
    bcore_t  c[4];
    uint8_t  ram[4][4096];
    qdiv_t   qd[4];
    fifo_t   f[4];
    uint32_t regs[64];
    uint32_t alias_txf[4], alias_set[4], alias_clr[4];
    uint32_t ev, eset, eclr;
    uint64_t eat;
    int      esetp, eclrp;
    uint32_t agg_o, agg_e, pad_o, pad_e, gs, gc, ds, dc;
    uint64_t gat, pad_due;
    uint32_t snap_v;
    uint64_t snap_p;
    hent_t   hist[HIST];
    int      hn, hh;
    int      dma_busy, dma_owner;
    uint64_t dma_ready, dma_free;
    uint32_t dma_data;
    int      irq_lines, aggq, busy, gwarned;
    uint32_t haddr;   /* word address bits 11:2 of the last host APB access */
    uint64_t apb_free; /* CPU cycle of the pclk edge that ended the last host access */
    uint64_t now, fbase, cbase;
    int      mul;
} bio_t;

uint32_t bao_expand_c(uint32_t c);

static uint32_t reg(bio_t *o, uint32_t off) { return o->regs[(off & 0xFF) >> 2]; }

/* Time */

static uint64_t fclk_now(bao_t *b, bio_t *o)
{
    return o->fbase + (b->cycles - o->cbase) * (uint64_t)o->mul;
}

static uint64_t cpu_at(bio_t *o, uint64_t t)
{
    return t <= o->fbase ? o->cbase : o->cbase + (t - o->fbase) / (uint64_t)o->mul;
}

/* Pins */

static uint32_t pins_now(bao_t *b, bio_t *o)
{
    uint32_t po = o->pad_o ^ reg(o, 0x68), poe = o->pad_e ^ reg(o, 0x64), sel = b->iox[0x200 >> 2], v = 0;
    int i;
    for (i = 0; i < 32; i++) {
        int port = i < 16 ? 1 : 2, lvl;
        uint32_t bit = 1u << (i & 15);
        if ((sel >> i) & 1 && (poe >> i) & 1)
            lvl = (int)((po >> i) & 1);
        else if (b->iox[(0x148 >> 2) + port] & bit)
            lvl = (b->iox[(0x130 >> 2) + port] & bit) != 0;
        else if (b->gpio_ext_set[port] & bit)
            lvl = (b->gpio_ext[port] & bit) != 0;
        else
            lvl = (b->iox[(0x160 >> 2) + port] & bit) != 0;
        v |= (uint32_t)lvl << i;
    }
    return v;
}

static hent_t *hist_at(bio_t *o, int i) { return &o->hist[(o->hh + i) % HIST]; }

static void rec_pins(bao_t *b, bio_t *o, uint64_t t)
{
    uint32_t v = pins_now(b, o);
    hent_t *last = hist_at(o, o->hn - 1);
    if (last->v == v)
        return;
    if (last->t >= t) {
        last->v = v;
        return;
    }
    if (o->hn == HIST) {
        o->hh = (o->hh + 1) % HIST;
        o->hn--;
    }
    hist_at(o, o->hn)->t = t;
    hist_at(o, o->hn)->v = v;
    o->hn++;
}

/* pad input (before in_invert) during cycle t */
static uint32_t pin_at(bio_t *o, uint64_t t)
{
    int i;
    for (i = o->hn - 1; i > 0; i--)
        if (hist_at(o, i)->t <= t)
            return hist_at(o, i)->v;
    return hist_at(o, 0)->v;
}

/* pad round trip delay in FCLK, fitted to the Dabao and applied on the input side */
#define PAD_LAG_700 5

static uint64_t pad_lag(bio_t *o) { return ((uint64_t)PAD_LAG_700 * (uint64_t)o->mul + 1) / 2; }

/* pad input as gpio_in sees it during cycle t */
static uint32_t pad_in(bio_t *o, uint64_t t)
{
    uint64_t l = pad_lag(o);
    return pin_at(o, t >= l ? t - l : 0);
}

/* gpio_in_cleaned: two synchronizer stages, one with sync_bypass (bio_bdma.sv 879 to 887) */
static uint32_t cleaned(bio_t *o, uint64_t t)
{
    uint32_t byp = reg(o, 0x60);
    uint32_t a = pad_in(o, t >= 1 ? t - 1 : 0), c = pad_in(o, t >= 2 ? t - 2 : 0);
    return ((a & byp) | (c & ~byp)) ^ reg(o, 0x6C);
}

/* first rise of a pin (after in_invert) at or after cycle from */
static uint64_t next_rise(bio_t *o, int pin, uint64_t from)
{
    uint32_t inv = reg(o, 0x6C);
    int i, prev = (int)(((hist_at(o, 0)->v ^ inv) >> pin) & 1);
    for (i = 1; i < o->hn; i++) {
        hent_t *e = hist_at(o, i);
        int now = (int)(((e->v ^ inv) >> pin) & 1);
        if (e->t >= from && now && !prev)
            return e->t;
        prev = now;
    }
    return TNEVER;
}

/* Quantum dividers */

static uint32_t qd_cnt(const qdiv_t *q, uint64_t n)
{
    uint64_t k = n - q->t0;
    if (!q->div)
        return q->c;
    if (q->div >= 256)
        return (uint32_t)(((uint64_t)q->c + 256u * (k % q->div)) % q->div);
    return (uint32_t)(((uint64_t)q->c + 256u * (k & 0xFFFF)) & 0xFFFFFF);
}

static int qd_pen(const qdiv_t *q, uint64_t n)
{
    if (n <= q->t0 || !q->div)
        return q->pen;
    return qd_cnt(q, n - 1) < (q->div >> 1);
}

static int qd_old(const qdiv_t *q, uint64_t n)
{
    if (n <= q->t0 || !q->div)
        return q->old;
    return qd_pen(q, n - 1);
}

static int qd_pe(const qdiv_t *q, uint64_t n)
{
    int d1 = (q->div >> 8) == 1;
    if (n <= q->t0)
        return q->pe;
    if (!q->div)
        return 1;
    return (qd_pen(q, n - 1) && !qd_old(q, n - 1)) ^ d1;
}

/* the j th counter wrap puts a penable pulse at t0 + ceil((j M - c) / 256) + 2 */
static uint64_t qd_wrap(const qdiv_t *q, uint64_t j)
{
    uint64_t m = q->div >= 256 ? q->div : 0x1000000u;
    return q->t0 + (j * m - q->c + 255) / 256 + 2;
}

/* with no divider, or div_int 1, penable is high nearly every cycle */
static int qd_dense(const qdiv_t *q) { return !q->div || (q->div >> 8) == 1; }

/* first penable pulse at or after cycle t */
static uint64_t qd_next(const qdiv_t *q, uint64_t t)
{
    uint64_t n, m, j;
    int i;
    if (t < q->t0)
        t = q->t0;
    for (n = t; n < t + 64; n++) {
        if (qd_pe(q, n))
            return n;
        if (!qd_dense(q) && n >= t + 7)
            break;
    }
    if (qd_dense(q))
        return TNEVER;
    n++;
    m = q->div >= 256 ? q->div : 0x1000000u;
    j = (256u * (n - q->t0 - 3) + q->c) / m + 1;
    for (i = 0; i < 4; i++, j++) {
        uint64_t cand = qd_wrap(q, j);
        if (cand >= n && qd_pe(q, cand))
            return cand;
    }
    return TNEVER;
}

/* last penable pulse at or before cycle x */
static uint64_t qd_prev(const qdiv_t *q, uint64_t x)
{
    uint64_t n, m, j;
    int i;
    if (x < q->t0)
        return TNEVER;
    for (n = x;; n--) {
        if (qd_pe(q, n))
            return n;
        if (n == q->t0)
            return TNEVER;
        if (x - n >= (qd_dense(q) ? 63u : 7u) && n - q->t0 >= 16)
            break;
    }
    if (qd_dense(q))
        return TNEVER;
    n--;
    m = q->div >= 256 ? q->div : 0x1000000u;
    j = (256u * (n - q->t0 - 2) + q->c) / m;
    for (i = 0; i < 4 && j >= 1; i++, j--) {
        uint64_t cand = qd_wrap(q, j);
        if (cand <= n && qd_pe(q, cand))
            return cand;
    }
    for (n = q->t0 + 15;; n--) {
        if (qd_pe(q, n))
            return n;
        if (n == q->t0)
            return TNEVER;
    }
}

static void qd_reset(qdiv_t *q, uint64_t t, uint32_t div)
{
    q->t0 = t + 1;
    q->c = 0;
    q->pen = 1;
    q->old = 0;
    q->pe = 0;
    q->div = div;
}

/* a new divider value carries the counter on from where it is (pio_divider.v) */
static void qd_rebase(qdiv_t *q, uint64_t t, uint32_t div)
{
    int steps = 0;
    if (t > q->t0) {
        uint32_t c = qd_cnt(q, t);
        int pen = qd_pen(q, t), old = qd_old(q, t), pe = qd_pe(q, t);
        q->c = c; q->pen = pen; q->old = old; q->pe = pe; q->t0 = t;
    }
    q->div = div;
    if (div == 256)
        q->c %= 256;
    while (div > 256 && q->c >= div && steps++ < 65536) {
        int d1 = (div >> 8) == 1, pen = q->pen;
        q->pe = (q->pen && !q->old) ^ d1;
        q->old = pen;
        q->pen = q->c < (div >> 1);
        q->c = q->c >= div - 256 ? q->c + 256 - div : (q->c + 256) & 0xFFFFFF;
        q->t0++;
    }
    if (div > 256)
        q->c %= div;
}

static int ext_on(bio_t *o, int c) { return (int)((reg(o, 0x44) >> c) & 1); }
static int ext_pin(bio_t *o, int c) { return (int)((reg(o, 0x44) >> (4 + 5 * c)) & 0x1F); }

/* quantum is penable registered once, or a rise of gpio_in_sync1 (bio_bdma.sv 706) */
static int quantum(bio_t *o, int c, uint64_t t)
{
    if (ext_on(o, c))
    {
        uint64_t d = 3 + pad_lag(o);
        return t >= d && next_rise(o, ext_pin(o, c), t - d) == t - d;
    }
    return t >= 1 && qd_pe(&o->qd[c], t - 1);
}

static uint64_t next_quantum(bio_t *o, int c, uint64_t t)
{
    uint64_t n;
    if (ext_on(o, c)) {
        uint64_t d = 3 + pad_lag(o);
        n = next_rise(o, ext_pin(o, c), t >= d ? t - d : 0);
        return n == TNEVER ? n : n + d;
    }
    n = qd_next(&o->qd[c], t - 1);
    return n == TNEVER ? n : n + 1;
}

/* gpio_in_snapped loads on penable of the chosen core (bio_bdma.sv 1010) */
static uint32_t snapped(bio_t *o, uint64_t t)
{
    int w = (int)((reg(o, 0x08) >> 3) & 3);
    uint64_t p = t >= 1 ? qd_prev(&o->qd[w], t - 1) : TNEVER;
    if (p != TNEVER && p > o->snap_p) {
        o->snap_v = cleaned(o, p);
        o->snap_p = p;
    }
    return o->snap_v;
}

static uint32_t pins_read(bio_t *o, uint64_t t)
{
    uint32_t s = snapped(o, t);
    return (reg(o, 0x08) >> 5) & 1 ? s : cleaned(o, t);
}

/* FIFOs */

static void fifo_level_set(fifo_t *f, int before, uint64_t t)
{
    if (f->qn + f->r1 != before) {
        f->lvl_prev = before;
        f->lvl_t = t;
    }
}

static void fifo_adv(fifo_t *f, uint64_t t)
{
    while (f->ft < t) {
        uint64_t n = f->ft;
        int before = f->qn + f->r1;
        if (f->at == n) {
            int q0 = f->qn, sre = f->qn > 0 && (!f->r1 || f->are);
            if (sre) {
                f->dat1 = f->q[f->qh];
                f->qh = (f->qh + 1) & 7;
                f->qn--;
                f->r1 = 1;
            } else if (f->are) {
                f->r1 = 0;
            }
            if (f->awe && q0 != 8) {
                f->q[(f->qh + f->qn) & 7] = f->awd;
                f->qn++;
            }
            f->at = TNEVER;
        } else if (!f->r1 && f->qn) {
            f->dat1 = f->q[f->qh];
            f->qh = (f->qh + 1) & 7;
            f->qn--;
            f->r1 = 1;
        } else {
            f->ft = (f->at != TNEVER && f->at < t) ? f->at : t;
            continue;
        }
        fifo_level_set(f, before, n + 1);
        f->ft = n + 1;
    }
}

static void fifo_acc(fifo_t *f, uint64_t t)
{
    fifo_adv(f, t);
    if (f->at != t) {
        f->at = t;
        f->are = f->awe = 0;
        f->aprio = 99;
    }
}

static void fifo_pop(fifo_t *f, uint64_t t)
{
    fifo_acc(f, t);
    f->are = 1;
}

/* one write lands per cycle: host first, then core 0 to 3 (bio_bdma.sv 713) */
static void fifo_push(fifo_t *f, uint64_t t, uint32_t v, int prio)
{
    if (t < f->hold)
        return;
    fifo_acc(f, t);
    if (!f->awe || prio < f->aprio) {
        f->awe = 1;
        f->awd = v;
        f->aprio = prio;
    }
}

static void fifo_clear(fifo_t *f, uint64_t t)
{
    int before;
    fifo_adv(f, t);
    before = f->qn + f->r1;
    f->qn = f->qh = f->r1 = 0;
    f->at = TNEVER;
    fifo_level_set(f, before, t);
}

static int fifo_level_at(fifo_t *f, uint64_t t)
{
    return f->lvl_t <= t ? f->qn + f->r1 : f->lvl_prev;
}

/* Events */

static void ev_adv(bio_t *o, uint64_t t)
{
    if (o->eat != TNEVER && o->eat < t) {
        uint32_t s = o->esetp < 99 ? o->eset : 0, c = o->eclrp < 99 ? o->eclr : 0;
        o->ev = (o->ev & ~(c & ~s)) | (s & ~c);
        o->eat = TNEVER;
    }
}

/* the set and clear aggregators each take one source per cycle (bio_bdma.sv 834) */
static void ev_op(bio_t *o, uint64_t t, uint32_t v, int set, int prio)
{
    ev_adv(o, t);
    if (o->eat != t) {
        o->eat = t;
        o->eset = o->eclr = 0;
        o->esetp = o->eclrp = 99;
    }
    if (set) {
        if (prio < o->esetp) { o->eset = v; o->esetp = prio; }
        else if (prio == 0 && o->esetp == 0) o->eset |= v;
    } else {
        if (prio < o->eclrp) { o->eclr = v; o->eclrp = prio; }
        else if (prio == 0 && o->eclrp == 0) o->eclr |= v;
    }
}

/* FIFO level events are registered from the level of the cycle before (bio_bdma.sv 798) */
static uint32_t fifo_flags(bio_t *o, uint64_t t)
{
    uint32_t f = 0, lvl = reg(o, 0x30), ty = reg(o, 0x34);
    int i;
    for (i = 0; i < 8; i++) {
        fifo_t *q = &o->f[i >> 1];
        uint32_t l, e = (lvl >> (4 * i)) & 0xF;
        fifo_adv(q, t);
        l = (uint32_t)fifo_level_at(q, t >= 1 ? t - 1 : 0);
        if (((ty >> i) & 1 && l < e) || ((ty >> (8 + i)) & 1 && l == e) || ((ty >> (16 + i)) & 1 && l > e))
            f |= 1u << (24 + i);
    }
    return f;
}

static uint32_t ev_all(bio_t *o, uint64_t t)
{
    ev_adv(o, t);
    return (o->ev & 0xFFFFFF) | fifo_flags(o, t);
}

static int ev_soon(bio_t *o, uint64_t t)
{
    int i;
    if (o->eat != TNEVER)
        return 1;
    for (i = 0; i < 4; i++) {
        fifo_t *f = &o->f[i];
        if (f->at != TNEVER || (!f->r1 && f->qn) || f->lvl_t >= t)
            return 1;
    }
    return 0;
}

/* Wake and RAM */

static uint32_t next_pc_of(const bcore_t *k)
{
    return (k->lstore && k->lbranch) ? (k->out & ~1u) : k->npc;
}

static int qhalt_of(const bcore_t *k)
{
    int wen = k->st == S_FETCH && (k->lbranch || k->lstore) && k->lrd;
    return k->dec.rs1 == 20 || k->dec.rs2 == 20 || (wen && k->lrd == 20);
}

/* host owns the RAM port while en_sync is low (bio_bdma.sv 1728, 1738 to 1763) */
static void ram_tick(bio_t *o, int c, bcore_t *k, int qh, int la_ext, int host)
{
    int ready = k->mvd && !k->dead && k->mvalid;
    int mvd = (k->mvalid && !k->ext) || (qh && k->mvd);
    if (host && k->mvalid) {
        uint8_t *m = o->ram[c] + 4u * (o->haddr & 0x3FF);
        k->ramq = (uint32_t)m[0] | ((uint32_t)m[1] << 8) | ((uint32_t)m[2] << 16) | ((uint32_t)m[3] << 24);
    } else if (!host && k->mvalid && !k->ext) {
        uint8_t *m = o->ram[c] + (k->maddr & 0xFFC);
        int i;
        k->ramq = (uint32_t)m[0] | ((uint32_t)m[1] << 8) | ((uint32_t)m[2] << 16) | ((uint32_t)m[3] << 24);
        if (k->mwstrb)
            for (i = 0; i < 4; i++)
                if ((k->mwstrb >> i) & 1)
                    m[i] = (uint8_t)(k->mwdata >> (8 * i));
    }
    k->dead = k->mvd && ready;
    k->mvd = mvd;
    k->ext = la_ext;
}

static void ram_skip(bio_t *o, int c, uint64_t from, uint64_t to, int host)
{
    bcore_t *k = &o->c[c];
    uint64_t n;
    int qh = qhalt_of(k), la_ext;
    uint32_t la = (k->do_pf || k->do_ri) ? next_pc_of(k) : k->op1;
    if (to <= from)
        return;
    la_ext = la >= 0x10000000u;
    n = to - from;
    if (n > 5)
        n = 4 + ((n - 4) & 1);
    while (n--)
        ram_tick(o, c, k, qh, la_ext, host);
}

static int en_sync_at(const bcore_t *k, uint64_t t) { return t >= k->on_from && t < k->on_until; }

/* run the RAM port of a core that is not stepped from k->t up to cycle to,
 * as the host's port where en_sync is low */
static void ram_catch(bio_t *o, int c, uint64_t to)
{
    bcore_t *k = &o->c[c];
    while (k->t < to) {
        uint64_t e;
        int host = !en_sync_at(k, k->t);
        if (host)
            e = k->t < k->on_from && k->on_from < k->on_until ? k->on_from : TNEVER;
        else
            e = k->on_until;
        if (e > to)
            e = to;
        ram_skip(o, c, k->t, e, host);
        k->t = e;
    }
}

static void wake_all(bio_t *o, uint64_t t)
{
    int c;
    for (c = 0; c < 4; c++) {
        bcore_t *k = &o->c[c];
        if (k->blocked) {
            uint64_t e = t < k->on_until ? t : k->on_until;
            k->blocked = 0;
            if (e > k->t) {
                ram_skip(o, c, k->t, e, 0);
                k->t = e;
            }
        }
    }
}

/* GPIO */

static void pad_apply(bao_t *b, bio_t *o, uint64_t d)
{
    uint64_t save = b->cycles, at = cpu_at(o, d);
    b->bio_out = o->pad_o ^ reg(o, 0x68);
    b->bio_oe = o->pad_e ^ reg(o, 0x64);
    if (at < save)
        b->cycles = at;
    gpio_notify(b);
    b->cycles = save;
    rec_pins(b, o, d);
    wake_all(o, d);
}

static uint64_t pad_when(bio_t *o, uint64_t a)
{
    uint64_t p;
    if (!((reg(o, 0x08) >> 2) & 1))
        return a + 1;
    p = qd_next(&o->qd[reg(o, 0x08) & 3], a);
    return p == TNEVER ? p : p + 1;
}

/* gpio_out_aclk is an scc_ff of the ORed core strobes; the pads follow a
 * cycle later, or on penable of the snap core (bio_bdma.sv 941 to 1009) */
static void gpio_advance(bao_t *b, bio_t *o, uint64_t t)
{
    for (;;) {
        uint64_t na = o->gat != TNEVER ? o->gat + 1 : TNEVER;
        if (o->pad_due != TNEVER && o->pad_due <= t && o->pad_due <= na) {
            uint64_t d = o->pad_due;
            o->pad_due = TNEVER;
            if (o->pad_o != o->agg_o || o->pad_e != o->agg_e) {
                o->pad_o = o->agg_o;
                o->pad_e = o->agg_e;
                pad_apply(b, o, d);
            }
            continue;
        }
        if (na != TNEVER && na <= t) {
            uint32_t ao = o->agg_o, ae = o->agg_e;
            o->agg_o = (o->agg_o & ~(o->gc & ~o->gs)) | (o->gs & ~o->gc);
            o->agg_e = (o->agg_e & ~(o->dc & ~o->ds)) | (o->ds & ~o->dc);
            o->gat = TNEVER;
            if (o->agg_o != ao || o->agg_e != ae)
                o->pad_due = pad_when(o, na);
            continue;
        }
        break;
    }
}

static void gpio_op(bio_t *o, uint64_t t, uint32_t s, uint32_t c, uint32_t ds, uint32_t dc)
{
    if (o->gat != t) {
        o->gat = t;
        o->gs = o->gc = o->ds = o->dc = 0;
    }
    o->gs |= s;
    o->gc |= c;
    o->ds |= ds;
    o->dc |= dc;
}

/* DMA */

static int filter_ok(bio_t *o, uint32_t a)
{
    int w;
    if (reg(o, 0x08) & 0x80)
        return 1;
    for (w = 0; w < 4; w++) {
        uint32_t base = reg(o, 0xE0 + 8u * (uint32_t)w) & 0xFFFFF;
        uint32_t len = reg(o, 0xE4 + 8u * (uint32_t)w) & 0xFFFFF;
        uint32_t bound = base + len > 0xFFFFF ? 0xFFFFF : base + len;
        if ((a >> 12) >= base && (a >> 12) < bound)
            return 1;
    }
    return 0;
}

static uint32_t dma_rd32(bao_t *b, uint32_t a)
{
    uint32_t v = 0;
    a &= ~3u;
    if (b->rdmap[a >> 16]) {
        uint8_t *p = b->rdmap[a >> 16] + (a & 0xFFFF);
        return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
    }
    if (bus_read(b, a, 4, &v))
        v = 0;
    return v;
}

static void dma_wr(bao_t *b, uint32_t a, int size, uint32_t v)
{
    b->side++;
    if (b->wrmap[a >> 16]) {
        uint8_t *p = b->wrmap[a >> 16] + (a & 0xFFFF);
        int i;
        for (i = 0; i < size; i++)
            p[i] = (uint8_t)(v >> (8 * i));
        return;
    }
    bus_write(b, a, size, v);
}

/* crossbar, axil_filter and the AHB size bodge (bio_bdma.sv 1195 to 1521, 2470) */
static uint32_t dma_access(bao_t *b, bio_t *o, int c, uint32_t a, int wstrb, uint32_t wd)
{
    int peri;
    uint32_t g;
    if (a >= 0x40000000u && a < 0x60000000u)
        peri = 1;
    else if (a >= 0x60000000u && a < 0x80000000u)
        peri = 0;
    else
        return 0;
    if (!filter_ok(o, a)) {
        g = reg(o, peri ? 0xA4 : 0xA0);
        if (!((o->gwarned >> c) & 1))
            bao_logf(b, BAO_LOG_WARN, "BIO core %d access 0x%08X outside its DMA windows goes to the gutter 0x%08X", c, a, g);
        o->gwarned |= 1 << c;
        a = g;
        if (a < 0x40000000u || a >= 0x80000000u)
            return 0;
    }
    if (!wstrb)
        return dma_rd32(b, a);
    if (peri) {
        switch (wstrb) {
        case 1: dma_wr(b, a & ~3u, 1, wd); break;
        case 2: dma_wr(b, (a & ~3u) + 1, 1, wd >> 8); break;
        case 4: dma_wr(b, (a & ~3u) + 2, 1, wd >> 16); break;
        case 8: dma_wr(b, (a & ~3u) + 3, 1, wd >> 24); break;
        case 3: dma_wr(b, a & ~3u, 2, wd); break;
        case 12: dma_wr(b, (a & ~3u) + 2, 2, wd >> 16); break;
        default: dma_wr(b, a, 4, wd); break;
        }
    } else {
        int i;
        if (wstrb == 0xF)
            dma_wr(b, a & ~3u, 4, wd);
        else
            for (i = 0; i < 4; i++)
                if ((wstrb >> i) & 1)
                    dma_wr(b, (a & ~3u) + (uint32_t)i, 1, wd >> (8 * i));
    }
    return 0;
}

/* Decode */

static int32_t imm_i(uint32_t i) { return (int32_t)i >> 20; }
static int32_t imm_s(uint32_t i) { return ((int32_t)(i & 0xFE000000u) >> 20) | (int32_t)((i >> 7) & 31); }
static int32_t imm_b(uint32_t i)
{
    return (int32_t)(((((i >> 31) & 1) << 12) | (((i >> 7) & 1) << 11) | (((i >> 25) & 0x3F) << 5) |
                      (((i >> 8) & 0xF) << 1)) << 19) >> 19;
}
static int32_t imm_j(uint32_t i)
{
    return (int32_t)(((((i >> 31) & 1) << 20) | (((i >> 21) & 0x3FF) << 1) | (((i >> 20) & 1) << 11) |
                      (((i >> 12) & 0xFF) << 12)) << 11) >> 11;
}

/* picorv32.v 921 to 1188: decoded_rs1 and rs2 come from bits 19:15 and 24:20 of
 * every 32 bit instruction, immediates included */
static dec_t decode32(uint32_t i)
{
    dec_t d;
    uint32_t op = i & 0x7F, f7 = i >> 25;
    memset(&d, 0, sizeof d);
    d.rd = (int)((i >> 7) & 31);
    d.rs1 = (int)((i >> 15) & 31);
    d.rs2 = (int)((i >> 20) & 31);
    d.f3 = (int)((i >> 12) & 7);
    d.alt = (int)((i >> 30) & 1);
    d.cls = I_ILL;
    switch (op) {
    case 0x37: d.cls = I_LUI; d.imm = (int32_t)(i & 0xFFFFF000u); d.noren2 = 1; break;
    case 0x17: d.cls = I_AUIPC; d.imm = (int32_t)(i & 0xFFFFF000u); d.noren2 = 1; break;
    case 0x6F: d.cls = I_JAL; d.imm = imm_j(i); d.noren2 = 1; break;
    case 0x67:
        d.imm = imm_i(i);
        if (d.f3 == 0) { d.cls = I_JALR; d.noren2 = 1; }
        break;
    case 0x63:
        d.imm = imm_b(i);
        if (d.f3 != 2 && d.f3 != 3) d.cls = I_BR;
        break;
    case 0x03:
        d.imm = imm_i(i);
        d.noren2 = 1;
        if (d.f3 != 3 && d.f3 < 6) d.cls = I_LD;
        break;
    case 0x23:
        d.imm = imm_s(i);
        if (d.f3 <= 2) d.cls = I_ST;
        break;
    case 0x13:
        d.imm = imm_i(i);
        if (d.f3 == 1) {
            if (f7 == 0) { d.cls = I_SHI; d.noren2 = 1; }
        } else if (d.f3 == 5) {
            if (f7 == 0 || f7 == 0x20) { d.cls = I_SHI; d.noren2 = 1; }
        } else {
            d.cls = I_ALUI;
            d.noren2 = 1;
        }
        break;
    case 0x33:
        if (f7 == 0) d.cls = I_ALU;
        else if (f7 == 0x20) { if (d.f3 == 0 || d.f3 == 5) d.cls = I_ALU; }
        else if (f7 == 1) { if (d.f3 < 4) d.cls = I_MUL; }
        break;
    case 0x0F:
        if (d.f3 == 0) d.cls = I_FENCE;
        break;
    case 0x73:
        if ((i >> 21) == 0 && ((i >> 7) & 0x1FFF) == 0) d.cls = I_ECALL;
        break;
    default:
        break;
    }
    return d;
}

/* compressed decode with the register fields PicoRV32 latches for each form */
static dec_t decode16(uint32_t c)
{
    dec_t d, e;
    uint32_t q = c & 3, f = (c >> 13) & 7, r = (c >> 7) & 31, r2 = (c >> 2) & 31;
    int rp = (int)(8 + ((c >> 7) & 7)), r2p = (int)(8 + ((c >> 2) & 7)), cls = I_ILL, nr2 = 0;
    uint32_t x;
    memset(&d, 0, sizeof d);
    if (q == 0) {
        if (f == 0) { d.rs1 = 2; d.rd = r2p; if ((c >> 5) & 0xFF) { cls = I_ALUI; nr2 = 1; } }
        else if (f == 2) { d.rs1 = rp; d.rd = r2p; cls = I_LD; nr2 = 1; }
        else if (f == 6) { d.rs1 = rp; d.rs2 = r2p; cls = I_ST; }
    } else if (q == 1) {
        switch (f) {
        case 0: d.rd = d.rs1 = (int)r; cls = I_ALUI; nr2 = 1; break;
        case 1: d.rd = 1; cls = I_JAL; nr2 = 1; break;
        case 2: d.rd = (int)r; cls = I_ALUI; nr2 = 1; break;
        case 3:
            if ((c >> 12) & 1 || r2) {
                if (r == 2) { d.rd = d.rs1 = 2; cls = I_ALUI; }
                else { d.rd = (int)r; cls = I_LUI; }
                nr2 = 1;
            }
            break;
        case 4:
            if (!((c >> 11) & 1) && !((c >> 12) & 1)) { d.rd = d.rs1 = rp; d.rs2 = (int)r2; cls = I_SHI; nr2 = 1; }
            if (((c >> 10) & 3) == 2) { d.rd = d.rs1 = rp; cls = I_ALUI; nr2 = 1; }
            if (((c >> 10) & 7) == 3) { d.rd = d.rs1 = rp; d.rs2 = r2p; cls = I_ALU; }
            break;
        case 5: cls = I_JAL; nr2 = 1; break;
        default: d.rs1 = rp; d.rs2 = 0; cls = I_BR; break;
        }
    } else if (q == 2) {
        if (f == 0) { if (!((c >> 12) & 1)) { d.rd = d.rs1 = (int)r; d.rs2 = (int)r2; cls = I_SHI; nr2 = 1; } }
        else if (f == 2) { if (r) { d.rd = (int)r; d.rs1 = 2; cls = I_LD; nr2 = 1; } }
        else if (f == 4) {
            int b12 = (int)((c >> 12) & 1);
            if (!b12 && r && !r2) { d.rd = 0; d.rs1 = (int)r; cls = I_JALR; nr2 = 1; }
            if (!b12 && r2) { d.rd = (int)r; d.rs1 = 0; d.rs2 = (int)r2; cls = I_ALU; }
            if (b12 && r && !r2) { d.rd = 1; d.rs1 = (int)r; cls = I_JALR; nr2 = 1; }
            if (b12 && r2) { d.rd = d.rs1 = (int)r; d.rs2 = (int)r2; cls = I_ALU; }
        } else if (f == 6) { d.rs1 = 2; d.rs2 = (int)r2; cls = I_ST; }
    }
    if (c == 0x9002)
        cls = I_ECALL;
    d.compr = 1;
    d.noren2 = nr2;
    if (cls != I_ILL && cls != I_ECALL) {
        x = bao_expand_c(c);
        if (!x) {
            cls = I_ILL;
        } else {
            e = decode32(x);
            d.f3 = e.f3;
            d.alt = e.alt;
            d.imm = e.imm;
        }
    }
    d.cls = cls;
    return d;
}

static dec_t decode(uint32_t lat)
{
    return (lat & 3) != 3 ? decode16(lat & 0xFFFF) : decode32(lat);
}

/* Execute helpers */

static uint32_t alu(const dec_t *d, uint32_t a, uint32_t b2)
{
    switch (d->cls) {
    case I_LUI: case I_AUIPC: case I_JALR:
        return a + b2;
    case I_ALUI: case I_SHI: case I_ALU:
        switch (d->f3) {
        case 0: return (d->cls == I_ALU && d->alt) ? a - b2 : a + b2;
        case 1: return a << (b2 & 31);
        case 2: return (int32_t)a < (int32_t)b2;
        case 3: return a < b2;
        case 4: return a ^ b2;
        case 5: return d->alt ? (uint32_t)((int32_t)a >> (b2 & 31)) : a >> (b2 & 31);
        case 6: return a | b2;
        default: return a & b2;
        }
    default:
        return 0;
    }
}

static int br_taken(int f3, uint32_t a, uint32_t b2)
{
    switch (f3) {
    case 0: return a == b2;
    case 1: return a != b2;
    case 4: return (int32_t)a < (int32_t)b2;
    case 5: return (int32_t)a >= (int32_t)b2;
    case 6: return a < b2;
    default: return a >= b2;
    }
}

static uint32_t mul_res(int f3, uint32_t a, uint32_t b2)
{
    uint64_t p = (uint64_t)a * b2;
    if ((f3 == 1 || f3 == 2) && (a >> 31))
        p -= (uint64_t)b2 << 32;
    if (f3 == 1 && (b2 >> 31))
        p -= (uint64_t)a << 32;
    return f3 ? (uint32_t)(p >> 32) : (uint32_t)p;
}

/* Core registers */

static uint32_t reg_read(bio_t *o, int c, int r, uint64_t t)
{
    bcore_t *k = &o->c[c];
    if (r < 16)
        return r ? k->x[r] : 0;
    switch (r) {
    case 16: case 17: case 18: case 19:
        fifo_adv(&o->f[r - 16], t);
        return o->f[r - 16].dat1;
    case 21: return pins_read(o, t);
    case 26: return k->mask;
    case 27: return k->evmask;
    case 30: return ev_all(o, t);
    case 31: return ((uint32_t)c << 30) | (uint32_t)((t - k->en_at) & 0x3FFFFFFF);
    default: return 0;
    }
}

/* picorv32_regs_bio write path (bio_bdma.sv 2108 to 2168) */
static void reg_write(bio_t *o, int c, bcore_t *n, int r, uint32_t v, uint64_t t)
{
    uint32_t m = o->c[c].mask;
    if (r < 16) {
        n->x[r] = v;
        return;
    }
    switch (r) {
    case 16: case 17: case 18: case 19:
        fifo_push(&o->f[r - 16], t, v, c + 1);
        wake_all(o, t + 1);
        break;
    case 21: gpio_op(o, t, m & v, m & ~v, 0, 0); break;
    case 22: gpio_op(o, t, m & v, 0, 0, 0); break;
    case 23: gpio_op(o, t, 0, m & ~v, 0, 0); break;
    case 24: gpio_op(o, t, 0, 0, m & v, 0); break;
    case 25: gpio_op(o, t, 0, 0, 0, m & v); break;
    case 26: n->mask = v; break;
    case 27: n->evmask = v; break;
    case 28: ev_op(o, t, v & 0xFFFFFF, 1, c + 1); wake_all(o, t + 1); break;
    case 29: ev_op(o, t, v & 0xFFFFFF, 0, c + 1); wake_all(o, t + 1); break;
    default: break;
    }
}

/* One core cycle */

static void core_cycle(bao_t *b, bio_t *o, int c, uint64_t t)
{
    bcore_t *k = &o->c[c], n = *k;
    const dec_t *d = &k->cur;
    uint32_t next_pc = next_pc_of(k), rdata, noshuf, lat, la_addr, wd, mword;
    int fw = (k->do_pf || k->do_ri) && (next_pc & 2) && !k->secondword;
    int clr = k->clrq, usepf, ready = 0, xfer, done, fwsel, la_read, la_write, wen, qh, dtrig, ws;
    int set_ri = 0, set_rd = 0, set_wd = 0;

    if (!k->pbuf) clr = 0;
    if (k->lbranch) clr = 1;
    usepf = fw && k->pbuf && !clr;

    switch (k->mwsize) {
    case 1: wd = (k->op2 & 0xFFFF) * 0x10001u; ws = (k->op1 & 2) ? 0xC : 0x3; break;
    case 2: wd = (k->op2 & 0xFF) * 0x01010101u; ws = 1 << (k->op1 & 3); break;
    default: wd = k->op2; ws = 0xF; break;
    }

    if (k->ext) {
        if (k->mvalid) {
            if (o->dma_busy && o->dma_owner == c) {
                if (t >= o->dma_ready) {
                    o->dma_data = dma_access(b, o, c, k->maddr, k->mwstrb, k->mwdata);
                    o->dma_busy = 0;
                    o->dma_free = t + 1;
                    ready = 1;
                }
            } else if (!o->dma_busy && t >= o->dma_free) {
                o->dma_busy = 1;
                o->dma_owner = c;
                o->dma_ready = t + (k->mwstrb ? DMA_WR_CYCLES : DMA_RD_CYCLES);
            }
        }
        rdata = o->dma_data;
    } else {
        rdata = k->ramq;
        ready = k->mvd && !k->dead && k->mvalid;
    }

    xfer = (k->mvalid && ready) || (usepf && k->do_ri);
    noshuf = xfer ? rdata : k->rdq;
    if (usepf) lat = k->pbufh;
    else if (k->secondword) lat = (noshuf << 16) | k->pbufh;
    else if (fw) lat = noshuf >> 16;
    else lat = noshuf;
    done = ((xfer && k->mstate && (k->do_ri || k->do_rd || k->do_wd)) || (k->mstate == 3 && k->do_ri)) &&
           (!fw || ((lat & 3) != 3 && xfer));
    fwsel = k->lastmv ? k->fwreg : fw;
    la_write = !k->mstate && k->do_wd;
    la_read = (!usepf && !k->mstate && (k->do_ri || k->do_pf || k->do_rd)) ||
              (xfer && fwsel && !k->secondword && (lat & 3) == 3);
    la_addr = (k->do_pf || k->do_ri) ? (((next_pc >> 2) + (uint32_t)(xfer && fwsel)) << 2) : (k->op1 & ~3u);
    switch (k->mwsize) {
    case 1: mword = (k->op1 & 2) ? rdata >> 16 : rdata & 0xFFFF; break;
    case 2: mword = (rdata >> (8 * (k->op1 & 3))) & 0xFF; break;
    default: mword = rdata; break;
    }

    wen = k->st == S_FETCH && (k->lbranch || k->lstore) && k->lrd;
    qh = qhalt_of(k);
    if (wen)
        reg_write(o, c, &n, k->lrd, k->lbranch ? k->pc + (k->lcompr ? 2u : 4u) : (k->lstalu ? k->aluq : k->out), t);

    dtrig = k->do_ri && done;
    switch (k->st) {
    case S_FETCH: {
        uint32_t cur_pc = k->npc;
        n.do_ri = !k->dtrig;
        n.mwsize = 0;
        if (k->lbranch)
            cur_pc = k->lstore ? ((k->lstalu ? k->aluq : k->out) & ~1u) : k->npc;
        n.pc = n.npc = cur_pc;
        n.lstore = n.lstalu = n.lbranch = 0;
        n.lis = 0;
        n.lrd = k->dec.rd;
        n.lcompr = k->dec.compr;
        if (k->dtrig) {
            n.cur = k->dec;
            n.npc = cur_pc + (k->dec.compr ? 2u : 4u);
            n.last_l = t;
            n.last_lpc = cur_pc;
            if (k->dec.cls == I_JAL) {
                n.do_ri = 1;
                n.npc = cur_pc + (uint32_t)k->dec.imm;
                n.lbranch = 1;
            } else {
                n.do_ri = 0;
                n.do_pf = k->dec.cls != I_JALR;
                n.st = S_LDRS1;
            }
        }
        break;
    }
    case S_LDRS1: {
        int r1 = k->dec.rs1, r2 = k->dec.rs2, ren2 = !d->noren2;
        uint32_t v1 = r1 ? reg_read(o, c, r1, t) : 0, v2 = r2 ? reg_read(o, c, r2, t) : 0;
        int pops = 0, f;
        if (r1 >= 16 && r1 <= 19) pops |= 1 << (r1 - 16);
        if (ren2 && r2 >= 16 && r2 <= 19) pops |= 1 << (r2 - 16);
        for (f = 0; f < 4; f++)
            if ((pops >> f) & 1)
                fifo_pop(&o->f[f], t);
        if (pops)
            wake_all(o, t + 1);
        if (d->cls == I_ILL || d->cls == I_MUL || d->cls == I_ECALL) {
            n.op1 = v1;
            n.op2 = v2;
            n.pvalid = 1;
            if (k->mul1) {
                n.do_ri = 1;
                n.pvalid = 0;
                n.out = k->mrd;
                n.lstore = 1;
                n.st = S_FETCH;
            } else if (k->ptimeout || d->cls == I_ECALL) {
                n.pvalid = 0;
                n.st = S_TRAP;
            }
        } else if (d->cls == I_LUI || d->cls == I_AUIPC) {
            n.op1 = d->cls == I_LUI ? 0 : k->pc;
            n.op2 = (uint32_t)d->imm;
            n.do_ri = k->do_pf;
            n.st = S_EXEC;
        } else if (d->cls == I_LD) {
            n.op1 = v1;
            n.st = S_LDMEM;
            n.do_ri = 1;
        } else if (d->cls == I_JALR || d->cls == I_ALUI || d->cls == I_SHI) {
            n.op1 = v1;
            n.op2 = d->cls == I_SHI ? (uint32_t)k->dec.rs2 : (uint32_t)d->imm;
            n.do_ri = k->do_pf;
            n.st = S_EXEC;
        } else {
            n.op1 = v1;
            n.op2 = v2;
            if (d->cls == I_ST) {
                n.st = S_STMEM;
                n.do_ri = 1;
            } else {
                n.do_ri = k->do_pf;
                n.st = S_EXEC;
            }
        }
        break;
    }
    case S_EXEC:
        n.out = k->pc + (uint32_t)d->imm;
        if (d->cls == I_BR) {
            int tk = br_taken(d->f3, k->op1, k->op2);
            n.lrd = 0;
            n.lstore = n.lbranch = tk;
            if (done)
                n.st = S_FETCH;
            if (tk) {
                dtrig = 0;
                set_ri = 1;
            }
        } else {
            n.aluq = alu(d, k->op1, k->op2);
            n.lbranch = d->cls == I_JALR;
            n.lstore = 1;
            n.lstalu = 1;
            n.st = S_FETCH;
        }
        break;
    case S_STMEM:
        if (!k->do_pf || done) {
            if (!k->do_wd) {
                n.mwsize = d->f3 == 0 ? 2 : d->f3 == 1 ? 1 : 0;
                n.op1 = k->op1 + (uint32_t)d->imm;
                set_wd = 1;
            }
            if (!k->do_pf && done) {
                n.st = S_FETCH;
                dtrig = 1;
            }
        }
        break;
    case S_LDMEM:
        n.lstore = 1;
        if (!k->do_pf || done) {
            if (!k->do_rd) {
                int w = d->f3 & 3;
                n.mwsize = w == 0 ? 2 : w == 1 ? 1 : 0;
                n.lis = d->f3 == 0 ? 2 : d->f3 == 1 ? 1 : 0;
                n.op1 = k->op1 + (uint32_t)d->imm;
                set_rd = 1;
            }
            if (!k->do_pf && done) {
                n.out = k->lis == 2 ? (uint32_t)(int8_t)mword : k->lis == 1 ? (uint32_t)(int16_t)mword : mword;
                dtrig = 1;
                n.st = S_FETCH;
            }
        }
        break;
    default:
        n.trap = 1;
        break;
    }

    n.mul0 = k->pvalid && d->cls == I_MUL && !(k->mul0 || k->mul1);
    n.mul1 = k->mul0;
    n.mrs1 = k->op1;
    n.mrs2 = k->op2;
    if (d->cls == I_MUL)
        n.mrd = mul_res(d->f3, k->mrs1, k->mrs2);
    if (k->pvalid) {
        if (k->pcnt)
            n.pcnt = k->pcnt - 1;
    } else {
        n.pcnt = 15;
    }
    n.ptimeout = k->pcnt == 0;

    if (done)
        n.do_pf = n.do_ri = n.do_rd = n.do_wd = 0;
    if (set_ri) n.do_ri = 1;
    if (set_rd) n.do_rd = 1;
    if (set_wd) n.do_wd = 1;
    n.dtrig = dtrig;
    if (k->do_ri && done)
        n.dec = decode(lat);

    if (la_read || la_write) {
        n.maddr = la_addr;
        n.mwstrb = la_write ? ws : 0;
    }
    if (la_write)
        n.mwdata = wd;
    switch (k->mstate) {
    case 0:
        if (k->do_pf || k->do_ri || k->do_rd) {
            n.mvalid = !usepf;
            n.mwstrb = 0;
            n.mstate = 1;
        }
        if (k->do_wd) {
            n.mvalid = 1;
            n.mstate = 2;
        }
        break;
    case 1:
        if (xfer) {
            if (la_read) {
                n.mvalid = 1;
                n.secondword = 1;
                if (!usepf)
                    n.pbufh = rdata >> 16;
            } else {
                n.mvalid = 0;
                n.secondword = 0;
                if (!k->do_rd) {
                    if ((rdata & 3) != 3 || k->secondword) {
                        n.pbufh = rdata >> 16;
                        n.pbuf = 1;
                    } else {
                        n.pbuf = 0;
                    }
                }
                n.mstate = (k->do_ri || k->do_rd) ? 0 : 3;
            }
        }
        break;
    case 2:
        if (xfer) {
            n.mvalid = 0;
            n.mstate = 0;
        }
        break;
    default:
        if (k->do_ri)
            n.mstate = 0;
        break;
    }
    if (clr)
        n.pbuf = 0;
    n.clrq = clr;
    n.lastmv = k->mvalid && !ready;
    if (!k->lastmv)
        n.fwreg = fw;
    if (xfer)
        n.rdq = lat;

    ram_tick(o, c, k, qh, la_addr >= 0x10000000u, 0);
    n.ramq = k->ramq;
    n.mvd = k->mvd;
    n.dead = k->dead;
    n.ext = k->ext;
    n.t = t + 1;
    *k = n;
}

/* Stalls and the run loop */

/* bio_bdma.sv 695: 0 when the core runs this cycle, else the cycle to look again
 * (TNEVER: sleep until something wakes it) */
static uint64_t stall_at(bio_t *o, int c, uint64_t t)
{
    bcore_t *k = &o->c[c];
    int wen = k->st == S_FETCH && (k->lbranch || k->lstore) && k->lrd;
    int hard = 0, soon = 0;
    if (k->st == S_LDRS1) {
        int r1 = k->dec.rs1, r2 = k->cur.noren2 ? 0 : k->dec.rs2, i;
        int rr[2];
        rr[0] = r1;
        rr[1] = r2;
        for (i = 0; i < 2; i++) {
            if (rr[i] >= 16 && rr[i] <= 19) {
                fifo_t *f = &o->f[rr[i] - 16];
                fifo_adv(f, t);
                if (!f->r1) {
                    hard = 1;
                    if (f->qn || f->at != TNEVER)
                        soon = 1;
                }
            }
        }
        if ((r1 == 30 || r2 == 30) && k->evmask && !(k->evmask & ev_all(o, t))) {
            hard = 1;
            if (ev_soon(o, t))
                soon = 1;
        }
    }
    if (wen && k->lrd >= 16 && k->lrd <= 19) {
        fifo_t *f = &o->f[k->lrd - 16];
        fifo_adv(f, t);
        if (f->qn == 8) {
            hard = 1;
            if (f->at != TNEVER)
                soon = 1;
        }
    }
    if (hard)
        return soon ? t + 1 : TNEVER;
    if (qhalt_of(k) && !quantum(o, c, t)) {
        uint64_t q = next_quantum(o, c, t + 1);
        if (q == TNEVER && ext_on(o, c) && o->pad_due != TNEVER)
            return o->pad_due > t ? o->pad_due : t + 1;
        return q;
    }
    return 0;
}

/* a core spinning on a jump to itself changes nothing; move it on whole loops */
static void skip_spin(bio_t *o, bcore_t *k, uint64_t t, uint64_t target)
{
    uint64_t p;
    if (k->st != S_FETCH || !k->dtrig || k->dec.cls != I_JAL || k->dec.imm || k->dec.rd ||
        k->dec.rs1 == 20 || k->dec.rs2 == 20 || k->lrd || k->lstore)
        return;
    if (k->last_lpc != k->npc || !k->last_l || t <= k->last_l)
        return;
    p = t - k->last_l;
    if (p > 32 || target <= t + p)
        return;
    k->t = t + ((target - 1 - t) / p) * p;
    (void)o;
}

static void restart_apply(bio_t *o, int c);

/* the next thing a core does: its scheduled reset, the cycle en_sync rises,
 * or its next cycle (TNEVER when it waits for something else) */
static uint64_t core_next(const bcore_t *k, int *is_rst)
{
    uint64_t t = TNEVER;
    *is_rst = 0;
    if (k->on_from < k->on_until) {
        if (k->t < k->on_from)
            t = k->on_from;
        else if (k->t < k->on_until && !k->blocked && k->st != S_TRAP)
            t = k->t;
    }
    if (k->rst_at != TNEVER && k->rst_at <= t) {
        t = k->rst_at;
        *is_rst = 1;
    }
    return t;
}

static void run_until(bao_t *b, bio_t *o, uint64_t target)
{
    while (!b->stop) {
        int c, best = -1, rst = 0;
        uint64_t bt = target, r;
        bcore_t *k;
        for (c = 0; c < 4; c++) {
            int is_rst;
            uint64_t n = core_next(&o->c[c], &is_rst);
            if (n < bt) {
                bt = n;
                best = c;
                rst = is_rst;
            }
        }
        if (best < 0)
            break;
        k = &o->c[best];
        o->now = bt;
        gpio_advance(b, o, bt);
        if (rst) {
            restart_apply(o, best);
            continue;
        }
        if (k->t < bt) {
            /* en_sync rises: the core goes on from where its clock stopped */
            ram_catch(o, best, bt);
            k->blocked = 0;
            continue;
        }
        r = stall_at(o, best, bt);
        if (r) {
            if (r == TNEVER) {
                k->blocked = 1;
            } else {
                if (r > k->on_until)
                    r = k->on_until;
                ram_skip(o, best, bt, r, 0);
                k->t = r;
            }
            continue;
        }
        {
            uint64_t lim = target;
            if (k->on_until < lim)
                lim = k->on_until;
            if (k->rst_at < lim)
                lim = k->rst_at;
            skip_spin(o, k, bt, lim);
        }
        if (k->t != bt) {
            continue;
        }
        core_cycle(b, o, best, bt);
        if (k->st == S_TRAP) {
            k->trap = 1;
            bao_logf(b, BAO_LOG_WARN, "BIO core %d trapped at pc 0x%03X", best, k->pc & 0xFFF);
        }
    }
}

/* IRQ */

static const char *const pio_names[4] = {"pioirq0", "pioirq1", "pioirq2", "pioirq3"};

static void evmap_feedback(bio_t *o, uint64_t t)
{
    int i;
    for (i = 0; i < 4; i++) {
        uint32_t e = 131u + (uint32_t)i;
        if ((o->irq_lines >> i) & 1 && (reg(o, 0xB0 + 4u * (5 - e / 32)) >> (e & 31)) & 1)
            ev_op(o, t, 1u << (e / 8), 1, 0);
    }
}

/* bio_bdma.sv 1051 to 1074: edge mode takes irq_agg[i] & ~irq_agg_q, which
 * truncates to bit 0 of irq_agg_q for every line */
static void irq_check(bao_t *b, bio_t *o)
{
    uint32_t ev = ev_all(o, o->now);
    int agg = 0, lines = 0, i;
    for (i = 0; i < 4; i++)
        if (ev & reg(o, 0x70 + 4u * (uint32_t)i))
            agg |= 1 << i;
    for (i = 0; i < 4; i++) {
        int a = (agg >> i) & 1;
        if ((reg(o, 0x80) >> i) & 1)
            a = a && !(o->aggq & 1);
        lines |= a << i;
    }
    o->aggq = agg;
    for (i = 0; i < 4; i++)
        if (((lines ^ o->irq_lines) >> i) & 1)
            irq_level(b, pio_names[i], (o->irq_lines >> i) & 1, (lines >> i) & 1);
    o->irq_lines = lines;
    if (lines)
        evmap_feedback(o, o->now);
}

uint32_t bio_irq_lines(bao_t *b)
{
    bio_t *o = b->bio;
    return o ? (uint32_t)o->irq_lines : 0;
}

/* Catch up */

static bio_t *bio(bao_t *b)
{
    bio_t *o = b->bio;
    int c, i;
    if (o)
        return o;
    o = calloc(1, sizeof(bio_t));
    if (!o)
        return NULL;
    b->bio = o;
    o->mul = b->bio_mul ? b->bio_mul : 1;
    o->cbase = b->cycles;
    o->eat = o->gat = o->pad_due = TNEVER;
    o->esetp = o->eclrp = 99;
    for (c = 0; c < 4; c++) {
        o->c[c].mask = 0xFFFFFFFFu;
        o->c[c].clrq = 1;
        o->c[c].pcnt = 15;
        o->c[c].rst_at = TNEVER;
        qd_reset(&o->qd[c], 0, 0);
        o->f[c].at = TNEVER;
    }
    for (i = 0; i < 4; i++)
        o->f[i].aprio = 99;
    o->hn = 1;
    o->hist[0].t = 0;
    o->hist[0].v = pins_now(b, o);
    return o;
}

static int any_active(bio_t *o)
{
    int c;
    for (c = 0; c < 4; c++) {
        const bcore_t *k = &o->c[c];
        if (k->rst_at != TNEVER)
            return 1;
        if (k->on_from < k->on_until && k->on_until > o->now && k->st != S_TRAP)
            return 1;
    }
    return o->pad_due != TNEVER || o->gat != TNEVER;
}

void bio_catch_up(bao_t *b)
{
    bio_t *o = b->bio;
    uint64_t target;
    if (!o || o->busy)
        return;
    target = fclk_now(b, o);
    if (target <= o->now)
        return;
    o->busy = 1;
    run_until(b, o, target);
    gpio_advance(b, o, target);
    o->now = target;
    irq_check(b, o);
    o->busy = 0;
}

void bio_clock_change(bao_t *b, int mul)
{
    bio_t *o = b->bio;
    if (o) {
        bio_catch_up(b);
        o->fbase = fclk_now(b, o);
        o->cbase = b->cycles;
        o->mul = mul;
    }
    b->bio_mul = mul;
}

void bio_host_pins(bao_t *b)
{
    bio_t *o = b->bio;
    if (!o)
        return;
    rec_pins(b, o, o->now);
    wake_all(o, o->now);
}

void bio_dmareq(bao_t *b, int e)
{
    bio_t *o = b->bio;
    if (!o || e < 0 || e >= 192)
        return;
    if (!((reg(o, 0xB0 + 4u * (uint32_t)(5 - e / 32)) >> (e & 31)) & 1))
        return;
    bio_catch_up(b);
    ev_op(o, o->now, 1u << (e / 8), 1, 0);
    wake_all(o, o->now);
}

int bio_isolated(bao_t *b)
{
    bio_t *o = b->bio;
    int w;
    if (!o)
        return 1;
    if (reg(o, 0x08) & 0x80 || reg(o, 0xA0) || reg(o, 0xA4))
        return 0;
    for (w = 0; w < 8; w++)
        if (b->iox[(0x100 >> 2) + w] & IOX_INT_EN)
            return 0;
    for (w = 0; w < 4; w++)
        if ((reg(o, 0xE4 + 8u * (uint32_t)w) & 0xFFFFF) || reg(o, 0x70 + 4u * (uint32_t)w))
            return 0;
    return 1;
}

int bio_active(bao_t *b)
{
    bio_t *o = b->bio;
    return o && any_active(o);
}

/* Host side */

/* synchronous reset (picorv32.v 1545 to 1571); decoder registers are not reset */
static void core_state_reset(bcore_t *k)
{
    k->st = S_FETCH;
    k->trap = 0;
    k->pc = k->npc = 0;
    k->lbranch = k->lstalu = k->lcompr = 0;
    k->lstore = 1;
    k->lrd = 2;
    k->out = 0xFFFu;
    k->lis = k->dtrig = 0;
    k->mstate = k->mvalid = 0;
    k->do_pf = k->do_ri = k->do_rd = k->do_wd = 0;
    k->secondword = k->pbuf = k->fwreg = k->lastmv = 0;
    k->clrq = 1;
    k->pvalid = k->mul0 = k->mul1 = 0;
    k->pcnt = 15;
    k->ptimeout = 0;
    k->last_l = 0;
    k->blocked = 0;
}

/* a_restart, three aclk after the restart bit, holds the core and its FIFO in
 * reset until three aclk after the bit self clears (bio_bdma.sv 447 to 449,
 * 1931, 1969). gpio_mask and event_mask reset asynchronously (2086 to 2098). */
static void restart_apply(bio_t *o, int c)
{
    bcore_t *k = &o->c[c];
    uint64_t t = k->rst_at, end = k->rst_end;
    k->rst_at = TNEVER;
    if (k->t < t)
        ram_catch(o, c, t);
    fifo_clear(&o->f[c], t);
    o->f[c].hold = end;
    k->mask = 0xFFFFFFFFu;
    k->evmask = 0;
    o->gwarned &= ~(1 << c);
    if (o->dma_busy && o->dma_owner == c)
        o->dma_busy = 0;
    if (!en_sync_at(k, t))
        return;
    core_state_reset(k);
    if (end > k->on_until)
        end = k->on_until;
    if (k->t < t)
        k->t = t;
    if (end > k->t) {
        ram_skip(o, c, k->t, end, 0);
        k->t = end;
    }
}

/* take the last strobe under the old settings, so later strobes are the new ones */
static void snap_sync(bio_t *o, uint64_t now)
{
    (void)snapped(o, now + 1);
    if (o->snap_p < now)
        o->snap_p = now;
}

/* host access through the AHB to APB bridge, one pclk setup and one access */
static void apb_wait(bao_t *b, bio_t *o)
{
    uint64_t r = b->cycles > o->apb_free ? b->cycles : o->apb_free;
    uint64_t e = ((r + 8) & ~(uint64_t)7) + 8;
    b->cycles = e;
    o->apb_free = e;
}

/* FCLK cycle of the first pclk edge (ACLK / 8) at or after the current CPU cycle */
static uint64_t pclk_edge(bao_t *b, bio_t *o)
{
    uint64_t c = (b->cycles + 7) & ~(uint64_t)7;
    return o->fbase + (c - o->cbase) * (uint64_t)o->mul;
}

/* SFR_CTRL: enable lands 3 aclk after the pclk edge, restart one pclk later
 * (bio_bdma.sv 502, 565, 566, 1842 to 1856) */
static void set_ctrl(bao_t *b, bio_t *o, uint32_t v, uint64_t now)
{
    int c;
    uint64_t e0 = pclk_edge(b, o), p = 8u * (uint64_t)o->mul;
    snap_sync(o, now);
    for (c = 0; c < 4; c++) {
        bcore_t *k = &o->c[c];
        int en = (int)((v >> c) & 1);
        if (k->t < now && (k->blocked || k->st == S_TRAP || !en_sync_at(k, k->t)))
            ram_catch(o, c, now);
        if ((v >> (c + 8)) & 1) {
            uint32_t q = reg(o, 0x50 + 4u * (uint32_t)c);
            qd_reset(&o->qd[c], e0 + 3 * p - 1, ((q >> 16) << 8) | ((q >> 8) & 0xFF));
        }
        if ((v >> (c + 4)) & 1) {
            k->rst_at = e0 + p + 3;
            k->rst_end = e0 + 3 * p + 3;
        } else if (k->rst_at != TNEVER && k->rst_end > e0 + p + 3) {
            /* this write's action pulse loads 0 into the restart bit early */
            k->rst_end = e0 + p + 3;
            if (k->rst_end <= k->rst_at)
                k->rst_at = TNEVER;
        }
        if (en && !k->en) {
            k->on_from = k->en_at = e0 + 3;
            k->on_until = TNEVER;
        } else if (!en && k->en) {
            k->on_until = e0 + 3;
        }
        k->en = en;
    }
    o->regs[0] = v & 0xFFF;
}

static uint32_t flevel(bio_t *o, uint64_t now)
{
    uint32_t v = 0;
    int f;
    for (f = 0; f < 4; f++) {
        fifo_adv(&o->f[f], now);
        v |= (uint32_t)(o->f[f].qn + o->f[f].r1) << (4 * f);
    }
    return v;
}

static uint32_t host_pop(bao_t *b, bio_t *o, int f, uint64_t now)
{
    fifo_t *q = &o->f[f];
    uint32_t v;
    fifo_adv(q, now);
    v = q->dat1;
    if (q->r1) {
        fifo_pop(q, now);
        /* a core waiting to write a full FIFO can go on */
        wake_all(o, now);
    }
    b->side++;
    return v;
}

static uint32_t evstat(bio_t *o, int e)
{
    uint32_t w = 5u - (uint32_t)e, v = 0;
    int i;
    for (i = 0; i < 4; i++) {
        uint32_t n = 131u + (uint32_t)i;
        if (n / 32 == w && (o->irq_lines >> i) & 1)
            v |= 1u << (n & 31);
    }
    return v & reg(o, 0xB0 + 4u * (5 - w));
}

int bio_read(bao_t *b, uint32_t a, uint32_t *v)
{
    bio_t *o = bio(b);
    uint32_t off;
    uint64_t now;
    if (!o)
        return -1;
    b->ff_block = 1;
    apb_wait(b, o);
    bio_catch_up(b);
    now = o->now;
    o->haddr = (a & 0xFFF) >> 2;
    if (a >= IMEM_BASE && a < IMEM_BASE + 0x4000u) {
        int c = (int)((a - IMEM_BASE) >> 12);
        uint8_t *m = o->ram[c] + (a & 0xFFC);
        *v = o->c[c].en ? o->c[c].ramq
                        : (uint32_t)m[0] | ((uint32_t)m[1] << 8) | ((uint32_t)m[2] << 16) | ((uint32_t)m[3] << 24);
        return 0;
    }
    if (a >= ALIAS_BASE && a < ALIAS_BASE + 0x4000u) {
        int p = (int)((a - ALIAS_BASE) >> 12);
        off = a & 0xFFF;
        if (off == 0x0C) *v = flevel(o, now);
        else if (off == 0x10 + 4u * (uint32_t)p) *v = o->alias_txf[p];
        else if (off == 0x20 + 4u * (uint32_t)p) *v = host_pop(b, o, p, now);
        else if (off == 0x38) *v = o->alias_set[p];
        else if (off == 0x3C) *v = o->alias_clr[p];
        else if (off == 0x40) *v = ev_all(o, now);
        else *v = 0;
        return 0;
    }
    if (a < BIO_BASE || a >= BIO_BASE + 0x1000u)
        return -1;
    off = a - BIO_BASE;
    switch (off) {
    case 0x00: *v = o->regs[0] & 0xF; break;
    case 0x04: *v = 8u | (4u << 8) | (4096u << 16); break;
    case 0x0C: *v = flevel(o, now); break;
    case 0x20: case 0x24: case 0x28: case 0x2C: *v = host_pop(b, o, (int)(off - 0x20) >> 2, now); break;
    case 0x40: *v = ev_all(o, now); break;
    case 0x90: case 0x94: case 0x98: case 0x9C: {
        bcore_t *k = &o->c[(off - 0x90) >> 2];
        *v = (k->pc & 0xFFF) | ((uint32_t)k->trap << 12);
        break;
    }
    case 0xC8: case 0xCC: case 0xD0: case 0xD4: case 0xD8: case 0xDC:
        *v = evstat(o, (int)(off - 0xC8) >> 2);
        break;
    case 0x08: case 0x10: case 0x14: case 0x18: case 0x1C: case 0x30: case 0x34: case 0x38: case 0x3C:
    case 0x44: case 0x48: case 0x50: case 0x54: case 0x58: case 0x5C: case 0x60: case 0x64: case 0x68:
    case 0x6C: case 0x70: case 0x74: case 0x78: case 0x7C: case 0x80:
    case 0xB0: case 0xB4: case 0xB8: case 0xBC: case 0xC0: case 0xC4:
        *v = o->regs[off >> 2];
        break;
    default:
        *v = 0;
        break;
    }
    return 0;
}

static void config_write(bao_t *b, bio_t *o, uint32_t v, uint64_t now)
{
    snap_sync(o, now);
    gpio_advance(b, o, now);
    o->regs[0x08 >> 2] = v & 0x3FF;
    if (o->pad_o != o->agg_o || o->pad_e != o->agg_e)
        o->pad_due = pad_when(o, now);
}

int bio_write(bao_t *b, uint32_t a, uint32_t v)
{
    bio_t *o = bio(b);
    uint32_t off;
    uint64_t now;
    if (!o)
        return -1;
    apb_wait(b, o);
    bio_catch_up(b);
    now = o->now;
    o->haddr = (a & 0xFFF) >> 2;
    if (a >= IMEM_BASE && a < IMEM_BASE + 0x4000u) {
        int c = (int)((a - IMEM_BASE) >> 12);
        uint8_t *m = o->ram[c] + (a & 0xFFC);
        if (o->c[c].en) {
            bao_logf(b, BAO_LOG_WARN, "BIO core %d is running, its IMEM write at 0x%08X is ignored", c, a);
            return 0;
        }
        m[0] = (uint8_t)v; m[1] = (uint8_t)(v >> 8); m[2] = (uint8_t)(v >> 16); m[3] = (uint8_t)(v >> 24);
        return 0;
    }
    if (a >= ALIAS_BASE && a < ALIAS_BASE + 0x4000u) {
        int p = (int)((a - ALIAS_BASE) >> 12);
        off = a & 0xFFF;
        if (off == 0x10 + 4u * (uint32_t)p) {
            o->alias_txf[p] = v;
            fifo_push(&o->f[p], now, v, 0);
        } else if (off == 0x38) {
            o->alias_set[p] = v & 0xFFFFFF;
            ev_op(o, now, v & 0xFFFFFF, 1, 0);
        } else if (off == 0x3C) {
            o->alias_clr[p] = v & 0xFFFFFF;
            ev_op(o, now, v & 0xFFFFFF, 0, 0);
        }
        wake_all(o, now);
        bao_kick(b);
        return 0;
    }
    if (a < BIO_BASE || a >= BIO_BASE + 0x1000u)
        return -1;
    off = a - BIO_BASE;
    switch (off) {
    case 0x00: set_ctrl(b, o, v, now); break;
    case 0x08: config_write(b, o, v, now); break;
    case 0x10: case 0x14: case 0x18: case 0x1C:
        o->regs[off >> 2] = v;
        fifo_push(&o->f[(off - 0x10) >> 2], now, v, 0);
        break;
    case 0x30: o->regs[off >> 2] = v; break;
    case 0x34: o->regs[off >> 2] = v & 0xFFFFFF; break;
    case 0x38: o->regs[off >> 2] = v & 0xFFFFFF; ev_op(o, now, v & 0xFFFFFF, 1, 0); break;
    case 0x3C: o->regs[off >> 2] = v & 0xFFFFFF; ev_op(o, now, v & 0xFFFFFF, 0, 0); break;
    case 0x44: o->regs[off >> 2] = v & 0xFFFFFF; break;
    case 0x48: {
        int f;
        o->regs[off >> 2] = v & 0xF;
        for (f = 0; f < 4; f++)
            if ((v >> f) & 1)
                fifo_clear(&o->f[f], now);
        break;
    }
    case 0x50: case 0x54: case 0x58: case 0x5C: {
        int c = (int)(off - 0x50) >> 2;
        snap_sync(o, now);
        gpio_advance(b, o, now);
        o->regs[off >> 2] = v;
        qd_rebase(&o->qd[c], now, ((v >> 16) << 8) | ((v >> 8) & 0xFF));
        if (o->pad_due != TNEVER)
            o->pad_due = pad_when(o, now);
        break;
    }
    case 0x60: case 0x6C: case 0x70: case 0x74: case 0x78: case 0x7C:
    case 0xA0: case 0xA4: case 0xB0: case 0xB4: case 0xB8: case 0xBC: case 0xC0: case 0xC4:
        o->regs[off >> 2] = v;
        break;
    case 0x64: case 0x68:
        gpio_advance(b, o, now);
        o->regs[off >> 2] = v;
        pad_apply(b, o, now);
        break;
    case 0x80: o->regs[off >> 2] = v & 0xF; break;
    case 0xE0: case 0xE4: case 0xE8: case 0xEC: case 0xF0: case 0xF4: case 0xF8: case 0xFC:
        o->regs[off >> 2] = v & 0xFFFFF;
        break;
    default:
        break;
    }
    wake_all(o, now);
    irq_check(b, o);
    bao_kick(b);
    return 0;
}

void bio_reset(bao_t *b)
{
    free(b->bio);
    b->bio = NULL;
    b->bio_out = b->bio_oe = 0;
}

void bio_free(bao_t *b)
{
    free(b->bio);
    b->bio = NULL;
}
