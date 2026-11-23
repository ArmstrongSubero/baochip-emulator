/*
 * mdma.c: the PL230 memory to memory DMA and its request block
 *
 * Copyright (c) 2026 Armstrong Subero
 * SPDX-License-Identifier: Apache-2.0
 *
 * PL230 at 0x40011000, request block at 0x40012000. Channel control data and
 * cycle types follow the PL230 TRM. Transfers take no time.
 */

#include <string.h>
#include "bao_int.h"

#define PL230_BASE 0x40011000u
#define MREQ_BASE  0x40012000u
#define HCLKGR     0x40040064u

static int clocked(bao_t *b)
{
    uint32_t *g = bao_mmio_slot(b, HCLKGR);
    return !g || ((*g >> 4) & 1);
}

static void note(bao_t *b, uint32_t a, const char *what)
{
    if (bao_first_time(b, a))
        bao_logf(b, BAO_LOG_WARN, "%s 0x%08X at %s", what, a, bao_where(b, b->pc));
}

/* bmxcore.sv 162 to 165: no slave below 0x61000000 above 0x5FFFFFFF (the
 * RRAM) or from 0xA0000000, and ahb_demux.sv 119 to 128 then answers
 * ready with data 0 and no error */
static int reachable(uint32_t a)
{
    return a < 0x60000000u || (a >= 0x61000000u && a < 0xA0000000u);
}

static uint32_t mrd(bao_t *b, uint32_t a, int size)
{
    uint32_t v = 0;
    uint8_t *p;
    if (!reachable(a))
        return 0;
    if ((p = b->rdmap[a >> 16]) != NULL) {
        int i;
        for (i = 0; i < size; i++)
            v |= (uint32_t)p[(a + (uint32_t)i) & 0xFFFF] << (8 * i);
        return v;
    }
    if (bus_read(b, a, size, &v))
        v = 0;
    return v;
}

static void mwr(bao_t *b, uint32_t a, int size, uint32_t v)
{
    uint8_t *p;
    b->side++;
    if (!reachable(a))
        return;
    if ((p = b->wrmap[a >> 16]) != NULL) {
        int i;
        for (i = 0; i < size; i++)
            p[(a + (uint32_t)i) & 0xFFFF] = (uint8_t)(v >> (8 * i));
        return;
    }
    bus_write(b, a, size, v);
}

/* a channel just became active: pulse mode counts it off quelen
 * (mdma.sv 169 to 176) */
static void active_rise(mdma_t *m, int ch)
{
    uint32_t cr = m->cr[ch];
    if ((cr & 3u) != 3u)
        return;
    m->quelen[ch] = (uint8_t)((m->quelen[ch] - 1u) & 15u);
    if (m->quelen[ch] > 1)
        m->queof[ch] = 1;
}

static void done(bao_t *b, int ch)
{
    (void)ch;
    irq_raise(b, "mdmairq");
}

/* one arbitration of channel ch, returns 1 when it continues without a request */
static int arbitrate(bao_t *b, int ch, int single, int *used)
{
    mdma_t *m = &b->mdma;
    uint32_t bit = 1u << ch;
    int alt = (m->prialt & bit) != 0;
    uint32_t d = (m->ctrl_base & ~0xFFu) + (alt ? 0x80u : 0u) + 16u * (uint32_t)ch;
    uint32_t cfg = mrd(b, d + 8, 4), cc = cfg & 7u;
    uint32_t src, dst, n, batch, count, i, rem;
    int ssz, dsz, sinc, dinc;

    if (cc == 0) {
        m->enable &= ~bit;
        return 0;
    }
    if ((cc == 1 || cc == 3 || cc == 7) && *used)
        return 0;

    src = mrd(b, d, 4);
    dst = mrd(b, d + 4, 4);
    n = (cfg >> 4) & 0x3FFu;
    batch = ((cfg >> 14) & 15u) >= 10 ? 1024u : 1u << ((cfg >> 14) & 15u);
    count = single ? 1u : (batch < n + 1 ? batch : n + 1);
    ssz = (int)((cfg >> 24) & 3u);
    dsz = (int)((cfg >> 28) & 3u);
    sinc = (int)((cfg >> 26) & 3u);
    dinc = (int)((cfg >> 30) & 3u);
    if (ssz == 3 || dsz == 3 || ssz != dsz)
        note(b, d + 8, "MDMA channel_cfg with a reserved or mismatched size, taken as given,");
    if (ssz == 3) ssz = 2;
    if (dsz == 3) dsz = 2;

    /* a scatter gather primary writes each task into the four words of
     * the alternate structure, which its destination end pointer names */
    for (i = 0; i < count; i++) {
        uint32_t k = n - i, kd = (cc == 4 || cc == 6) ? k & 3u : k;
        uint32_t sa = sinc == 3 ? src : src - (k << sinc);
        uint32_t da = dinc == 3 ? dst : dst - (kd << dinc);
        mwr(b, da, 1 << dsz, mrd(b, sa, 1 << ssz));
        if (b->stop)
            return 0;
    }

    rem = n + 1 - count;
    if (rem) {
        mwr(b, d + 8, 4, (cfg & ~0x3FF0u) | ((rem - 1) << 4));
        if (rem < batch)
            m->useburst &= ~bit;
    } else {
        mwr(b, d + 8, 4, cfg & ~0x3FF7u);
    }
    if (cc == 1 || cc == 3 || cc == 7)
        *used = 1;

    switch (cc) {
    case 1:
        if (!rem) { m->enable &= ~bit; done(b, ch); }
        return 0;
    case 2:
        if (!rem) { m->enable &= ~bit; done(b, ch); return 0; }
        return 1;
    case 3:
        if (!rem) { m->prialt ^= bit; done(b, ch); }
        return 0;
    case 4:
    case 6:
        m->prialt |= bit;
        return 1;
    case 5:
        if (!rem) m->prialt &= ~bit;
        return 1;
    default:
        if (!rem) { m->prialt &= ~bit; return 1; }
        return 0;
    }
}

static int pick(const mdma_t *m, uint32_t want)
{
    int ch;
    for (ch = 0; ch < MDMA_CHANNELS; ch++)
        if ((want & m->priority) >> ch & 1)
            return ch;
    for (ch = 0; ch < MDMA_CHANNELS; ch++)
        if (want >> ch & 1)
            return ch;
    return -1;
}

/* serve burst and single requests, highest priority and then lowest
 * channel first, re arbitrating after every batch as the TRM describes */
static void serve(bao_t *b, uint32_t burst, uint32_t single)
{
    mdma_t *m = &b->mdma;
    uint32_t want, sgl;
    int used[MDMA_CHANNELS];
    int ch;

    if (!(m->cfg & 1u) || !clocked(b))
        return;
    want = (burst | single) & m->enable;
    sgl = single & ~burst & ~m->useburst & want;
    want &= burst | sgl;
    if (!want)
        return;
    memset(used, 0, sizeof(used));
    for (ch = 0; ch < MDMA_CHANNELS; ch++)
        if (want >> ch & 1)
            active_rise(m, ch);
    while (want && !b->stop) {
        ch = pick(m, want);
        if (!arbitrate(b, ch, (int)(sgl >> ch & 1), &used[ch]) || !(m->enable >> ch & 1))
            want &= ~(1u << ch);
    }
}

static void run(bao_t *b, uint32_t burst, uint32_t single)
{
    mdma_t *m = &b->mdma;
    m->q_burst |= burst;
    m->q_single |= single;
    if (m->busy)
        return;
    m->busy = 1;
    while (m->q_burst | m->q_single) {
        burst = m->q_burst;
        single = m->q_single;
        m->q_burst = m->q_single = 0;
        serve(b, burst, single);
    }
    m->busy = 0;
}

/* event line e rose; e is the vex_irq index, IRQ array e/16 bit e%16 */
void mdma_event(bao_t *b, int e)
{
    mdma_t *m = &b->mdma;
    uint32_t burst = 0, single = 0;
    int ch;

    if (!clocked(b))
        return;
    for (ch = 0; ch < MDMA_CHANNELS; ch++) {
        uint32_t cr = m->cr[ch];
        if ((int)m->evsel[ch] != e || !(cr & 1u))
            continue;
        if ((cr & 3u) == 3u) {
            m->quelen[ch] = (uint8_t)((m->quelen[ch] + 1u) & 15u);
            if (m->quelen[ch] > 1)
                m->queof[ch] = 1;
        }
        if (m->reqmask >> ch & 1)
            continue;
        if (cr & 4u) burst |= 1u << ch;
        if (cr & 8u) single |= 1u << ch;
    }
    if (burst | single)
        run(b, burst, single);
}

/* PL230 TRM identification values, revision taken as 0 */
static uint32_t pl230_id(uint32_t off)
{
    switch (off) {
    case 0xFD0: return 0x04;
    case 0xFE0: return 0x30;
    case 0xFE4: return 0xB2;
    case 0xFE8: return 0x0B;
    case 0xFF0: return 0x0D;
    case 0xFF4: return 0xF0;
    case 0xFF8: return 0x05;
    case 0xFFC: return 0xB1;
    default: return 0;
    }
}

int mdma_read(bao_t *b, uint32_t a, uint32_t *v)
{
    mdma_t *m = &b->mdma;
    uint32_t off = a & 0xFFFu;

    *v = 0;
    if ((a & ~0xFFFu) == MREQ_BASE) {
        int i = (int)((off >> 2) & 7u);
        if (off < 0x20) *v = m->evsel[i];
        else if (off < 0x40) *v = m->cr[i];
        else if (off < 0x60) *v = ((uint32_t)m->queof[i] << 4) | m->quelen[i];
        else note(b, a, "MDMA read with no register, reads 0,");
        return 0;
    }
    switch (off) {
    case 0x00: *v = 0x10000000u | ((MDMA_CHANNELS - 1u) << 16) | (m->cfg & 1u); break;
    case 0x08: *v = m->ctrl_base & ~0xFFu; break;
    case 0x0C: *v = (m->ctrl_base & ~0xFFu) + 0x80u; break;
    case 0x10: {
        int ch;
        for (ch = 0; ch < MDMA_CHANNELS; ch++)
            *v |= ((m->cr[ch] >> 4) & 1u) << ch;
        break;
    }
    case 0x18: *v = m->useburst; break;
    case 0x20: *v = m->reqmask; break;
    case 0x28: *v = m->enable; break;
    case 0x30: *v = m->prialt; break;
    case 0x38: *v = m->priority; break;
    case 0x4C: *v = m->err; break;
    case 0x04: case 0x14: case 0x1C: case 0x24: case 0x2C: case 0x34: case 0x3C:
        break;
    default:
        if (off >= 0xE00 && off < 0xF00) {
            uint32_t *slot = bao_mmio_slot(b, a);
            if (bao_first_time(b, a))
                bao_logf(b, BAO_LOG_WARN, "PL230 test register read 0x%08X at %s (not modelled, acting as storage)",
                         a, bao_where(b, b->pc));
            *v = slot ? *slot : 0;
        } else if (off >= 0xFD0) {
            *v = pl230_id(off);
        } else {
            note(b, a, "PL230 read with no register, reads 0,");
        }
    }
    return 0;
}

int mdma_write(bao_t *b, uint32_t a, uint32_t v)
{
    mdma_t *m = &b->mdma;
    uint32_t off = a & 0xFFFu, ch8 = v & 0xFFu;

    if (!clocked(b))
        return 0;
    if ((a & ~0xFFFu) == MREQ_BASE) {
        int i = (int)((off >> 2) & 7u);
        if (off < 0x20) {
            m->evsel[i] = v & 0xFFu;
        } else if (off < 0x40) {
            m->cr[i] = v & 0x1Fu;
            if ((m->cr[i] & 3u) != 3u)
                m->quelen[i] = m->queof[i] = 0;
        } else if (off >= 0x60) {
            note(b, a, "MDMA write with no register, dropped,");
        }
        return 0;
    }
    switch (off) {
    case 0x04: m->cfg = v & 0xE1u; break;
    case 0x08: m->ctrl_base = v & ~0xFFu; break;
    case 0x14: run(b, ch8, 0); break;
    case 0x18: m->useburst |= ch8; break;
    case 0x1C: m->useburst &= ~ch8; break;
    case 0x20: m->reqmask |= ch8; break;
    case 0x24: m->reqmask &= ~ch8; break;
    case 0x28: m->enable |= ch8; break;
    case 0x2C: m->enable &= ~ch8; break;
    case 0x30: m->prialt |= ch8; break;
    case 0x34: m->prialt &= ~ch8; break;
    case 0x38: m->priority |= ch8; break;
    case 0x3C: m->priority &= ~ch8; break;
    case 0x4C: if (v & 1u) m->err = 0; break;
    case 0x00: case 0x0C: case 0x10: break;
    default:
        if (off >= 0xE00 && off < 0xF00) {
            uint32_t *slot = bao_mmio_slot(b, a);
            if (bao_first_time(b, a))
                bao_logf(b, BAO_LOG_WARN, "PL230 test register write 0x%08X at %s (not modelled, acting as storage)",
                         a, bao_where(b, b->pc));
            if (slot) *slot = v;
        } else if (off < 0xFD0) {
            note(b, a, "PL230 write with no register, dropped,");
        }
    }
    return 0;
}

void mdma_reset(bao_t *b)
{
    memset(&b->mdma, 0, sizeof(b->mdma));
}
