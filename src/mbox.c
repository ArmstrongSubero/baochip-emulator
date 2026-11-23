/*
 * mbox.c: the mailbox between the VexRiscv and a second core
 *
 * Copyright (c) 2026 Armstrong Subero
 * SPDX-License-Identifier: Apache-2.0
 *
 * MAILBOX at 0xE0018000, MBOX_APB at 0x40013000 and MB_CLIENT at 0xE0019000.
 * Handshakes take no time.
 */

#include <string.h>
#include "bao_int.h"

#define MBOX_APB   0x40013000u
#define MAILBOX    0xE0018000u
#define MB_CLIENT  0xE0019000u
#define ACLKGR     0x40040060u

#define EV_AVAILABLE  1u
#define EV_ABORT_INIT 2u
#define EV_ABORT_DONE 4u
#define EV_ERROR      8u

#define CL_APB 0
#define CL_CSR 1

static void note(bao_t *b, uint32_t a, const char *what)
{
    if (bao_first_time(b, a))
        bao_logf(b, BAO_LOG_WARN, "%s 0x%08X at %s", what, a, bao_where(b, b->pc));
}

/* soc_coresub.sv 454 to 455: aclk and hclk of mbox_apb both pass
 * clkmboxgate, which soc_top.sv 576 takes from ACLKGR bit 5 */
static int apb_clocked(bao_t *b)
{
    uint32_t *g = bao_mmio_slot(b, ACLKGR);
    return !g || ((*g >> 5) & 1);
}

static int fifo_writable(const mbfifo_t *f) { return f->cnt < MBOX_FIFO_WORDS; }

static void fifo_push(mbfifo_t *f, uint32_t v)
{
    f->d[(f->head + f->cnt) % MBOX_FIFO_WORDS] = v;
    f->cnt++;
}

/* the read port keeps its last word once the FIFO is empty */
static uint32_t fifo_dout(const mbfifo_t *f)
{
    return f->cnt ? f->d[f->head] : f->last;
}

static void fifo_pop(mbfifo_t *f)
{
    f->last = f->d[f->head];
    f->head = (f->head + 1) % MBOX_FIFO_WORDS;
    f->cnt--;
}

static int selected(const mbox_t *m) { return m->loopback ? CL_CSR : CL_APB; }

/* one aclk of the abort handshake (mbox_client.v 315 to 388) */
static uint32_t abort_step(mbabort_t *s, int abort, int r_abort)
{
    uint32_t ev = 0;
    switch (s->state) {
    case 1:
        if (r_abort) {
            s->state = 0;
            s->in_progress = 0;
            s->w_abort = 0;
            ev = EV_ABORT_DONE;
        } else {
            s->w_abort = 1;
        }
        break;
    case 2:
        if (abort) {
            s->state = 3;
            s->in_progress = 0;
            s->ack = 1;
            s->w_abort = 1;
        }
        break;
    case 3:
        s->state = 0;
        s->w_abort = 0;
        break;
    default:
        if (abort && !r_abort) {
            s->state = 1;
            s->ack = 0;
            s->in_progress = 1;
            s->w_abort = 1;
        } else if (abort && r_abort) {
            s->ack = 1;
            s->w_abort = 1;
        } else if (r_abort) {
            s->state = 2;
            s->in_progress = 1;
            s->w_abort = 0;
            ev = EV_ABORT_INIT;
        } else {
            s->w_abort = 0;
        }
    }
    return ev;
}

static void client_events(bao_t *b, int c, uint32_t ev)
{
    mbox_t *m = &b->mbox;
    if (c == CL_CSR) {
        m->cl[c].pending |= ev;
        return;
    }
    if (ev & EV_AVAILABLE) irq_raise(b, "mbox_irq_available");
    if (ev & EV_ABORT_INIT) irq_raise(b, "mbox_irq_abort_init");
    if (ev & EV_ABORT_DONE) irq_raise(b, "mbox_irq_done");
}

/* error events: pending on the rise of tx_err | rx_err (cram_axi.sv
 * 20951 and 21089); the APB client's irq_error is the level itself
 * (mbox_client.v 264) */
static void error_edges(bao_t *b)
{
    mbox_t *m = &b->mbox;
    int c, now;

    now = m->tx_err | m->rx_err;
    if (now && !m->err_was)
        m->pending |= EV_ERROR;
    m->err_was = now;

    now = m->cl[CL_CSR].tx_err | m->cl[CL_CSR].rx_err;
    if (now && !m->cl[CL_CSR].err_was)
        m->cl[CL_CSR].pending |= EV_ERROR;
    m->cl[CL_CSR].err_was = now;

    c = CL_APB;
    now = m->cl[c].tx_err | m->cl[c].rx_err;
    if (now != m->cl[c].err_was)
        irq_level(b, "mbox_irq_error", m->cl[c].err_was, now);
    m->cl[c].err_was = now;
}

/* run the three abort state machines a cycle at a time until they rest;
 * the pulses act on the first cycle only */
static void abort_settle(bao_t *b, int abort_mbx, int abort_cl0, int abort_cl1)
{
    mbox_t *m = &b->mbox;
    int step, c;

    for (step = 0; step < 16; step++) {
        mbabort_t om = m->ab, oc[2];
        int r_mbx, r_cl;
        uint32_t ev;

        oc[0] = m->cl[0].ab;
        oc[1] = m->cl[1].ab;
        r_mbx = oc[selected(m)].w_abort;
        r_cl = om.w_abort;

        ev = abort_step(&m->ab, step == 0 && abort_mbx, r_mbx);
        m->pending |= ev;
        for (c = 0; c < 2; c++) {
            if (c == CL_APB && !apb_clocked(b))
                continue;
            ev = abort_step(&m->cl[c].ab, step == 0 && (c ? abort_cl1 : abort_cl0), r_cl);
            client_events(b, c, ev);
        }
        if (step > 0 && memcmp(&om, &m->ab, sizeof(om)) == 0 &&
            memcmp(&oc[0], &m->cl[0].ab, sizeof(oc[0])) == 0 &&
            memcmp(&oc[1], &m->cl[1].ab, sizeof(oc[1])) == 0)
            break;
    }
}

/* take a word from the selected client when the FIFO has room (cram_axi.sv 12765) */
static void drain_pending(bao_t *b)
{
    mbox_t *m = &b->mbox;
    int s = selected(m);
    if (!m->cl[s].w_pending || !fifo_writable(&m->rx))
        return;
    if (!m->ab.in_progress)
        fifo_push(&m->rx, m->cl[s].wdata);
    m->cl[s].w_pending = 0;
    m->cl[!s].w_pending = 0;
}

static void client_write(bao_t *b, int c, uint32_t v)
{
    mbox_t *m = &b->mbox;
    mbclient_t *cl = &m->cl[c];
    int ready = c == selected(m) && fifo_writable(&m->rx);

    cl->wdata = v;
    if (ready) {
        if (!m->ab.in_progress)
            fifo_push(&m->rx, v);
        if (m->cl[!c].w_pending)
            m->cl[!c].w_pending = 0;
        return;
    }
    /* MB_CLIENT flags a write over a word still pending (cram_axi.sv
     * 21051); the APB client flags any cycle a word waits (mbox_client.v
     * 524) */
    if (c == CL_CSR ? cl->w_pending : 1)
        cl->tx_err = 1;
    cl->w_pending = 1;
}

static uint32_t client_rdata(bao_t *b, int c)
{
    mbox_t *m = &b->mbox;
    uint32_t v = fifo_dout(&m->tx);
    if (!m->tx.cnt)
        m->cl[c].rx_err = 1;
    else if (c == selected(m))
        fifo_pop(&m->tx);
    return v;
}

static uint32_t client_status_read(bao_t *b, int c)
{
    mbclient_t *cl = &b->mbox.cl[c];
    int valid = b->mbox.tx.cnt != 0;
    uint32_t v;
    int tx_free = !cl->w_pending;

    if (c == CL_APB)
        /* mbox.sv 96, 97 and 113 cross the error bits: bit 4 is the
         * client's rx_err and bit 5 its tx_err */
        v = (uint32_t)valid | (uint32_t)tx_free << 1 | (uint32_t)cl->ab.in_progress << 2 |
            (uint32_t)cl->ab.ack << 3 | (uint32_t)cl->rx_err << 4 | (uint32_t)cl->tx_err << 5;
    else
        v = (uint32_t)valid | (uint32_t)tx_free << 1 | (uint32_t)cl->ab.in_progress << 2 |
            (uint32_t)cl->ab.ack << 3 | (uint32_t)cl->tx_err << 4 | (uint32_t)cl->rx_err << 5;
    if (cl->tx_err)
        cl->w_pending = 0;
    cl->tx_err = cl->rx_err = 0;
    return v;
}

static void client_done(bao_t *b, int c)
{
    if (c == selected(&b->mbox))
        b->mbox.pending |= EV_AVAILABLE;
}

static void mailbox_done(bao_t *b)
{
    client_events(b, CL_CSR, EV_AVAILABLE);
    if (apb_clocked(b))
        client_events(b, CL_APB, EV_AVAILABLE);
}

static uint32_t mailbox_status(const mbox_t *m)
{
    return (m->rx.cnt & 0x7FFu) | (m->tx.cnt & 0x7FFu) << 11 |
           (uint32_t)m->ab.in_progress << 22 | (uint32_t)m->ab.ack << 23 |
           (uint32_t)m->tx_err << 24 | (uint32_t)m->rx_err << 25;
}

static void finish(bao_t *b)
{
    error_edges(b);
    b->side++;
    bao_kick(b);
}

int mbox_read(bao_t *b, uint32_t a, uint32_t *v)
{
    mbox_t *m = &b->mbox;
    uint32_t off = a & 0xFFFu;
    int touched = 0;

    *v = 0;
    if ((a & ~0xFFFu) == MAILBOX) {
        switch (off) {
        case 0x00: *v = m->wdata; break;
        case 0x04:
            *v = fifo_dout(&m->rx);
            if (m->rx.cnt) {
                fifo_pop(&m->rx);
                drain_pending(b);
            } else {
                m->rx_err = 1;
            }
            touched = 1;
            break;
        case 0x08: *v = (uint32_t)(m->tx_err | m->rx_err) << 3; break;
        case 0x0C: *v = m->pending; break;
        case 0x10: *v = m->enable; break;
        case 0x14:
            *v = mailbox_status(m);
            touched = m->tx_err | m->rx_err;
            m->tx_err = m->rx_err = 0;
            break;
        case 0x18: *v = m->control; break;
        case 0x1C: *v = m->done; break;
        case 0x20: *v = m->loopback; break;
        default: note(b, a, "mailbox read with no register, reads 0,");
        }
    } else if ((a & ~0xFFFu) == MB_CLIENT) {
        mbclient_t *cl = &m->cl[CL_CSR];
        switch (off) {
        case 0x00: *v = cl->wdata; break;
        case 0x04: *v = client_rdata(b, CL_CSR); touched = 1; break;
        case 0x08:
            touched = cl->tx_err | cl->rx_err;
            *v = client_status_read(b, CL_CSR);
            break;
        case 0x0C: *v = (uint32_t)(cl->tx_err | cl->rx_err) << 3; break;
        case 0x10: *v = cl->pending; break;
        case 0x14: *v = cl->enable; break;
        case 0x18: *v = cl->control; break;
        case 0x1C: *v = cl->done; break;
        default: note(b, a, "MB_CLIENT read with no register, reads 0,");
        }
    } else {
        mbclient_t *cl = &m->cl[CL_APB];
        int clk = apb_clocked(b);
        switch (off) {
        case 0x00: *v = cl->wdata; break;
        case 0x04:
            if (clk) {
                *v = client_rdata(b, CL_APB);
                touched = 1;
            } else {
                *v = fifo_dout(&m->tx);
            }
            break;
        case 0x08:
            if (clk) {
                touched = cl->tx_err | cl->rx_err;
                *v = client_status_read(b, CL_APB);
            } else {
                mbclient_t keep = *cl;
                *v = client_status_read(b, CL_APB);
                *cl = keep;
            }
            break;
        case 0x18: case 0x1C: break;
        default: note(b, a, "MBOX_APB read with no register, reads 0,");
        }
    }
    if (touched)
        finish(b);
    return 0;
}

int mbox_write(bao_t *b, uint32_t a, uint32_t v)
{
    mbox_t *m = &b->mbox;
    uint32_t off = a & 0xFFFu;

    if ((a & ~0xFFFu) == MAILBOX) {
        switch (off) {
        case 0x00:
            m->wdata = v;
            if (!fifo_writable(&m->tx))
                m->tx_err = 1;
            else if (!m->ab.in_progress)
                fifo_push(&m->tx, v);
            break;
        case 0x0C: m->pending &= ~(v & 0xFu); break;
        case 0x10: m->enable = v & 0xFu; break;
        case 0x18:
            m->control = v & 1u;
            if (v & 1u) {
                /* the abort pulse also empties both FIFOs (cram_axi.sv
                 * 12727, 12750) */
                m->tx.cnt = m->tx.head = 0;
                m->rx.cnt = m->rx.head = 0;
                abort_settle(b, 1, 0, 0);
                drain_pending(b);
            }
            break;
        case 0x1C:
            m->done = v & 1u;
            if (v & 1u)
                mailbox_done(b);
            break;
        case 0x20:
            m->loopback = v & 1u;
            drain_pending(b);
            abort_settle(b, 0, 0, 0);
            break;
        case 0x04: case 0x08: case 0x14: break;
        default: note(b, a, "mailbox write with no register, dropped,");
        }
    } else if ((a & ~0xFFFu) == MB_CLIENT) {
        mbclient_t *cl = &m->cl[CL_CSR];
        switch (off) {
        case 0x00: client_write(b, CL_CSR, v); break;
        case 0x10: cl->pending &= ~(v & 0xFu); break;
        case 0x14: cl->enable = v & 0xFu; break;
        case 0x18:
            cl->control = v & 1u;
            if (v & 1u) {
                abort_settle(b, 0, 0, 1);
                drain_pending(b);
            }
            break;
        case 0x1C:
            cl->done = v & 1u;
            if (v & 1u)
                client_done(b, CL_CSR);
            break;
        case 0x04: case 0x08: case 0x0C: break;
        default: note(b, a, "MB_CLIENT write with no register, dropped,");
        }
    } else {
        if (!apb_clocked(b)) {
            note(b, a, "MBOX_APB write with its clock gated, lost,");
            return 0;
        }
        switch (off) {
        case 0x00: client_write(b, CL_APB, v); break;
        /* apb_ar pulses only for the value 1 (apb_sfr.sv 259, mbox.sv 114
         * and 115) */
        case 0x18:
            if (v == 1u) {
                abort_settle(b, 0, 1, 0);
                drain_pending(b);
            }
            break;
        case 0x1C:
            if (v == 1u)
                client_done(b, CL_APB);
            break;
        case 0x04: case 0x08: break;
        default: note(b, a, "MBOX_APB write with no register, dropped,");
        }
    }
    finish(b);
    return 0;
}

uint32_t mbox_irq_lines(bao_t *b)
{
    const mbox_t *m = &b->mbox;
    uint32_t l = 0;
    if (m->pending & m->enable)
        l |= 1u << 22;
    if (m->cl[CL_CSR].pending & m->cl[CL_CSR].enable)
        l |= 1u << 23;
    return l;
}

void mbox_reset(bao_t *b)
{
    memset(&b->mbox, 0, sizeof(b->mbox));
}
