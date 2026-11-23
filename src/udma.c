/*
 * udma.c: UDMA UART, SPI master, I2C and ADC models
 *
 * Copyright (c) 2026 Armstrong Subero
 * SPDX-License-Identifier: Apache-2.0
 *
 * Channel registers follow udma_ch_addrgen.sv. Events are raised when the
 * bus time for a transfer has passed.
 */

#include <string.h>
#include <stdio.h>
#include "bao_int.h"

#define UDMA_BASE 0x50100000u

/* CFG datasize readback per reg_if */
enum { UDMA_DS_ZERO, UDMA_DS_WORD, UDMA_DS_STORED };

/* udma_ctrl.sv: CG and RST hold one bit per peripheral, 20 of them
 * (udma_sub.sv 173 with the counts of soc_ifsub.sv 77 to 84) */
#define UDMA_PERIPH_MASK 0xFFFFFu
#define UART_SETUP_MASK  0xFFFF033Fu

/* CG bit per peripheral (udma_sub.sv 207 to 217) */
#define CG_UART(i) (i)
#define CG_SPIM(i) (4 + (i))
#define CG_I2C(i)  (8 + (i))
#define CG_ADC     19

static int cg_on(bao_t *b, int per)
{
    return (int)((b->udma_cg >> per) & 1);
}

static uint64_t uart_byte_cycles(bao_t *b, uart_t *u)
{
    uint32_t div = u->setup >> 16;
    if (b->uart_instant)
        return 8;
    if (div == 0)
        div = 1;
    return (uint64_t)10 * div * BAO_ACLK_HZ / BAO_PERCLK_HZ;
}

static void ch_reset(udma_ch_t *c)
{
    memset(c, 0, sizeof(*c));
}

static int ch_cfg_write(udma_ch_t *c, uint32_t v)
{
    c->cfg = v & 0x87;
    if (v & 0x40) {
        c->en = 0;
        c->left = 0;
        return 0;
    }
    if ((v & 0x10) && !c->en) {
        c->en = 1;
        c->cur = c->saddr;
        c->left = c->size;
        return 1;
    }
    return 0;
}

static uint32_t ch_cfg_read(udma_ch_t *c)
{
    uint32_t v = (c->en ? 0x10u : 0) | (c->cfg & 1);
    if (c->rd == UDMA_DS_WORD) v |= 0x4;
    else if (c->rd == UDMA_DS_STORED) v |= c->cfg & 0x86;
    return v;
}

/* an offset the reg_if does not decode reads 0 and drops writes */
static int udma_none(bao_t *b, uint32_t a, uint32_t *v)
{
    if (v) *v = 0;
    if (bao_first_time(b, a))
        bao_logf(b, BAO_LOG_WARN, "UDMA %s with no register 0x%08X at %s, %s",
                 v ? "read" : "write", a, bao_where(b, b->pc), v ? "reads 0" : "dropped");
    return 0;
}

static int ch_reg_read(udma_ch_t *c, uint32_t off, uint32_t *v)
{
    switch (off) {
    case 0x0: *v = c->en ? c->cur : c->saddr; return 1;
    case 0x4: *v = c->en ? c->left : 0; return 1;
    case 0x8: *v = ch_cfg_read(c); return 1;
    default: return 0;
    }
}

static int ch_elem(udma_ch_t *c)
{
    switch ((c->cfg >> 1) & 3) {
    case 1: return 2;
    case 2: return 4;
    default: return 1;
    }
}

static void ch_advance(udma_ch_t *c, uint32_t n)
{
    if (n > c->left)
        n = c->left;
    c->cur += n;
    c->left -= n;
}

static void ch_finish(bao_t *b, udma_ch_t *c, const char *ev)
{
    if (c->en && c->left == 0) {
        if (c->cfg & 1) {
            c->cur = c->saddr;
            c->left = c->size;
        } else {
            c->en = 0;
        }
        if (ev)
            irq_raise(b, ev);
    }
}

static int uart_rx_ready(uart_t *u)
{
    if (!(u->setup & (1u << 9)))
        return 0;
    if ((u->setup & (1u << 4)) || (u->irq_en & 1))
        return u->fifo_count < (int)sizeof(u->fifo);
    return u->rx.en && u->rx.left > 0;
}

static void uart_start_tx(bao_t *b, int i)
{
    uart_t *u = &b->uart[i];
    uint32_t n = u->tx.left, k;
    uint8_t buf[0x10000];

    if (!cg_on(b, CG_UART(i)))
        return;
    if (n > sizeof(buf))
        n = sizeof(buf);
    if (bus_dma_read(b, u->tx.cur, buf, n)) {
        bao_logf(b, BAO_LOG_ERROR, "UART%d TX DMA from 0x%08X is outside IFRAM/SRAM", i, u->tx.cur);
        return;
    }
    for (k = 0; k < n; k++)
        if (b->uart_cb)
            b->uart_cb(b->uart_user, i, buf[k]);
    u->tx_busy = 1;
    u->tx_done_at = b->cycles + (n ? n : 1) * uart_byte_cycles(b, u);
    u->tx_pending_len = (int)n;
    bao_kick(b);
}

static void uart_update(bao_t *b, int i)
{
    uart_t *u = &b->uart[i];
    char ev[24];

    if (u->tx_busy && b->cycles >= u->tx_done_at) {
        u->tx_busy = 0;
        ch_advance(&u->tx, (uint32_t)u->tx_pending_len);
        snprintf(ev, sizeof(ev), "uart%d_tx", i);
        ch_finish(b, &u->tx, ev);
        if (u->tx.en)
            uart_start_tx(b, i);
    }
    while (cg_on(b, CG_UART(i)) && u->host_count > 0 && uart_rx_ready(u) &&
           b->cycles >= u->rx_next_at) {
        uint8_t c = u->host_q[u->host_head];
        u->host_head = (u->host_head + 1) % (int)sizeof(u->host_q);
        u->host_count--;
        if ((u->setup & (1u << 4)) || (u->irq_en & 1)) {
            u->fifo[(u->fifo_head + u->fifo_count) % (int)sizeof(u->fifo)] = c;
            u->fifo_count++;
            if ((u->irq_en & 1) && !(u->setup & (1u << 4))) {
                snprintf(ev, sizeof(ev), "uart%d_rx_char", i);
                irq_raise(b, ev);
            }
        } else {
            bus_dma_write(b, u->rx.cur, &c, 1);
            ch_advance(&u->rx, 1);
            snprintf(ev, sizeof(ev), "uart%d_rx", i);
            ch_finish(b, &u->rx, ev);
        }
        u->rx_next_at = b->cycles + uart_byte_cycles(b, u);
    }
}

static int spi_xfer(bao_t *b, int i, int tx)
{
    int r = -1;
    if (dev_spi_xfer(b, i, b->spim[i].cs, tx, &r))
        return r & 0xFF;
    if (b->spi_cb)
        r = b->spi_cb(b->spi_user, i, b->spim[i].cs, tx);
    return r < 0 ? 0xFF : (r & 0xFF);
}

static void spi_cs(bao_t *b, int i, int active)
{
    spim_t *s = &b->spim[i];
    if (s->cs_active == active)
        return;
    s->cs_active = active;
    dev_spi_cs(b, i, s->cs, active);
    if (b->spi_cs_cb)
        b->spi_cs_cb(b->spi_user, i, s->cs, active);
}

static int spim_needs(bao_t *b, spim_t *s, uint32_t *words, int n, uint32_t *need_tx, uint32_t *need_rx)
{
    int k;
    *need_tx = *need_rx = 0;
    for (k = 0; k < n; k++) {
        uint32_t w = words[k], op = w >> 28, cnt = (w & 0xFFFF) + 1;
        if (op == 6 || op == 0xC) *need_tx += cnt * (uint32_t)ch_elem(&s->tx);
        if (op == 7 || op == 0xC) *need_rx += cnt * (uint32_t)ch_elem(&s->rx);
        if (op == 0xB || op == 0xD || op == 0xE) {
            bao_logf(b, BAO_LOG_WARN, "SPIM command 0x%X is not modelled yet", op);
            return -1;
        }
    }
    return 0;
}

static void spim_start(bao_t *b, int i)
{
    spim_t *s = &b->spim[i];
    uint32_t words[1024], need_tx, need_rx, bits = 0;
    uint8_t txb[0x10000];
    int n, k, rpt_start = -1;
    uint32_t rpt_left = 0, tpos = 0;

    if (s->busy || !s->cmd.en || !cg_on(b, CG_SPIM(i)))
        return;
    n = (int)(s->cmd.left / 4);
    if (n > 1024) n = 1024;
    for (k = 0; k < n; k++) {
        uint8_t t[4];
        if (bus_dma_read(b, s->cmd.cur + 4u * (uint32_t)k, t, 4)) {
            bao_logf(b, BAO_LOG_ERROR, "SPIM%d command fetch from 0x%08X failed", i, s->cmd.cur);
            return;
        }
        words[k] = (uint32_t)t[0] | ((uint32_t)t[1] << 8) | ((uint32_t)t[2] << 16) | ((uint32_t)t[3] << 24);
    }
    if (spim_needs(b, s, words, n, &need_tx, &need_rx))
        return;
    if ((need_tx && (!s->tx.en || s->tx.left < need_tx)) ||
        (need_rx && (!s->rx.en || s->rx.left < need_rx)))
        return;
    if (need_tx && bus_dma_read(b, s->tx.cur, txb, need_tx)) {
        bao_logf(b, BAO_LOG_ERROR, "SPIM%d TX DMA from 0x%08X failed", i, s->tx.cur);
        return;
    }

    s->rx_stage_len = 0;
    s->events = 0;
    for (k = 0; k < n; k++) {
        uint32_t w = words[k], op = w >> 28;
        switch (op) {
        case 0x0:
            s->clkdiv = w & 0xFF;
            s->cpha = (w >> 8) & 1;
            s->cpol = (w >> 9) & 1;
            break;
        case 0x1:
            s->cs = (int)(w & 3);
            spi_cs(b, i, 1);
            break;
        case 0x2: {
            uint32_t nb = ((w >> 16) & 0xF) + 1, v = w & 0xFFFF;
            if (nb > 8) spi_xfer(b, i, (int)((v >> 8) & 0xFF));
            spi_xfer(b, i, (int)(v & 0xFF));
            bits += nb;
            break;
        }
        case 0x4:
            bits += ((w >> 16) & 0x1F) + 1;
            break;
        case 0x5:
            bits += w & 0xFF;
            break;
        case 0xB: {
            uint32_t bpw = ((w >> 16) & 0x1F) + 1, e;
            for (e = (bpw + 7) / 8; e > 0; e--)
                spi_xfer(b, i, 0xFF);
            bits += bpw;
            break;
        }
        case 0x6: case 0x7: case 0xC: {
            uint32_t cnt = (w & 0xFFFF) + 1, bpw = ((w >> 16) & 0x1F) + 1, j, e;
            uint32_t wpt = 1u << ((w >> 21) & 3);
            int et = ch_elem(&s->tx) / (int)wpt, er = ch_elem(&s->rx) / (int)wpt;
            if (et < 1) et = 1;
            if (er < 1) er = 1;
            for (j = 0; j < cnt; j++) {
                uint32_t v = 0, r = 0, nbytes = (bpw + 7) / 8;
                if (op != 0x7) {
                    for (e = 0; e < (uint32_t)et; e++)
                        v |= (uint32_t)txb[tpos + e] << (8 * e);
                    tpos += (uint32_t)et;
                }
                for (e = nbytes; e > 0; e--) {
                    int o = spi_xfer(b, i, op == 0x7 ? 0xFF : (int)((v >> (8 * (e - 1))) & 0xFF));
                    r |= (uint32_t)o << (8 * (e - 1));
                }
                if (op != 0x6) {
                    for (e = 0; e < (uint32_t)er && s->rx_stage_len < (int)sizeof(s->rx_stage); e++)
                        s->rx_stage[s->rx_stage_len++] = (uint8_t)(r >> (8 * e));
                }
                bits += bpw;
            }
            break;
        }
        case 0x8:
            rpt_start = k + 1;
            rpt_left = w & 0xFFFF;
            break;
        case 0xA:
            if (rpt_start >= 0 && rpt_left > 0) {
                rpt_left--;
                k = rpt_start - 1;
            }
            break;
        case 0x9:
            if (!((w >> 1) & 1))
                spi_cs(b, i, 0);
            if (w & 1)
                s->events |= 1;
            break;
        default:
            break;
        }
    }
    s->tx_used = (int)tpos;
    s->rx_used = s->rx_stage_len;
    s->busy = 1;
    s->done_at = b->cycles + 1 + (uint64_t)bits * 2 * (s->clkdiv + 1) * BAO_ACLK_HZ / BAO_PERCLK_HZ;
    bao_kick(b);
}

static void spim_update(bao_t *b, int i)
{
    spim_t *s = &b->spim[i];
    char ev[24];
    if (!s->busy || b->cycles < s->done_at)
        return;
    s->busy = 0;
    if (s->rx_used) {
        bus_dma_write(b, s->rx.cur, s->rx_stage, (uint32_t)s->rx_used);
        ch_advance(&s->rx, (uint32_t)s->rx_used);
        snprintf(ev, sizeof(ev), "spim%d_rx", i);
        ch_finish(b, &s->rx, ev);
    }
    if (s->tx_used) {
        ch_advance(&s->tx, (uint32_t)s->tx_used);
        snprintf(ev, sizeof(ev), "spim%d_tx", i);
        ch_finish(b, &s->tx, ev);
    }
    s->cmd.left = 0;
    snprintf(ev, sizeof(ev), "spim%d_cmd", i);
    ch_finish(b, &s->cmd, ev);
    if (s->events & 1) {
        snprintf(ev, sizeof(ev), "spim%d_eot", i);
        irq_raise(b, ev);
    }
}

static int i2c_dev(bao_t *b, int i, int event, uint8_t *data)
{
    int r = 0;
    if (dev_i2c(b, i, event, data, &r))
        return r;
    if (b->i2c_cb)
        return b->i2c_cb(b->i2c_user, i, event, data);
    if (event == BAO_I2C_READ)
        *data = 0xFF;
    return 0;
}

static void i2c_start(bao_t *b, int i)
{
    i2c_t *c = &b->i2c[i];
    uint32_t words[1024], need_tx = 0, need_rx = 0, bits = 0, tpos = 0;
    uint8_t txb[0x10000];
    int n, k;
    uint32_t repeat = 0;
    char ev[24];

    if (c->busy || !c->cmd.en || !cg_on(b, CG_I2C(i)))
        return;
    n = (int)(c->cmd.left / 4);
    if (n > 1024) n = 1024;
    for (k = 0; k < n; k++) {
        uint8_t t[4];
        if (bus_dma_read(b, c->cmd.cur + 4u * (uint32_t)k, t, 4)) {
            bao_logf(b, BAO_LOG_ERROR, "I2C%d command fetch from 0x%08X failed", i, c->cmd.cur);
            return;
        }
        words[k] = (uint32_t)t[0] | ((uint32_t)t[1] << 8) | ((uint32_t)t[2] << 16) | ((uint32_t)t[3] << 24);
    }
    for (k = 0; k < n; k++) {
        uint32_t op = words[k] >> 28, times = 1;
        if (op == 0xC) { repeat = words[k] & 0xFFFF; continue; }
        if (repeat) { times = repeat; repeat = 0; }
        if (op == 0x8) need_tx += times;
        if (op == 0x4 || op == 0x6) need_rx += times;
    }
    if ((need_tx && (!c->tx.en || c->tx.left < need_tx)) ||
        (need_rx && (!c->rx.en || c->rx.left < need_rx)))
        return;
    if (need_tx && bus_dma_read(b, c->tx.cur, txb, need_tx))
        return;

    c->rx_stage_len = 0;
    c->events = 0;
    repeat = 0;
    for (k = 0; k < n; k++) {
        uint32_t w = words[k], op = w >> 28, times = 1, t;
        if (op == 0xC) { repeat = w & 0xFFFF; continue; }
        if (repeat) { times = repeat; repeat = 0; }
        for (t = 0; t < times; t++) {
            uint8_t d;
            switch (op) {
            case 0xE: c->div = w & 0xFFFF; break;
            case 0x0:
                i2c_dev(b, i, BAO_I2C_START, &d);
                c->addressed = 0;
                if (!(c->events & 8)) {
                    c->events |= 8;
                    snprintf(ev, sizeof(ev), "i2c%d_err", i);
                    irq_raise(b, ev);
                }
                bits += 1;
                break;
            case 0x2:
                i2c_dev(b, i, BAO_I2C_STOP, &d);
                bits += 1;
                break;
            case 0x7: case 0x8:
                d = op == 0x7 ? (uint8_t)(w & 0xFF) : txb[tpos++];
                if (!i2c_dev(b, i, c->addressed ? BAO_I2C_WRITE : BAO_I2C_ADDR, &d)) {
                    c->events |= 2;
                }
                c->addressed = 1;
                bits += 9;
                break;
            case 0x4: case 0x6:
                d = 0xFF;
                i2c_dev(b, i, BAO_I2C_READ, &d);
                if (c->rx_stage_len < (int)sizeof(c->rx_stage))
                    c->rx_stage[c->rx_stage_len++] = d;
                bits += 9;
                break;
            case 0xA:
                bits += w & 0xFFFF;
                break;
            case 0x9:
                c->events |= 1;
                break;
            default:
                break;
            }
        }
    }
    c->tx_used = (int)tpos;
    c->rx_used = c->rx_stage_len;
    c->busy = 1;
    c->done_at = b->cycles + 1 + (uint64_t)bits * 5 * (c->div + 1) * BAO_ACLK_HZ / BAO_PERCLK_HZ;
    /* The command path holds six words (a two entry FIFO and a four entry
     * clock crossing FIFO in udma_i2c_top.sv), so a list that fits is taken
     * whole and the channel goes idle before START reaches the pins. */
    if (n <= 6) {
        c->cmd.left = 0;
        snprintf(ev, sizeof(ev), "i2c%d_cmd", i);
        ch_finish(b, &c->cmd, ev);
        c->events |= 4;
    }
    bao_kick(b);
}

static void i2c_update(bao_t *b, int i)
{
    i2c_t *c = &b->i2c[i];
    char ev[24];
    if (!c->busy || b->cycles < c->done_at)
        return;
    c->busy = 0;
    if (c->rx_used) {
        bus_dma_write(b, c->rx.cur, c->rx_stage, (uint32_t)c->rx_used);
        ch_advance(&c->rx, (uint32_t)c->rx_used);
        snprintf(ev, sizeof(ev), "i2c%d_rx", i);
        ch_finish(b, &c->rx, ev);
    }
    if (c->tx_used) {
        ch_advance(&c->tx, (uint32_t)c->tx_used);
        snprintf(ev, sizeof(ev), "i2c%d_tx", i);
        ch_finish(b, &c->tx, ev);
    }
    if (c->events & 2) {
        c->nack = 1;
        snprintf(ev, sizeof(ev), "i2c%d_nack", i);
        irq_raise(b, ev);
    }
    if (!(c->events & 4)) {
        c->cmd.left = 0;
        snprintf(ev, sizeof(ev), "i2c%d_cmd", i);
        ch_finish(b, &c->cmd, ev);
    }
    if (c->events & 1) {
        snprintf(ev, sizeof(ev), "i2c%d_eot", i);
        irq_raise(b, ev);
    }
}

#define ADC_FLOOR 63u

static void adc_start(bao_t *b)
{
    if (!b->adc_rx.en || !(b->adc_cr & (1u << 14)) || !cg_on(b, CG_ADC))
        return;
    b->adc_done_at = b->cycles + 20 * (BAO_ACLK_HZ / 1000000);
    bao_kick(b);
}

static void adc_update(bao_t *b)
{
    uint32_t raw, ch;
    uint8_t t[4];
    if (b->adc_done_at == NEVER || b->cycles < b->adc_done_at)
        return;
    b->adc_done_at = NEVER;
    if (!b->adc_rx.en)
        return;
    if (b->adc_cr & (1u << 24)) {
        ch = (b->adc_cr >> 26) & 3;
        raw = b->adc_mv[ch] * 1023u / 1208u;
        if (raw > 1023) raw = 1023;
        /* board reads 63 below 0.30 V */
        if (raw < ADC_FLOOR) raw = ADC_FLOOR;
    } else {
        /* The temperature sensor: the board reads about 496 to 510 at room
         * temperature, 29 to 37 C on the typical 516 at 25 C, falling 1.72
         * counts a degree. */
        raw = 503;
    }
    t[0] = (uint8_t)raw; t[1] = (uint8_t)(raw >> 8); t[2] = 0; t[3] = 0;
    bus_dma_write(b, b->adc_rx.cur, t, b->adc_rx.left < 4 ? b->adc_rx.left : 4);
    ch_advance(&b->adc_rx, 4);
    ch_finish(b, &b->adc_rx, "adc_rx");
}

int udma_read(bao_t *b, uint32_t a, uint32_t *v)
{
    uint32_t blk = (a - UDMA_BASE) >> 12, off = a & 0xFFF;

    if (blk == 0) {
        if (off == 0) { *v = b->udma_cg; return 0; }
        if (off == 4) { *v = b->udma_evt; return 0; }
        if (off == 8) { *v = b->udma_rst; return 0; }
        return udma_none(b, a, v);
    }
    if (blk >= 1 && blk <= 4) {
        uart_t *u = &b->uart[blk - 1];
        if (off < 0x10) return ch_reg_read(&u->rx, off, v) ? 0 : udma_none(b, a, v);
        if (off < 0x20) return ch_reg_read(&u->tx, off - 0x10, v) ? 0 : udma_none(b, a, v);
        switch (off) {
        case 0x20: *v = (u->tx_busy ? 1u : 0); return 0;
        case 0x24: *v = u->setup & UART_SETUP_MASK; return 0;
        case 0x28: *v = u->error & 3; u->error = 0; b->side++; return 0;
        case 0x2C: *v = u->irq_en & 3; return 0;
        case 0x30: *v = u->fifo_count > 0; return 0;
        case 0x34:
            b->side++;
            if (u->fifo_count > 0) {
                *v = u->fifo[u->fifo_head];
                u->fifo_head = (u->fifo_head + 1) % (int)sizeof(u->fifo);
                u->fifo_count--;
                bao_kick(b);
            } else {
                *v = 0;
            }
            return 0;
        default: return udma_none(b, a, v);
        }
    }
    if (blk >= 5 && blk <= 8) {
        spim_t *s = &b->spim[blk - 5];
        if (off < 0x10) return ch_reg_read(&s->rx, off, v) ? 0 : udma_none(b, a, v);
        if (off < 0x20) return ch_reg_read(&s->tx, off - 0x10, v) ? 0 : udma_none(b, a, v);
        if (off < 0x30) return ch_reg_read(&s->cmd, off - 0x20, v) ? 0 : udma_none(b, a, v);
        if (off == 0x30) { *v = s->busy ? 1u : 0; return 0; }
        return udma_none(b, a, v);
    }
    /* SETUP at 0x34 is commented out of udma_i2c_reg_if.sv (250 to 252);
     * ACK at 0x38 clears on read (276) */
    if (blk >= 9 && blk <= 12) {
        i2c_t *c = &b->i2c[blk - 9];
        if (off < 0x10) return ch_reg_read(&c->rx, off, v) ? 0 : udma_none(b, a, v);
        if (off < 0x20) return ch_reg_read(&c->tx, off - 0x10, v) ? 0 : udma_none(b, a, v);
        if (off < 0x30) return ch_reg_read(&c->cmd, off - 0x20, v) ? 0 : udma_none(b, a, v);
        if (off == 0x30) { *v = c->busy ? 1u : 0; return 0; }
        if (off == 0x38) { *v = (uint32_t)c->nack; c->nack = 0; b->side++; return 0; }
        return udma_none(b, a, v);
    }
    if (blk == 0x14) {
        if (off < 0x10) return ch_reg_read(&b->adc_rx, off, v) ? 0 : udma_none(b, a, v);
        if (off == 0x10) { *v = b->adc_cr; return 0; }
        return udma_none(b, a, v);
    }
    return -1;
}

static int ch_write(udma_ch_t *c, uint32_t off, uint32_t v, int *started)
{
    switch (off) {
    case 0x0: c->saddr = v; return 1;
    case 0x4: c->size = v & 0xFFFFF; return 1;
    case 0x8: *started = ch_cfg_write(c, v); return 1;
    default: return 0;
    }
}

int udma_write(bao_t *b, uint32_t a, uint32_t v)
{
    uint32_t blk = (a - UDMA_BASE) >> 12, off = a & 0xFFF;
    int st = 0;

    if (blk == 0) {
        if (off == 0) {
            int i;
            b->udma_cg = v & UDMA_PERIPH_MASK;
            for (i = 0; i < 4; i++)
                if (b->uart[i].tx.en && !b->uart[i].tx_busy)
                    uart_start_tx(b, i);
            if (b->adc_done_at == NEVER)
                adc_start(b);
            bao_kick(b);
            return 0;
        }
        if (off == 4) { b->udma_evt = v; return 0; }
        if (off == 8) { b->udma_rst = v & UDMA_PERIPH_MASK; return 0; }
        return udma_none(b, a, NULL);
    }
    if (blk >= 1 && blk <= 4) {
        int i = (int)blk - 1;
        uart_t *u = &b->uart[i];
        if (off < 0x10) { ch_write(&u->rx, off, v, &st); bao_kick(b); return 0; }
        if (off < 0x20) {
            ch_write(&u->tx, off - 0x10, v, &st);
            if (st && !u->tx_busy)
                uart_start_tx(b, i);
            return 0;
        }
        switch (off) {
        case 0x24:
            if (!cg_on(b, CG_UART(i))) return 0;
            u->setup = v & UART_SETUP_MASK;
            if (v & (1u << 5)) u->fifo_count = 0;
            bao_kick(b);
            return 0;
        case 0x2C: if (cg_on(b, CG_UART(i))) u->irq_en = v & 3; return 0;
        case 0x20: case 0x28: case 0x30: case 0x34: return 0;
        default: return udma_none(b, a, NULL);
        }
    }
    if (blk >= 5 && blk <= 8) {
        int i = (int)blk - 5;
        spim_t *s = &b->spim[i];
        if (off < 0x10) ch_write(&s->rx, off, v, &st);
        else if (off < 0x20) ch_write(&s->tx, off - 0x10, v, &st);
        else if (off < 0x30) ch_write(&s->cmd, off - 0x20, v, &st);
        else if (off == 0x30) return 0;
        else return udma_none(b, a, NULL);
        if (st)
            spim_start(b, i);
        return 0;
    }
    if (blk >= 9 && blk <= 12) {
        int i = (int)blk - 9;
        i2c_t *c = &b->i2c[i];
        if (off < 0x10) ch_write(&c->rx, off, v, &st);
        else if (off < 0x20) ch_write(&c->tx, off - 0x10, v, &st);
        else if (off < 0x30) ch_write(&c->cmd, off - 0x20, v, &st);
        else if (off == 0x30 || off == 0x38) return 0;
        else return udma_none(b, a, NULL);
        if (st)
            i2c_start(b, i);
        return 0;
    }
    if (blk == 0x14) {
        if (off < 0x10) {
            ch_write(&b->adc_rx, off, v, &st);
            if (st) adc_start(b);
            return 0;
        }
        if (off == 0x10) { if (cg_on(b, CG_ADC)) b->adc_cr = v; return 0; }
        return udma_none(b, a, NULL);
    }
    return -1;
}

void udma_update(bao_t *b)
{
    int i;
    for (i = 0; i < 4; i++) {
        uart_update(b, i);
        spim_update(b, i);
        i2c_update(b, i);
        if (!b->spim[i].busy && b->spim[i].cmd.en)
            spim_start(b, i);
        if (!b->i2c[i].busy && b->i2c[i].cmd.en)
            i2c_start(b, i);
    }
    adc_update(b);
}

uint64_t udma_next_event(bao_t *b)
{
    uint64_t n = NEVER;
    int i;
    for (i = 0; i < 4; i++) {
        uart_t *u = &b->uart[i];
        if (u->tx_busy && u->tx_done_at < n) n = u->tx_done_at;
        if (u->host_count > 0 && uart_rx_ready(u)) {
            uint64_t t = u->rx_next_at > b->cycles ? u->rx_next_at : b->cycles + 1;
            if (t < n) n = t;
        }
        if (b->spim[i].busy && b->spim[i].done_at < n) n = b->spim[i].done_at;
        if (b->i2c[i].busy && b->i2c[i].done_at < n) n = b->i2c[i].done_at;
    }
    if (b->adc_done_at < n) n = b->adc_done_at;
    return n;
}

int udma_host_input_pending(bao_t *b)
{
    int i;
    for (i = 0; i < 4; i++)
        if (b->uart[i].host_count > 0)
            return 1;
    return 0;
}

void udma_reset(bao_t *b)
{
    int i;
    b->udma_cg = 0;
    for (i = 0; i < 4; i++) {
        memset(&b->uart[i], 0, sizeof(b->uart[i]));
        memset(&b->spim[i], 0, sizeof(b->spim[i]));
        memset(&b->i2c[i], 0, sizeof(b->i2c[i]));
    }
    b->uart[2].setup = 0x0316u | (99u << 16);
    b->udma_cg = 1u << CG_UART(2);
    b->udma_evt = b->udma_rst = 0;
    for (i = 0; i < 4; i++) {
        b->spim[i].cmd.rd = b->i2c[i].cmd.rd = UDMA_DS_WORD;
        b->spim[i].rx.rd = UDMA_DS_STORED;
        b->spim[i].rx.cfg = b->spim[i].tx.cfg = 0x4;
    }
    ch_reset(&b->adc_rx);
    b->adc_rx.rd = UDMA_DS_WORD;
    b->adc_cr = 0;
    b->adc_done_at = NEVER;
}

void bao_uart_rx(bao_t *b, int inst, const uint8_t *data, int len)
{
    uart_t *u;
    int k;
    if (inst < 0 || inst > 3)
        return;
    u = &b->uart[inst];
    for (k = 0; k < len && u->host_count < (int)sizeof(u->host_q); k++) {
        u->host_q[(u->host_head + u->host_count) % (int)sizeof(u->host_q)] = data[k];
        u->host_count++;
    }
    bao_kick(b);
}

/* What the SPI master was last set to: clkdiv in bits 7:0, CPHA in bit 8, CPOL in bit 9 */
BAO_API uint32_t bao_spi_config(bao_t *b, int inst)
{
    if (inst < 0 || inst > 3)
        return 0;
    return (b->spim[inst].clkdiv & 0xFF) | ((b->spim[inst].cpha & 1) << 8) | ((b->spim[inst].cpol & 1) << 9);
}

/* The I2C master's clock divider as last set by its command stream */
BAO_API uint32_t bao_i2c_div(bao_t *b, int inst)
{
    if (inst < 0 || inst > 3)
        return 0;
    return b->i2c[inst].div;
}
