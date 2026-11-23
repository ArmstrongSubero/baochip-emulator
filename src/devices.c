/*
 * devices.c: parts that sit on the Dabao's SPI and I2C buses
 *
 * Copyright (c) 2026 Armstrong Subero
 * SPDX-License-Identifier: Apache-2.0
 *
 * spi-loopback, w25q, sd, mpu6050, st7796 and ft6336. Options after a comma:
 * cs=, dc= and int= pins.
 */

#ifndef _WIN32
#define _POSIX_C_SOURCE 200809L
#endif
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "bao_int.h"

#ifdef _WIN32
#define strtok_r strtok_s
#endif

enum { DEV_NONE, DEV_LOOP, DEV_W25Q, DEV_SD, DEV_MPU, DEV_LCD, DEV_TOUCH };

#define LCD_W 320
#define LCD_H 480

typedef struct dev {
    int kind;
    int bus, sel;
    char path[256];

    uint8_t *mem;
    uint32_t size;
    int dirty;

    uint8_t cmd[8];
    int ncmd;
    uint32_t addr;
    int phase;
    uint8_t sr1, sr2;

    uint8_t out[1100];
    int out_head, out_len;
    uint8_t blk[520];
    int blk_len, blk_token;
    int acmd, idle, multi_read, multi_write;
    uint32_t lba;

    uint8_t regs[128];
    uint8_t ptr;
    int first;

    int cs_port, cs_pin, dc_port, dc_pin, int_port, int_pin, cs_prev;

    uint8_t *fb;
    uint8_t lcmd, lpar[16];
    int lnpar, colmod, madctl, inv, on, sleep;
    int xs, xe, ys, ye, cx, cy, pix_n;
    uint8_t pix[3];
    uint32_t frames;

    int tdown[2], tx[2], ty[2];
} dev_t;

#define MAXDEV 16

typedef struct {
    dev_t d[MAXDEV];
    int n;
    int gpio_cs;
} devs_t;

static devs_t *devs(bao_t *b)
{
    if (!b->devs)
        b->devs = calloc(1, sizeof(devs_t));
    return b->devs;
}

static int pin_low(bao_t *b, int port, int pin)
{
    return !((gpio_eff_out(b, port) >> pin) & 1) && ((gpio_eff_oe(b, port) >> pin) & 1);
}

static int selected(bao_t *b, dev_t *d, int sel)
{
    if (d->kind == DEV_LOOP) return 1;
    if (d->cs_port >= 0) return pin_low(b, d->cs_port, d->cs_pin);
    return d->sel == sel;
}

static dev_t *find(bao_t *b, int kindmask_spi, int bus, int sel)
{
    devs_t *t = b->devs;
    int i;
    if (!t)
        return NULL;
    for (i = 0; i < t->n; i++) {
        dev_t *d = &t->d[i];
        int spi = d->kind != DEV_MPU && d->kind != DEV_TOUCH;
        if (spi == kindmask_spi && d->bus == bus && selected(b, d, sel))
            return d;
    }
    return NULL;
}

static void q_put(dev_t *d, uint8_t v)
{
    if (d->out_len < (int)sizeof(d->out)) {
        d->out[(d->out_head + d->out_len) % (int)sizeof(d->out)] = v;
        d->out_len++;
    }
}

static int q_get(dev_t *d)
{
    int v;
    if (!d->out_len)
        return 0xFF;
    v = d->out[d->out_head];
    d->out_head = (d->out_head + 1) % (int)sizeof(d->out);
    d->out_len--;
    return v;
}

static void q_clear(dev_t *d)
{
    d->out_head = d->out_len = 0;
}

static int w25q_byte(dev_t *d, int tx)
{
    uint8_t op;
    if (d->ncmd < (int)sizeof(d->cmd))
        d->cmd[d->ncmd] = (uint8_t)tx;
    d->ncmd++;
    op = d->cmd[0];
    switch (op) {
    case 0x9F:
        if (d->ncmd == 2) return 0xEF;
        if (d->ncmd == 3) return 0x40;
        if (d->ncmd == 4) return 0x18;
        return 0xFF;
    case 0x90:
        if (d->ncmd == 5) return 0xEF;
        if (d->ncmd == 6) return 0x17;
        return 0xFF;
    case 0x05: return d->ncmd > 1 ? d->sr1 : 0xFF;
    case 0x35: return d->ncmd > 1 ? d->sr2 : 0xFF;
    case 0x31: if (d->ncmd == 2) d->sr2 = (uint8_t)tx; return 0xFF;
    case 0x01:
        if (d->ncmd == 2) d->sr1 = (uint8_t)(tx & 0xFC);
        if (d->ncmd == 3) d->sr2 = (uint8_t)tx;
        return 0xFF;
    case 0x03: case 0x0B: case 0x6B: case 0xEB: {
        int hdr = op == 0x03 ? 4 : 5;
        if (d->ncmd <= 4) {
            if (d->ncmd > 1) d->addr = (d->addr << 8) | (uint32_t)tx;
            if (d->ncmd == 1) d->addr = 0;
            return 0xFF;
        }
        if (d->ncmd <= hdr) return 0xFF;
        return d->mem[(d->addr++) & (d->size - 1)];
    }
    case 0x02:
        if (d->ncmd == 1) d->addr = 0;
        if (d->ncmd > 1 && d->ncmd <= 4) { d->addr = (d->addr << 8) | (uint32_t)tx; return 0xFF; }
        if (d->ncmd > 4 && (d->sr1 & 2)) {
            uint32_t a = (d->addr & ~0xFFu) | ((d->addr + (uint32_t)(d->ncmd - 5)) & 0xFF);
            d->mem[a & (d->size - 1)] &= (uint8_t)tx;
            d->dirty = 1;
        }
        return 0xFF;
    case 0x20: case 0x52: case 0xD8:
        if (d->ncmd == 1) d->addr = 0;
        if (d->ncmd > 1 && d->ncmd <= 4) d->addr = (d->addr << 8) | (uint32_t)tx;
        return 0xFF;
    default:
        return 0xFF;
    }
}

static void w25q_cs(dev_t *d, int active)
{
    uint8_t op = d->cmd[0];
    if (active) {
        d->ncmd = 0;
        return;
    }
    if (d->ncmd == 0)
        return;
    if (op == 0x06) d->sr1 |= 2;
    else if (op == 0x04) d->sr1 &= (uint8_t)~2;
    else if ((op == 0x20 || op == 0x52 || op == 0xD8) && d->ncmd >= 4 && (d->sr1 & 2)) {
        uint32_t len = op == 0x20 ? 0x1000u : op == 0x52 ? 0x8000u : 0x10000u;
        memset(d->mem + ((d->addr & (d->size - 1)) & ~(len - 1)), 0xFF, len);
        d->dirty = 1;
    } else if ((op == 0xC7 || op == 0x60) && (d->sr1 & 2)) {
        memset(d->mem, 0xFF, d->size);
        d->dirty = 1;
    } else if (op == 0x99) {
        d->sr1 = 0;
    }
    if (op == 0x02 || op == 0x20 || op == 0x52 || op == 0xD8 || op == 0xC7 || op == 0x60 ||
        op == 0x01 || op == 0x31)
        d->sr1 &= (uint8_t)~2;
    d->ncmd = 0;
}

static void sd_read_block(dev_t *d)
{
    uint32_t off = d->lba * 512u, i;
    q_put(d, 0xFF);
    q_put(d, 0xFE);
    for (i = 0; i < 512; i++)
        q_put(d, off + i < d->size ? d->mem[off + i] : 0);
    q_put(d, 0xFF);
    q_put(d, 0xFF);
    d->lba++;
}

static void sd_command(dev_t *d)
{
    uint8_t c = d->cmd[0] & 0x3F;
    uint32_t arg = ((uint32_t)d->cmd[1] << 24) | ((uint32_t)d->cmd[2] << 16) |
                   ((uint32_t)d->cmd[3] << 8) | d->cmd[4];
    int acmd = d->acmd;
    uint8_t r1 = d->idle ? 0x01 : 0x00;
    d->acmd = 0;

    if (c == 12) {
        d->multi_read = 0;
        q_clear(d);
        q_put(d, 0xFF);
        q_put(d, 0x00);
        q_put(d, 0x00);
        return;
    }
    q_put(d, 0xFF);
    if (acmd && c == 41) {
        d->idle = 0;
        q_put(d, 0x00);
        return;
    }
    switch (c) {
    case 0:
        d->idle = 1;
        d->multi_read = d->multi_write = 0;
        q_put(d, 0x01);
        break;
    case 8:
        q_put(d, r1); q_put(d, 0x00); q_put(d, 0x00); q_put(d, 0x01); q_put(d, (uint8_t)arg);
        break;
    case 55:
        d->acmd = 1;
        q_put(d, r1);
        break;
    case 58:
        q_put(d, r1); q_put(d, 0xC0); q_put(d, 0xFF); q_put(d, 0x80); q_put(d, 0x00);
        break;
    case 9: {
        uint32_t csize = d->size / (512u * 1024u) - 1, i;
        uint8_t csd[16] = { 0x40, 0x0E, 0x00, 0x32, 0x5B, 0x59, 0x00, 0x00,
                            0x00, 0x00, 0x7F, 0x80, 0x0A, 0x40, 0x00, 0x01 };
        csd[7] = (uint8_t)((csize >> 16) & 0x3F);
        csd[8] = (uint8_t)(csize >> 8);
        csd[9] = (uint8_t)csize;
        q_put(d, r1); q_put(d, 0xFF); q_put(d, 0xFE);
        for (i = 0; i < 16; i++) q_put(d, csd[i]);
        q_put(d, 0xFF); q_put(d, 0xFF);
        break;
    }
    case 10: {
        int i;
        static const uint8_t cid[16] = { 0x03, 'S', 'D', 'B', 'A', 'O', 'E', 'M',
                                         0x10, 0, 0, 0, 1, 0x01, 0x6A, 0x01 };
        q_put(d, r1); q_put(d, 0xFF); q_put(d, 0xFE);
        for (i = 0; i < 16; i++) q_put(d, cid[i]);
        q_put(d, 0xFF); q_put(d, 0xFF);
        break;
    }
    case 13:
        q_put(d, r1); q_put(d, 0x00);
        break;
    case 16:
        q_put(d, r1);
        break;
    case 17:
        q_put(d, r1);
        d->lba = arg;
        sd_read_block(d);
        break;
    case 18:
        q_put(d, r1);
        d->lba = arg;
        d->multi_read = 1;
        sd_read_block(d);
        break;
    case 24:
    case 25:
        q_put(d, r1);
        d->lba = arg;
        d->multi_write = c == 25;
        d->phase = 1;
        break;
    default:
        q_put(d, 0x05);
        break;
    }
}

static int sd_byte(dev_t *d, int tx)
{
    int out;

    if (d->phase == 1) {
        out = q_get(d);
        if (tx == 0xFE || tx == 0xFC) {
            d->phase = 2;
            d->blk_len = 0;
        } else if (tx == 0xFD) {
            d->phase = 0;
            d->multi_write = 0;
            q_put(d, 0x00);
            q_put(d, 0xFF);
        }
        return out;
    }
    if (d->phase == 2) {
        d->blk[d->blk_len++] = (uint8_t)tx;
        if (d->blk_len == 514) {
            uint32_t off = d->lba * 512u;
            if (off + 512 <= d->size) {
                memcpy(d->mem + off, d->blk, 512);
                d->dirty = 1;
            }
            d->lba++;
            q_put(d, 0x05);
            q_put(d, 0x00);
            q_put(d, 0x00);
            q_put(d, 0xFF);
            d->phase = d->multi_write ? 1 : 0;
        }
        return 0xFF;
    }

    out = q_get(d);
    if (d->ncmd == 0 && (tx & 0xC0) == 0x40) {
        d->cmd[d->ncmd++] = (uint8_t)tx;
    } else if (d->ncmd > 0) {
        d->cmd[d->ncmd++] = (uint8_t)tx;
        if (d->ncmd == 6) {
            d->ncmd = 0;
            sd_command(d);
        }
    }
    if (d->multi_read && d->out_len < 4)
        sd_read_block(d);
    return out;
}

static int mpu_i2c(dev_t *d, int ev, uint8_t *data)
{
    switch (ev) {
    case BAO_I2C_WRITE:
        if (d->first) {
            d->ptr = *data & 0x7F;
            d->first = 0;
        } else {
            d->regs[d->ptr & 0x7F] = *data;
            if ((d->ptr & 0x7F) == 0x6B && (*data & 0x80))
                d->regs[0x6B] = 0x40;
            d->ptr++;
        }
        return 1;
    case BAO_I2C_READ:
        *data = d->regs[d->ptr & 0x7F];
        d->ptr++;
        return 1;
    default:
        return 1;
    }
}

static void mpu_reset(dev_t *d, int addr)
{
    int16_t v[7] = { 0, 0, 16384, -3920, 0, 0, 0 };
    int i;
    memset(d->regs, 0, sizeof(d->regs));
    d->regs[0x75] = (uint8_t)(addr & 0x7E);
    d->regs[0x6B] = 0x40;
    for (i = 0; i < 7; i++) {
        d->regs[0x3B + 2 * i] = (uint8_t)((uint16_t)v[i] >> 8);
        d->regs[0x3C + 2 * i] = (uint8_t)v[i];
    }
}

static void lcd_pixel(dev_t *d)
{
    int r, g, bl, px, py, lw = (d->madctl & 0x20) ? LCD_H : LCD_W, lh = (d->madctl & 0x20) ? LCD_W : LCD_H;
    uint8_t *p;
    if (d->colmod == 6) {
        r = d->pix[0]; g = d->pix[1]; bl = d->pix[2];
    } else {
        uint16_t v = (uint16_t)((d->pix[0] << 8) | d->pix[1]);
        r = ((v >> 11) & 31) * 255 / 31;
        g = ((v >> 5) & 63) * 255 / 63;
        bl = (v & 31) * 255 / 31;
    }
    if (!(d->madctl & 0x08)) { int t = r; r = bl; bl = t; }
    if (d->cx < lw && d->cy < lh) {
        px = d->cx; py = d->cy;
        if (d->madctl & 0x20) { int t = px; px = py; py = t; }
        if (!(d->madctl & 0x40)) px = LCD_W - 1 - px;
        if (d->madctl & 0x80) py = LCD_H - 1 - py;
        if (px >= 0 && px < LCD_W && py >= 0 && py < LCD_H) {
            p = d->fb + 3 * (py * LCD_W + px);
            p[0] = (uint8_t)r; p[1] = (uint8_t)g; p[2] = (uint8_t)bl;
        }
    }
    d->cx++;
    if (d->cx > d->xe) {
        d->cx = d->xs;
        d->cy++;
        if (d->cy > d->ye) { d->cy = d->ys; d->frames++; }
    }
}

static void lcd_byte(bao_t *b, dev_t *d, int v)
{
    int dc = d->dc_port >= 0 ? (int)((gpio_eff_out(b, d->dc_port) >> d->dc_pin) & 1) : 1;
    if (!dc) {
        d->lcmd = (uint8_t)v;
        d->lnpar = 0;
        switch (v) {
        case 0x01: d->madctl = 0; d->colmod = 5; d->inv = 0; d->on = 0; d->sleep = 1; break;
        case 0x10: d->sleep = 1; break;
        case 0x11: d->sleep = 0; break;
        case 0x20: d->inv = 0; break;
        case 0x21: d->inv = 1; break;
        case 0x28: d->on = 0; break;
        case 0x29: d->on = 1; break;
        case 0x2C: d->cx = d->xs; d->cy = d->ys; d->pix_n = 0; break;
        case 0x3C: d->pix_n = 0; break;
        default: break;
        }
        return;
    }
    if (d->lcmd == 0x2C || d->lcmd == 0x3C) {
        d->pix[d->pix_n++] = (uint8_t)v;
        if (d->pix_n == (d->colmod == 6 ? 3 : 2)) {
            d->pix_n = 0;
            lcd_pixel(d);
        }
        return;
    }
    if (d->lnpar < 16)
        d->lpar[d->lnpar] = (uint8_t)v;
    d->lnpar++;
    if (d->lcmd == 0x2A && d->lnpar == 4) {
        d->xs = (d->lpar[0] << 8) | d->lpar[1];
        d->xe = (d->lpar[2] << 8) | d->lpar[3];
    } else if (d->lcmd == 0x2B && d->lnpar == 4) {
        d->ys = (d->lpar[0] << 8) | d->lpar[1];
        d->ye = (d->lpar[2] << 8) | d->lpar[3];
    } else if (d->lcmd == 0x36 && d->lnpar == 1) {
        d->madctl = v;
    } else if (d->lcmd == 0x3A && d->lnpar == 1) {
        d->colmod = (v & 7) == 6 ? 6 : 5;
    }
}

int dev_spi_xfer(bao_t *b, int inst, int cs, int tx, int *rx)
{
    dev_t *d = find(b, 1, inst, cs);
    if (!d)
        return 0;
    switch (d->kind) {
    case DEV_LOOP: *rx = tx; break;
    case DEV_W25Q: *rx = w25q_byte(d, tx); break;
    case DEV_SD: *rx = sd_byte(d, tx); break;
    case DEV_LCD: lcd_byte(b, d, tx); *rx = 0xFF; break;
    default: return 0;
    }
    return 1;
}

static void cs_edge(dev_t *d, int active)
{
    if (d->kind == DEV_W25Q)
        w25q_cs(d, active);
    if (d->kind == DEV_SD && !active)
        d->ncmd = 0;
    if (d->kind == DEV_LCD && !active)
        d->pix_n = 0;
}

int dev_spi_cs(bao_t *b, int inst, int cs, int active)
{
    dev_t *d = find(b, 1, inst, cs);
    if (!d || d->cs_port >= 0)
        return 0;
    cs_edge(d, active);
    return 1;
}

void dev_gpio_changed(bao_t *b)
{
    devs_t *t = b->devs;
    int i;
    if (!t || !t->gpio_cs)
        return;
    for (i = 0; i < t->n; i++) {
        dev_t *d = &t->d[i];
        if (d->cs_port >= 0) {
            int act = pin_low(b, d->cs_port, d->cs_pin);
            if (act != d->cs_prev) {
                d->cs_prev = act;
                cs_edge(d, act);
            }
        }
    }
}

static void touch_int(bao_t *b, dev_t *d)
{
    if (d->int_port >= 0)
        bao_gpio_set_input(b, d->int_port, d->int_pin, !(d->tdown[0] || d->tdown[1]));
}

static uint8_t touch_reg(dev_t *d, uint8_t r)
{
    int n = d->tdown[0] + d->tdown[1], p;
    if (r == 0x02) return (uint8_t)n;
    if (r >= 0x03 && r <= 0x0E) {
        int slot = (r - 0x03) / 6, off = (r - 0x03) % 6, id;
        p = slot;
        if (slot == 0 && !d->tdown[0] && d->tdown[1]) p = 1;
        if (slot == 1 && (!d->tdown[0] || !d->tdown[1])) return 0xFF;
        if (!d->tdown[p]) return 0xFF;
        id = p;
        switch (off) {
        case 0: return (uint8_t)(0x80 | ((d->tx[p] >> 8) & 0x0F));
        case 1: return (uint8_t)d->tx[p];
        case 2: return (uint8_t)((id << 4) | ((d->ty[p] >> 8) & 0x0F));
        case 3: return (uint8_t)d->ty[p];
        case 4: return 0x20;
        default: return 0x00;
        }
    }
    switch (r) {
    case 0xA3: return 0x64;
    case 0xA6: return 0x10;
    case 0xA8: return 0x11;
    case 0xA0: return 0x01;
    default: return d->regs[r & 0x7F];
    }
}

static int touch_i2c(dev_t *d, int ev, uint8_t *data)
{
    if (ev == BAO_I2C_WRITE) {
        if (d->first) { d->ptr = *data; d->first = 0; }
        else { d->regs[d->ptr & 0x7F] = *data; d->ptr++; }
        return 1;
    }
    if (ev == BAO_I2C_READ) {
        *data = touch_reg(d, d->ptr);
        d->ptr++;
        return 1;
    }
    return 1;
}

int dev_i2c(bao_t *b, int inst, int ev, uint8_t *data, int *ret)
{
    devs_t *t = b->devs;
    int i, hit = 0;
    if (!t)
        return 0;
    for (i = 0; i < t->n; i++) {
        dev_t *d = &t->d[i];
        if ((d->kind != DEV_MPU && d->kind != DEV_TOUCH) || d->bus != inst)
            continue;
        switch (ev) {
        case BAO_I2C_START:
            d->phase = 0;
            break;
        case BAO_I2C_ADDR:
            d->phase = (*data >> 1) == d->sel;
            if (d->phase) {
                hit = 1;
                *ret = 1;
                if (!(*data & 1))
                    d->first = 1;
            }
            break;
        case BAO_I2C_WRITE:
        case BAO_I2C_READ:
            if (d->phase) {
                hit = 1;
                *ret = d->kind == DEV_TOUCH ? touch_i2c(d, ev, data) : mpu_i2c(d, ev, data);
            }
            break;
        default:
            d->phase = 0;
            break;
        }
    }
    return hit;
}

static int load_image(dev_t *d, uint32_t size)
{
    FILE *f;
    d->mem = malloc(size);
    if (!d->mem)
        return -1;
    memset(d->mem, d->kind == DEV_SD ? 0 : 0xFF, size);
    d->size = size;
    if (d->path[0] && (f = fopen(d->path, "rb")) != NULL) {
        long n;
        fseek(f, 0, SEEK_END);
        n = ftell(f);
        fseek(f, 0, SEEK_SET);
        if (d->kind == DEV_SD && n > 0) {
            free(d->mem);
            d->size = (uint32_t)n;
            d->mem = calloc(1, (size_t)n);
            if (!d->mem) { fclose(f); return -1; }
        }
        if (fread(d->mem, 1, d->size < (uint32_t)n ? d->size : (size_t)n, f) == 0 && n > 0) {
            fclose(f);
            return -1;
        }
        fclose(f);
    }
    return 0;
}

static int parse_pin(const char *v, int *port, int *pin)
{
    if ((v[0] != 'P' && v[0] != 'p') || v[1] < 'A' || v[1] > 'F')
        return -1;
    *port = v[1] - 'A';
    *pin = atoi(v + 2);
    return (*pin >= 0 && *pin < 16) ? 0 : -1;
}

static int parse_opts(bao_t *b, dev_t *d, const char *spec)
{
    char buf[256], *tok, *save = NULL;
    snprintf(buf, sizeof(buf), "%s", spec);
    for (tok = strtok_r(buf, ",", &save); tok; tok = strtok_r(NULL, ",", &save)) {
        int *pp = NULL, *pn = NULL;
        if (!strncmp(tok, "cs=", 3)) { pp = &d->cs_port; pn = &d->cs_pin; }
        else if (!strncmp(tok, "dc=", 3)) { pp = &d->dc_port; pn = &d->dc_pin; }
        else if (!strncmp(tok, "int=", 4)) { pp = &d->int_port; pn = &d->int_pin; }
        if (pp) {
            if (parse_pin(strchr(tok, '=') + 1, pp, pn)) {
                bao_logf(b, BAO_LOG_ERROR, "bad pin in %s", tok);
                return -1;
            }
        } else {
            snprintf(d->path, sizeof(d->path), "%s", tok);
        }
    }
    return 0;
}

int bao_attach(bao_t *b, const char *kind, int bus, int sel, const char *path)
{
    devs_t *t = devs(b);
    dev_t *d;
    if (!t || t->n >= MAXDEV)
        return -1;
    d = &t->d[t->n];
    memset(d, 0, sizeof(*d));
    d->bus = bus;
    d->sel = sel;
    d->cs_port = d->dc_port = d->int_port = -1;
    if (path && parse_opts(b, d, path))
        return -1;
    if (!strcmp(kind, "spi-loopback")) {
        d->kind = DEV_LOOP;
    } else if (!strcmp(kind, "w25q")) {
        d->kind = DEV_W25Q;
        if (load_image(d, 16u << 20)) return -1;
    } else if (!strcmp(kind, "sd")) {
        d->kind = DEV_SD;
        if (load_image(d, 64u << 20)) return -1;
    } else if (!strcmp(kind, "mpu6050")) {
        d->kind = DEV_MPU;
        mpu_reset(d, sel ? sel : 0x68);
        if (!sel) d->sel = 0x68;
    } else if (!strcmp(kind, "st7796")) {
        d->kind = DEV_LCD;
        d->fb = calloc(LCD_W * LCD_H, 3);
        if (!d->fb) return -1;
        d->colmod = 5;
        d->sleep = 1;
        d->xe = LCD_W - 1;
        d->ye = LCD_H - 1;
    } else if (!strcmp(kind, "ft6336")) {
        d->kind = DEV_TOUCH;
        if (!sel) d->sel = 0x38;
    } else {
        bao_logf(b, BAO_LOG_ERROR, "unknown device %s", kind);
        return -1;
    }
    t->n++;
    if (d->cs_port >= 0)
        t->gpio_cs = 1;
    if (d->kind == DEV_TOUCH)
        touch_int(b, d);
    return 0;
}

static dev_t *first_of(bao_t *b, int kind)
{
    devs_t *t = b->devs;
    int i;
    if (!t) return NULL;
    for (i = 0; i < t->n; i++)
        if (t->d[i].kind == kind) return &t->d[i];
    return NULL;
}

int bao_touch(bao_t *b, int id, int down, int x, int y)
{
    dev_t *d = first_of(b, DEV_TOUCH);
    if (!d || id < 0 || id > 1)
        return -1;
    d->tdown[id] = down ? 1 : 0;
    d->tx[id] = x;
    d->ty[id] = y;
    touch_int(b, d);
    return 0;
}

int bao_display_rgb_rot(bao_t *b, uint8_t *out, int rot)
{
    dev_t *d = first_of(b, DEV_LCD);
    int vx, vy, vw = (rot == 90 || rot == 270) ? LCD_H : LCD_W, vh = (rot == 90 || rot == 270) ? LCD_W : LCD_H;
    if (!d || !out)
        return -1;
    for (vy = 0; vy < vh; vy++) {
        for (vx = 0; vx < vw; vx++) {
            int px, py;
            const uint8_t *s;
            uint8_t *o = out + 3 * (vy * vw + vx);
            if (rot == 90) { px = LCD_W - 1 - vy; py = vx; }
            else if (rot == 180) { px = LCD_W - 1 - vx; py = LCD_H - 1 - vy; }
            else if (rot == 270) { px = vy; py = LCD_H - 1 - vx; }
            else { px = vx; py = vy; }
            s = d->fb + 3 * (py * LCD_W + px);
            if (!d->on || d->sleep) { o[0] = o[1] = o[2] = 0; }
            else if (d->inv) { o[0] = (uint8_t)~s[0]; o[1] = (uint8_t)~s[1]; o[2] = (uint8_t)~s[2]; }
            else { o[0] = s[0]; o[1] = s[1]; o[2] = s[2]; }
        }
    }
    return 0;
}

int bao_display_rgb(bao_t *b, uint8_t *out, uint32_t *frames)
{
    dev_t *d = first_of(b, DEV_LCD);
    int i;
    if (!d)
        return -1;
    if (frames)
        *frames = d->frames;
    if (!out)
        return 0;
    if (!d->on || d->sleep) {
        memset(out, 0, LCD_W * LCD_H * 3);
        return 0;
    }
    for (i = 0; i < LCD_W * LCD_H * 3; i++)
        out[i] = d->inv ? (uint8_t)~d->fb[i] : d->fb[i];
    return 0;
}

int bao_mpu6050_set(bao_t *b, int16_t ax, int16_t ay, int16_t az, int16_t gx, int16_t gy, int16_t gz)
{
    devs_t *t = b->devs;
    int i, k;
    int16_t v[7];
    if (!t)
        return -1;
    v[0] = ax; v[1] = ay; v[2] = az; v[3] = -3920; v[4] = gx; v[5] = gy; v[6] = gz;
    for (i = 0; i < t->n; i++) {
        if (t->d[i].kind != DEV_MPU)
            continue;
        for (k = 0; k < 7; k++) {
            t->d[i].regs[0x3B + 2 * k] = (uint8_t)((uint16_t)v[k] >> 8);
            t->d[i].regs[0x3C + 2 * k] = (uint8_t)v[k];
        }
        return 0;
    }
    return -1;
}

void devices_save(bao_t *b)
{
    devs_t *t = b->devs;
    int i;
    if (!t)
        return;
    for (i = 0; i < t->n; i++) {
        dev_t *d = &t->d[i];
        if (d->mem && d->dirty && d->path[0]) {
            FILE *f = fopen(d->path, "wb");
            if (f) {
                fwrite(d->mem, 1, d->size, f);
                fclose(f);
            }
            d->dirty = 0;
        }
    }
}

void devices_free(bao_t *b)
{
    devs_t *t = b->devs;
    int i;
    if (!t)
        return;
    devices_save(b);
    for (i = 0; i < t->n; i++) {
        free(t->d[i].mem);
        free(t->d[i].fb);
    }
    free(t);
    b->devs = NULL;
}
