/*
 * usb.c: Corigine USB device controller and a host on the other end
 *
 * Copyright (c) 2026 Armstrong Subero
 * SPDX-License-Identifier: Apache-2.0
 *
 * Modelled from the register header and the TinyUSB corigine driver, not RTL.
 * The host enumerates, opens CDC and talks to mass storage.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "bao_int.h"

#define UDC_BASE        0x50202000u
#define UDC_LEN         0x3000u
#define DEV             0x400u
#define UICR            0x500u

#define R_DEVCONFIG     (DEV + 0x10)
#define R_EVENTCONFIG   (DEV + 0x14)
#define R_USBCMD        (DEV + 0x20)
#define R_USBSTS        (DEV + 0x24)
#define R_DCBAPLO       (DEV + 0x28)
#define R_PORTSC        (DEV + 0x30)
#define R_DOORBELL      (DEV + 0x40)
#define R_EPENABLE      (DEV + 0x60)
#define R_EPRUNNING     (DEV + 0x64)
#define R_CMDPARA0      (DEV + 0x70)
#define R_CMDPARA1      (DEV + 0x74)
#define R_CMDCTRL       (DEV + 0x78)
#define R_IMAN          (UICR + 0x00)
#define R_ERSTSZ        (UICR + 0x08)
#define R_ERSTBALO      (UICR + 0x10)
#define R_ERDPLO        (UICR + 0x18)

#define CMD_RUN         0x001u
#define CMD_SOFT_RESET  0x002u
#define CMD_INT_ENABLE  0x004u
#define STS_HALTED      0x001u
#define STS_EINT        0x008u
#define IMAN_IP         0x1u
#define IMAN_IE         0x2u
#define ERDP_EHB        0x8u

#define PORT_CCS        (1u << 0)
#define PORT_PP         (1u << 3)
#define PORT_PR         (1u << 4)
#define PORT_SPEED_FS   (1u << 10)
#define PORT_SPEED_HS   (3u << 10)
#define PORT_CSC        (1u << 17)
#define PORT_PPC        (1u << 20)
#define PORT_PRC        (1u << 21)
#define PORT_PLC        (1u << 22)
#define PORT_CEC        (1u << 23)
#define PORT_CHANGE     (PORT_CSC | PORT_PPC | PORT_PRC | PORT_PLC | PORT_CEC)

#define TRB_NORMAL      1u
#define TRB_DATA        3u
#define TRB_STATUS      4u
#define TRB_LINK        6u
#define EV_TRANSFER     32u
#define EV_PORT         34u
#define EV_SETUP        40u

#define CC_SUCCESS      1u
#define CC_TRANS_ERROR  4u
#define CC_SHORT        13u
#define CC_TAG_MISMATCH 193u

#define PEI_COUNT       10
#define MICROFRAME      (BAO_ACLK_HZ / 8000)
#define MS              (BAO_ACLK_HZ / 1000)

enum { C_IDLE, C_DATA_IN, C_DATA_OUT, C_STATUS, C_DONE };

enum {
    H_OFF, H_ATTACH, H_RESET, H_DEV64, H_ADDR, H_DEV18, H_CFG9, H_CFG, H_STR0,
    H_STRP, H_QUAL, H_SETCFG, H_LINE0, H_CODING, H_LINE1, H_LUN, H_INQUIRY,
    H_FORMATS, H_UNIT, H_CAPACITY, H_MODE, H_READY, H_FAILED
};

enum { M_IDLE, M_CBW, M_DATA_IN, M_DATA_OUT, M_CSW, M_DONE, M_FIN };

#define STALL_TRIES      4

#define DISK_MAX        65536u

typedef struct {
    uint32_t deq;
    int      dcs;
    int      mps;
    int      halted;
    int      stopped;    /* set by SET_HALT, cleared only by SET_TR_DQ_PTR or a new CONFIG_EP */
} ring_t;

typedef struct {
    uint32_t dw[4];
} trb_t;

typedef struct {
    uint32_t regs[UDC_LEN / 4];
    uint32_t usbcmd, usbsts, portsc, iman, erdp, epenable, eprunning, cmdctrl;
    int      line;

    uint32_t ev_base, ev_size, ev_enq;
    int      ev_ccs;
    trb_t    evq[64];
    int      evq_head, evq_count;

    ring_t   ep[PEI_COUNT];
    uint8_t  addr, cmd_addr;

    int      attached;
    int      hs, started;
    uint64_t at;            /* next cycle the host has something to do */
    uint64_t wait;          /* host does nothing before this cycle */

    int      phase, tag, stalled;
    uint8_t  setup[8];
    uint8_t  data[1024];
    uint32_t len, got;
    uint64_t deadline;

    uint8_t  dev_desc[18];
    uint8_t  cfg[1024];
    uint32_t cfg_len;
    int      cdc_itf, cdc_in, cdc_out;
    int      open;

    int      msc_itf, msc_in, msc_out;
    int      m_state, m_in, m_cblen, m_failed, m_sense, m_tries, m_stalls, m_clear;
    uint8_t  m_cb[16];
    uint8_t  m_buf[DISK_MAX];
    uint32_t m_len, m_got, m_tag;
    uint64_t m_deadline;
    int      disk_ready;
    uint32_t disk_blocks, disk_bs;

    uint8_t  tx[65536];     /* PC to device */
    uint32_t tx_head, tx_count;

    bao_usb_cb cb;
    void    *cb_user;
} usb_t;

static usb_t *usb(bao_t *b)
{
    if (!b->usb) {
        usb_t *u = calloc(1, sizeof(usb_t));
        if (u) {
            u->attached = 1;
            u->at = NEVER;
            u->usbsts = STS_HALTED;
        }
        b->usb = u;
    }
    return b->usb;
}

static int in_ifram(uint32_t a, uint32_t len)
{
    return a >= IFRAM_BASE && len <= IFRAM_SIZE && a - IFRAM_BASE <= IFRAM_SIZE - len;
}

static int dma_rd(bao_t *b, uint32_t a, void *dst, uint32_t len)
{
    if (!in_ifram(a, len)) {
        memset(dst, 0, len);
        bao_logf(b, BAO_LOG_ERROR, "USB controller read of 0x%08X, outside IFRAM, which its DMA cannot reach", a);
        return -1;
    }
    memcpy(dst, b->ifram + (a - IFRAM_BASE), len);
    return 0;
}

static int dma_wr(bao_t *b, uint32_t a, const void *src, uint32_t len)
{
    if (!in_ifram(a, len)) {
        bao_logf(b, BAO_LOG_ERROR, "USB controller write to 0x%08X, outside IFRAM, which its DMA cannot reach", a);
        return -1;
    }
    memcpy(b->ifram + (a - IFRAM_BASE), src, len);
    b->side++;
    return 0;
}

static void wake(bao_t *b, usb_t *u, uint64_t delay)
{
    uint64_t t = b->cycles + delay;
    if (t < u->at)
        u->at = t;
    bao_kick(b);
}

static void line_update(bao_t *b, usb_t *u)
{
    int now = (u->iman & IMAN_IP) && (u->iman & IMAN_IE) && (u->usbcmd & CMD_INT_ENABLE);
    if (now != u->line) {
        irq_level(b, "usbc", u->line, now);
        u->line = now;
    }
}

int usb_irq_line(bao_t *b)
{
    usb_t *u = b->usb;
    return u ? u->line : 0;
}

/* Event ring */

static int ev_ring_ready(bao_t *b, usb_t *u)
{
    uint32_t e[4];
    if (u->ev_base)
        return 1;
    if (!u->regs[R_ERSTBALO / 4])
        return 0;
    if (dma_rd(b, u->regs[R_ERSTBALO / 4], e, 16))
        return 0;
    u->ev_base = e[0] & ~0xFu;
    u->ev_size = e[2] & 0xFFFFu;
    u->ev_enq = u->ev_base;
    u->ev_ccs = 1;
    return u->ev_base && u->ev_size;
}

static void ev_flush(bao_t *b, usb_t *u)
{
    int posted = 0;
    if (!(u->usbcmd & CMD_RUN) || !ev_ring_ready(b, u))
        return;
    while (u->evq_count) {
        uint32_t next = u->ev_enq + 16;
        trb_t t = u->evq[u->evq_head];
        if (next >= u->ev_base + u->ev_size * 16)
            next = u->ev_base;
        if (next == (u->erdp & ~0xFu))
            break;
        t.dw[3] = (t.dw[3] & ~1u) | (uint32_t)u->ev_ccs;
        if (dma_wr(b, u->ev_enq, t.dw, 16))
            break;
        if (next == u->ev_base)
            u->ev_ccs ^= 1;
        u->ev_enq = next;
        u->evq_head = (u->evq_head + 1) % 64;
        u->evq_count--;
        posted = 1;
    }
    if (posted) {
        u->usbsts |= STS_EINT;
        u->iman |= IMAN_IP;
        u->erdp |= ERDP_EHB;
        line_update(b, u);
        b->side++;
    }
}

static void ev_post(bao_t *b, usb_t *u, uint32_t dw0, uint32_t dw1, uint32_t dw2, uint32_t dw3)
{
    trb_t *t;
    if (u->evq_count == 64) {
        bao_logf(b, BAO_LOG_WARN, "USB event dropped, the program is not reading the event ring");
        return;
    }
    t = &u->evq[(u->evq_head + u->evq_count) % 64];
    t->dw[0] = dw0; t->dw[1] = dw1; t->dw[2] = dw2; t->dw[3] = dw3;
    u->evq_count++;
    ev_flush(b, u);
}

static void ev_transfer(bao_t *b, usb_t *u, int pei, uint32_t trb_addr, uint32_t residual, uint32_t code)
{
    ev_post(b, u, trb_addr, 0, (residual & 0x1FFFFu) | (code << 24),
            (EV_TRANSFER << 10) | ((uint32_t)pei << 16));
}

/* Transfer rings */

static int ring_peek(bao_t *b, ring_t *r, trb_t *t)
{
    int hops;
    if (!r->deq)
        return 0;
    for (hops = 0; hops < 4; hops++) {
        if (dma_rd(b, r->deq, t->dw, 16))
            return 0;
        if ((int)(t->dw[3] & 1u) != r->dcs)
            return 0;
        if (((t->dw[3] >> 10) & 0x3Fu) != TRB_LINK)
            return 1;
        r->deq = t->dw[0] & ~0xFu;
        if (t->dw[3] & 2u)
            r->dcs ^= 1;
    }
    return 0;
}

/* Host: control transfers */

static void ctrl_start(bao_t *b, usb_t *u, const uint8_t *setup, const uint8_t *out)
{
    memcpy(u->setup, setup, 8);
    u->len = (uint32_t)setup[6] | ((uint32_t)setup[7] << 8);
    if (u->len > sizeof(u->data))
        u->len = sizeof(u->data);
    u->got = 0;
    u->stalled = 0;
    u->tag = (u->tag + 1) & 3;
    u->deadline = b->cycles + 1000ull * MS;
    if (u->len && out)
        memcpy(u->data, out, u->len);
    u->phase = u->len ? ((setup[0] & 0x80) ? C_DATA_IN : C_DATA_OUT) : C_STATUS;
    u->ep[0].halted = u->ep[1].halted = 0;
    ev_post(b, u,
            (uint32_t)setup[0] | ((uint32_t)setup[1] << 8) | ((uint32_t)setup[2] << 16) | ((uint32_t)setup[3] << 24),
            (uint32_t)setup[4] | ((uint32_t)setup[5] << 8) | ((uint32_t)setup[6] << 16) | ((uint32_t)setup[7] << 24),
            CC_SUCCESS << 24, (EV_SETUP << 10) | ((uint32_t)u->tag << 21));
}

/* 1 once the request has finished, 0 while it is starting or running */
static int req(bao_t *b, usb_t *u, uint8_t type, uint8_t request, uint16_t value, uint16_t index,
               uint16_t length, const uint8_t *out)
{
    uint8_t s[8];
    if (u->phase == C_DONE) {
        u->phase = C_IDLE;
        return 1;
    }
    if (u->phase != C_IDLE)
        return 0;
    s[0] = type; s[1] = request;
    s[2] = (uint8_t)value; s[3] = (uint8_t)(value >> 8);
    s[4] = (uint8_t)index; s[5] = (uint8_t)(index >> 8);
    s[6] = (uint8_t)length; s[7] = (uint8_t)(length >> 8);
    ctrl_start(b, u, s, out);
    return 0;
}

static void ep0_service(bao_t *b, usb_t *u)
{
    ring_t *r = &u->ep[0];
    trb_t t;
    while (u->phase != C_IDLE && u->phase != C_DONE && ring_peek(b, r, &t)) {
        uint32_t at = r->deq;
        uint32_t type = (t.dw[3] >> 10) & 0x3Fu;
        uint32_t len = t.dw[2] & 0x3FFFFu;
        int out = (t.dw[3] >> 16) & 1;
        int tag = (t.dw[3] >> 17) & 3;
        r->deq += 16;
        if (tag != u->tag) {
            ev_transfer(b, u, 0, at, len, CC_TAG_MISMATCH);
            continue;
        }
        if (type == TRB_DATA && !out && u->phase == C_DATA_IN) {
            uint32_t n = len;
            if (n > u->len - u->got)
                n = u->len - u->got;
            if (n)
                dma_rd(b, t.dw[0], u->data + u->got, n);
            u->got += n;
            ev_transfer(b, u, 0, at, len - n, CC_SUCCESS);
            if (u->got == u->len || len < (uint32_t)(r->mps ? r->mps : 64))
                u->phase = C_STATUS;
        } else if (type == TRB_DATA && out && u->phase == C_DATA_OUT) {
            uint32_t n = len;
            if (n > u->len - u->got)
                n = u->len - u->got;
            if (n)
                dma_wr(b, t.dw[0], u->data + u->got, n);
            u->got += n;
            ev_transfer(b, u, 0, at, len - n, n < len ? CC_SHORT : CC_SUCCESS);
            if (u->got == u->len)
                u->phase = C_STATUS;
        } else if (type == TRB_STATUS) {
            ev_transfer(b, u, 0, at, 0, CC_SUCCESS);
            if (t.dw[3] & (1u << 20))
                u->addr = u->cmd_addr;
            u->phase = C_DONE;
        } else {
            bao_logf(b, BAO_LOG_WARN, "USB EP0 transfer of type %u does not fit the request in progress", type);
            ev_transfer(b, u, 0, at, len, CC_TRANS_ERROR);
        }
    }
}

/* Host: mass storage, bulk only transport */

static void msc_start(bao_t *b, usb_t *u, const uint8_t *cb, int cblen, int in, uint32_t len)
{
    memset(u->m_cb, 0, sizeof(u->m_cb));
    memcpy(u->m_cb, cb, (size_t)cblen);
    u->m_cblen = cblen;
    u->m_in = in;
    u->m_len = len > DISK_MAX ? DISK_MAX : len;
    u->m_got = 0;
    u->m_stalls = 0;
    u->m_clear = 0;
    u->m_tag++;
    u->m_state = M_CBW;
    u->m_deadline = b->cycles + 2000ull * MS;
    wake(b, u, MICROFRAME);
}

/* 1 once the command has finished, 0 while it is starting or running */
static int scsi(bao_t *b, usb_t *u, const uint8_t *cb, int cblen, int in, uint32_t len)
{
    if (u->m_state == M_FIN) {
        u->m_state = M_IDLE;
        return 1;
    }
    if (u->m_state == M_IDLE) {
        u->m_failed = 0;
        u->m_sense = 0;
        msc_start(b, u, cb, cblen, in, len);
    }
    return 0;
}

static uint32_t msc_tx(usb_t *u, uint8_t *data, uint32_t max)
{
    uint32_t n;
    if (u->m_state == M_CBW && max >= 31) {
        memset(data, 0, 31);
        memcpy(data, "USBC", 4);
        memcpy(data + 4, &u->m_tag, 4);
        data[8] = (uint8_t)u->m_len; data[9] = (uint8_t)(u->m_len >> 8);
        data[10] = (uint8_t)(u->m_len >> 16); data[11] = (uint8_t)(u->m_len >> 24);
        data[12] = u->m_in ? 0x80 : 0x00;
        data[14] = (uint8_t)u->m_cblen;
        memcpy(data + 15, u->m_cb, 16);
        u->m_state = !u->m_len ? M_CSW : (u->m_in ? M_DATA_IN : M_DATA_OUT);
        return 31;
    }
    if (u->m_state != M_DATA_OUT)
        return 0;
    n = u->m_len - u->m_got;
    if (n > max)
        n = max;
    memcpy(data, u->m_buf + u->m_got, n);
    u->m_got += n;
    if (u->m_got == u->m_len)
        u->m_state = M_CSW;
    return n;
}

static void msc_rx(bao_t *b, usb_t *u, const uint8_t *data, uint32_t len, int mps)
{
    int csw = len == 13 && memcmp(data, "USBS", 4) == 0 && memcmp(data + 4, &u->m_tag, 4) == 0;
    if (u->m_state == M_DATA_IN && !csw) {
        uint32_t n = len;
        if (n > u->m_len - u->m_got)
            n = u->m_len - u->m_got;
        memcpy(u->m_buf + u->m_got, data, n);
        u->m_got += n;
        if (u->m_got == u->m_len || !len || (mps && len % (uint32_t)mps))
            u->m_state = M_CSW;
        return;
    }
    if ((u->m_state == M_CSW || u->m_state == M_DATA_IN) && csw) {
        if (data[12])
            u->m_failed = 1;
        u->m_state = M_DONE;
        return;
    }
    bao_logf(b, BAO_LOG_WARN, "USB disk sent %u bytes the host was not waiting for", len);
}

/* A failed command is followed by REQUEST SENSE, as a PC does, so the device clears its error */
static void msc_poll(bao_t *b, usb_t *u)
{
    static const uint8_t sense[6] = { 0x03, 0, 0, 0, 18, 0 };
    if (u->m_state >= M_CBW && u->m_state <= M_CSW) {
        int in_halt = u->msc_in >= 0 && u->ep[u->msc_in].halted;
        int out_halt = u->msc_out >= 0 && u->ep[u->msc_out].halted;
        if (u->m_clear || (in_halt && !out_halt && u->m_stalls < STALL_TRIES)) {
            /* A stalled Bulk-In pipe: clear the halt as a PC does, then read the status */
            if (!u->m_clear) {
                u->m_clear = 1;
                u->m_stalls++;
            }
            if (req(b, u, 0x02, 1, 0, (uint16_t)(0x80 | (u->msc_in / 2)), 0, NULL)) {
                u->m_clear = 0;
                if (u->m_state != M_CSW)
                    u->m_state = M_CSW;
            }
            wake(b, u, MICROFRAME);
        } else if (in_halt || out_halt) {
            bao_logf(b, BAO_LOG_WARN, "USB disk endpoint stalled, which this host does not recover from");
            u->m_failed = 1;
            u->m_state = M_FIN;
            wake(b, u, MICROFRAME);
        } else if (b->cycles >= u->m_deadline) {
            bao_logf(b, BAO_LOG_WARN, "USB disk did not finish command 0x%02X within two seconds", u->m_cb[0]);
            u->m_failed = 1;
            u->m_state = M_FIN;
            wake(b, u, MICROFRAME);
        } else if (u->m_deadline < u->at) {
            u->at = u->m_deadline;
        }
    }
    if (u->m_state != M_DONE)
        return;
    if (u->m_failed && !u->m_sense) {
        u->m_sense = 1;
        msc_start(b, u, sense, 6, 1, 18);
        return;
    }
    u->m_state = M_FIN;
    wake(b, u, MICROFRAME);
}

/* Host: bulk and interrupt endpoints */

static void host_rx(bao_t *b, usb_t *u, int pei, const uint8_t *data, uint32_t len)
{
    if (pei == u->msc_in)
        msc_rx(b, u, data, len, u->ep[pei].mps);
    else if (pei == u->cdc_in && u->cb && len)
        u->cb(u->cb_user, data, (int)len);
}

static uint32_t host_tx(usb_t *u, int pei, uint8_t *data, uint32_t max)
{
    uint32_t n = 0;
    if (pei == u->msc_out)
        return msc_tx(u, data, max);
    if (pei != u->cdc_out || !u->open)
        return 0;
    while (n < max && u->tx_count) {
        data[n++] = u->tx[u->tx_head];
        u->tx_head = (u->tx_head + 1) % sizeof(u->tx);
        u->tx_count--;
    }
    return n;
}

static void ep_service(bao_t *b, usb_t *u)
{
    static uint8_t buf[4096];
    int pei;
    trb_t t;
    if (u->hs < H_SETCFG)
        return;
    for (pei = 2; pei < PEI_COUNT; pei++) {
        ring_t *r = &u->ep[pei];
        if (!(u->epenable & (1u << pei)) || r->halted || r->stopped)
            continue;
        while (ring_peek(b, r, &t)) {
            uint32_t at = r->deq;
            uint32_t len = t.dw[2] & 0x3FFFFu;
            if (len > sizeof(buf))
                len = sizeof(buf);
            if (!(pei & 1)) {
                if (len)
                    dma_rd(b, t.dw[0], buf, len);
                r->deq += 16;
                host_rx(b, u, pei, buf, len);
                ev_transfer(b, u, pei, at, 0, CC_SUCCESS);
            } else {
                uint32_t n = host_tx(u, pei, buf, len);
                if (!n)
                    break;
                dma_wr(b, t.dw[0], buf, n);
                r->deq += 16;
                ev_transfer(b, u, pei, at, len - n, n < len ? CC_SHORT : CC_SUCCESS);
            }
        }
    }
}

/* Host: attach, enumeration and the serial port */

static void host_fail(bao_t *b, usb_t *u, const char *what)
{
    bao_logf(b, BAO_LOG_WARN, "USB host gave up: %s", what);
    u->hs = H_FAILED;
    u->phase = C_IDLE;
}

static void cfg_parse(usb_t *u)
{
    uint32_t i = 0;
    int cls = -1, itf = 0;
    u->cdc_itf = -1;
    u->cdc_in = u->cdc_out = -1;
    u->msc_itf = -1;
    u->msc_in = u->msc_out = -1;
    while (i + 2 <= u->cfg_len && u->cfg[i] >= 2) {
        const uint8_t *d = &u->cfg[i];
        if (d[1] == 4 && d[0] >= 9) {
            itf = d[2];
            cls = d[5];
            if (cls == 0x02 && u->cdc_itf < 0)
                u->cdc_itf = itf;
            if (cls == 0x08 && d[6] == 0x06 && d[7] == 0x50 && u->msc_itf < 0)
                u->msc_itf = itf;
        } else if (d[1] == 5 && d[0] >= 7 && (d[3] & 3) == 2 && (cls == 0x0A || (cls == 0x08 && itf == u->msc_itf))) {
            int pei = (d[2] & 0x0F) * 2 + ((d[2] & 0x80) ? 0 : 1);
            if (pei < PEI_COUNT && cls == 0x0A) {
                if (d[2] & 0x80) u->cdc_in = pei;
                else u->cdc_out = pei;
            } else if (pei < PEI_COUNT) {
                if (d[2] & 0x80) u->msc_in = pei;
                else u->msc_out = pei;
            }
        }
        i += d[0];
    }
}

static int host_step(bao_t *b, usb_t *u)
{
    static const uint8_t coding[7] = { 0x00, 0xC2, 0x01, 0x00, 0, 0, 8 };
    static const uint8_t inquiry[6] = { 0x12, 0, 0, 0, 36, 0 };
    static const uint8_t unit[6] = { 0x00, 0, 0, 0, 0, 0 };
    static const uint8_t capacity[10] = { 0x25, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
    /* Windows asks for more than these return, which makes a device stall Bulk-In */
    static const uint8_t formats[10] = { 0x23, 0, 0, 0, 0, 0, 0, 0, 252, 0 };
    static const uint8_t mode[6] = { 0x1A, 0, 0x3F, 0, 192, 0 };
    int disk = 0;
    switch (u->hs) {
    case H_ATTACH:
        u->portsc = PORT_CCS | PORT_PP | PORT_CSC | PORT_PPC;
        ev_post(b, u, 0, 0, CC_SUCCESS << 24, EV_PORT << 10);
        u->hs = H_RESET;
        u->wait = b->cycles + 20ull * MS;
        return 1;
    case H_RESET:
        u->portsc = PORT_CCS | PORT_PP | PORT_PRC |
                    ((u->regs[R_DEVCONFIG / 4] & 0xFu) == 3 ? PORT_SPEED_HS : PORT_SPEED_FS);
        u->addr = 0;
        ev_post(b, u, 0, 0, CC_SUCCESS << 24, EV_PORT << 10);
        u->hs = H_DEV64;
        u->wait = b->cycles + 10ull * MS;
        return 1;
    case H_DEV64:
        if (!req(b, u, 0x80, 6, 0x0100, 0, 64, NULL)) return 0;
        if (u->stalled || u->got < 8) { host_fail(b, u, "no device descriptor"); return 0; }
        u->hs = H_ADDR;
        break;
    case H_ADDR:
        if (!req(b, u, 0x00, 5, 7, 0, 0, NULL)) return 0;
        if (u->stalled || u->addr != 7) { host_fail(b, u, "address not taken"); return 0; }
        u->hs = H_DEV18;
        u->wait = b->cycles + 2ull * MS;
        return 1;
    case H_DEV18:
        if (!req(b, u, 0x80, 6, 0x0100, 0, 18, NULL)) return 0;
        if (u->stalled || u->got != 18) { host_fail(b, u, "short device descriptor"); return 0; }
        memcpy(u->dev_desc, u->data, 18);
        u->hs = H_CFG9;
        break;
    case H_CFG9:
        if (!req(b, u, 0x80, 6, 0x0200, 0, 9, NULL)) return 0;
        if (u->stalled || u->got != 9) { host_fail(b, u, "no configuration descriptor"); return 0; }
        u->cfg_len = (uint32_t)u->data[2] | ((uint32_t)u->data[3] << 8);
        if (u->cfg_len > sizeof(u->cfg)) u->cfg_len = sizeof(u->cfg);
        u->hs = H_CFG;
        break;
    case H_CFG:
        if (!req(b, u, 0x80, 6, 0x0200, 0, (uint16_t)u->cfg_len, NULL)) return 0;
        if (u->stalled || u->got != u->cfg_len) { host_fail(b, u, "short configuration descriptor"); return 0; }
        memcpy(u->cfg, u->data, u->cfg_len);
        cfg_parse(u);
        u->hs = H_STR0;
        break;
    case H_STR0:
        if (!req(b, u, 0x80, 6, 0x0300, 0, 255, NULL)) return 0;
        u->hs = (!u->stalled && u->dev_desc[15]) ? H_STRP : H_QUAL;
        break;
    case H_STRP:
        if (!req(b, u, 0x80, 6, (uint16_t)(0x0300 | u->dev_desc[15]), 0x0409, 255, NULL)) return 0;
        u->hs = H_QUAL;
        break;
    case H_QUAL:
        if (!req(b, u, 0x80, 6, 0x0600, 0, 10, NULL)) return 0;
        u->hs = H_SETCFG;
        break;
    case H_SETCFG:
        if (!req(b, u, 0x00, 9, u->cfg[5], 0, 0, NULL)) return 0;
        if (u->stalled) { host_fail(b, u, "configuration refused"); return 0; }
        bao_logf(b, BAO_LOG_INFO, "USB host configured the device %02x%02x:%02x%02x at address %u",
                 u->dev_desc[9], u->dev_desc[8], u->dev_desc[11], u->dev_desc[10], u->addr);
        disk = u->msc_itf >= 0 && u->msc_in >= 0 && u->msc_out >= 0;
        u->hs = u->cdc_itf >= 0 ? H_LINE0 : (disk ? H_LUN : H_READY);
        break;
    case H_LINE0:
        if (!req(b, u, 0x21, 0x22, 0, (uint16_t)u->cdc_itf, 0, NULL)) return 0;
        u->hs = H_CODING;
        u->wait = b->cycles + 50ull * MS;
        return 1;
    case H_CODING:
        if (!req(b, u, 0x21, 0x20, 0, (uint16_t)u->cdc_itf, 7, coding)) return 0;
        u->hs = H_LINE1;
        break;
    case H_LINE1:
        if (!req(b, u, 0x21, 0x22, 3, (uint16_t)u->cdc_itf, 0, NULL)) return 0;
        u->open = 1;
        disk = u->msc_itf >= 0 && u->msc_in >= 0 && u->msc_out >= 0;
        u->hs = disk ? H_LUN : H_READY;
        u->wait = b->cycles + 5ull * MS;
        return 1;
    case H_LUN:
        if (!req(b, u, 0xA1, 0xFE, 0, (uint16_t)u->msc_itf, 1, NULL)) return 0;
        u->hs = H_INQUIRY;
        break;
    case H_INQUIRY:
        if (!scsi(b, u, inquiry, 6, 1, 36)) return 0;
        if (u->m_failed) { bao_logf(b, BAO_LOG_WARN, "USB disk did not answer INQUIRY"); u->hs = H_READY; break; }
        u->m_tries = 0;
        u->hs = H_FORMATS;
        break;
    case H_FORMATS:
        if (!scsi(b, u, formats, 10, 1, 252)) return 0;
        u->hs = H_UNIT;
        break;
    case H_UNIT:
        if (!scsi(b, u, unit, 6, 0, 0)) return 0;
        if (u->m_failed && ++u->m_tries < 20) {
            u->wait = b->cycles + 100ull * MS;
            return 1;
        }
        if (u->m_failed) { bao_logf(b, BAO_LOG_WARN, "USB disk never became ready"); u->hs = H_READY; break; }
        u->hs = H_CAPACITY;
        break;
    case H_CAPACITY:
        if (!scsi(b, u, capacity, 10, 1, 8)) return 0;
        if (!u->m_failed && u->m_got == 8) {
            u->disk_blocks = (((uint32_t)u->m_buf[0] << 24) | ((uint32_t)u->m_buf[1] << 16) |
                              ((uint32_t)u->m_buf[2] << 8) | u->m_buf[3]) + 1;
            u->disk_bs = ((uint32_t)u->m_buf[4] << 24) | ((uint32_t)u->m_buf[5] << 16) |
                         ((uint32_t)u->m_buf[6] << 8) | u->m_buf[7];
            u->disk_ready = u->disk_bs != 0;
            bao_logf(b, BAO_LOG_INFO, "USB host found a disk of %u blocks of %u bytes", u->disk_blocks, u->disk_bs);
        } else {
            bao_logf(b, BAO_LOG_WARN, "USB disk did not give its capacity");
        }
        u->hs = H_MODE;
        break;
    case H_MODE:
        if (!scsi(b, u, mode, 6, 1, 192)) return 0;
        u->hs = H_READY;
        break;
    default:
        return 0;
    }
    u->wait = b->cycles + MICROFRAME;
    return 1;
}

static void host_run(bao_t *b, usb_t *u)
{
    int active;
    if (!(u->usbcmd & CMD_RUN) || u->hs == H_OFF)
        return;
    ev_flush(b, u);
    for (;;) {
        ep0_service(b, u);
        active = u->phase != C_IDLE && u->phase != C_DONE;
        if (active || b->cycles < u->wait)
            break;
        if (!host_step(b, u))
            break;
    }
    if (u->hs == H_FAILED)
        return;
    active = u->phase != C_IDLE && u->phase != C_DONE;
    if (active && b->cycles >= u->deadline) {
        host_fail(b, u, "the device did not answer a control request within one second");
        return;
    }
    if (active && u->deadline < u->at)
        u->at = u->deadline;
    if (!active && b->cycles < u->wait && u->wait < u->at)
        u->at = u->wait;
    ep_service(b, u);
    msc_poll(b, u);
}

/* Controller commands */

static void command(bao_t *b, usb_t *u, uint32_t v)
{
    uint32_t type = (v >> 4) & 0xFu;
    uint32_t p0 = u->regs[R_CMDPARA0 / 4];
    int pei;
    switch (type) {
    case 0:    /* INIT_EP0 */
        u->ep[0].deq = p0 & ~0xFu;
        u->ep[0].dcs = (int)(p0 & 1u);
        u->ep[0].mps = 64;
        break;
    case 1:    /* UPDATE_EP0_MPS */
        u->ep[0].mps = (int)(p0 >> 16);
        break;
    case 2:    /* SET_ADDR */
        u->cmd_addr = (uint8_t)p0;
        break;
    case 4:    /* CONFIG_EP */
        for (pei = 2; pei < PEI_COUNT; pei++) {
            uint32_t cx[4];
            if (!(p0 & (1u << pei)))
                continue;
            if (dma_rd(b, u->regs[R_DCBAPLO / 4] + (uint32_t)(pei - 2) * 16, cx, 16))
                continue;
            u->ep[pei].deq = cx[2] & ~0xFu;
            u->ep[pei].dcs = (int)(cx[2] & 1u);
            u->ep[pei].mps = (int)(cx[1] >> 16);
            u->ep[pei].halted = 0;
            u->ep[pei].stopped = 0;
            u->epenable |= 1u << pei;
        }
        break;
    case 5:    /* SET_HALT */
        /* halted endpoint leaves EPRUNNING, context gets the dequeue position */
        for (pei = 0; pei < PEI_COUNT; pei++) {
            if (!(p0 & (1u << pei)))
                continue;
            u->ep[pei].halted = 1;
            if (pei >= 2) {
                uint32_t dq = u->ep[pei].deq | (uint32_t)u->ep[pei].dcs;
                u->ep[pei].stopped = 1;
                u->eprunning &= ~(1u << pei);
                dma_wr(b, u->regs[R_DCBAPLO / 4] + (uint32_t)(pei - 2) * 16 + 8, &dq, 4);
            }
        }
        if ((p0 & 3u) && u->phase != C_IDLE && u->phase != C_DONE) {
            u->stalled = 1;
            u->phase = C_DONE;
        }
        break;
    case 6:    /* CLEAR_HALT */
        /* does not restart the endpoint, SET_TR_DQ_PTR does */
        for (pei = 0; pei < PEI_COUNT; pei++)
            if (p0 & (1u << pei))
                u->ep[pei].halted = 0;
        break;
    case 7:    /* RESET_EP */
        for (pei = 0; pei < PEI_COUNT; pei++)
            if (p0 & (1u << pei))
                u->ep[pei].halted = u->ep[pei].stopped = 0;
        break;
    case 9:    /* SET_TR_DQ_PTR */
        for (pei = 2; pei < PEI_COUNT; pei++) {
            uint32_t cx[4];
            if (!(p0 & (1u << pei)) || !(u->epenable & (1u << pei)))
                continue;
            if (dma_rd(b, u->regs[R_DCBAPLO / 4] + (uint32_t)(pei - 2) * 16, cx, 16))
                continue;
            u->ep[pei].deq = cx[2] & ~0xFu;
            u->ep[pei].dcs = (int)(cx[2] & 1u);
            u->ep[pei].stopped = 0;
            u->eprunning |= 1u << pei;
        }
        wake(b, u, MICROFRAME);
        break;
    case 8:    /* STOP_EP */
        u->eprunning &= ~p0;
        break;
    default:
        break;
    }
    u->cmdctrl = v & ~0x000F0001u;
    wake(b, u, MICROFRAME);
}

static void controller_reset(bao_t *b, usb_t *u)
{
    int was = u->line;
    memset(u->regs + DEV / 4, 0, 0x200);
    u->usbcmd = 0;
    u->usbsts = STS_HALTED;
    u->portsc = 0;
    u->iman = u->erdp = u->epenable = u->eprunning = u->cmdctrl = 0;
    u->ev_base = u->ev_size = u->ev_enq = 0;
    u->evq_head = u->evq_count = 0;
    memset(u->ep, 0, sizeof(u->ep));
    u->addr = u->cmd_addr = 0;
    u->hs = H_OFF;
    u->phase = C_IDLE;
    u->open = 0;
    u->m_state = M_IDLE;
    u->disk_ready = 0;
    u->disk_blocks = u->disk_bs = 0;
    u->at = NEVER;
    u->line = 0;
    if (was)
        irq_level(b, "usbc", 1, 0);
}

int usb_read(bao_t *b, uint32_t addr, uint32_t *v)
{
    usb_t *u = usb(b);
    uint32_t off = addr - UDC_BASE;
    if (!u || off >= UDC_LEN)
        return -1;
    switch (off) {
    case R_USBCMD:    *v = u->usbcmd; break;
    case R_USBSTS:    *v = u->usbsts; break;
    case R_PORTSC:    *v = u->portsc; break;
    case R_CMDCTRL:   *v = u->cmdctrl; break;
    case R_EPENABLE:  *v = u->epenable; break;
    case R_EPRUNNING: *v = u->eprunning; break;
    case R_IMAN:      *v = u->iman; break;
    case R_ERDPLO:    *v = u->erdp; break;
    default:          *v = u->regs[off / 4]; break;
    }
    return 0;
}

int usb_write(bao_t *b, uint32_t addr, uint32_t v)
{
    usb_t *u = usb(b);
    uint32_t off = addr - UDC_BASE;
    if (!u || off >= UDC_LEN)
        return -1;
    b->side++;
    switch (off) {
    case R_USBCMD:
        if (v & CMD_SOFT_RESET) {
            controller_reset(b, u);
            break;
        }
        if ((v & CMD_RUN) && !(u->usbcmd & CMD_RUN)) {
            u->usbsts &= ~STS_HALTED;
            if (u->attached) {
                u->hs = H_ATTACH;
                u->wait = b->cycles + 5ull * MS;
                wake(b, u, 5ull * MS);
            }
        } else if (!(v & CMD_RUN) && (u->usbcmd & CMD_RUN)) {
            u->usbsts |= STS_HALTED;
            u->hs = H_OFF;
            u->phase = C_IDLE;
            u->open = 0;
            u->m_state = M_IDLE;
            u->disk_ready = 0;
            u->portsc = 0;
        }
        u->usbcmd = v & ~CMD_SOFT_RESET;
        line_update(b, u);
        break;
    case R_USBSTS:
        u->usbsts &= ~(v & (STS_EINT | 0x4u));
        break;
    case R_PORTSC:
        u->portsc &= ~(v & PORT_CHANGE);
        break;
    case R_CMDCTRL:
        if (v & 1u)
            command(b, u, v);
        else
            u->cmdctrl = v;
        break;
    case R_DOORBELL:
        if ((v & 0x1Fu) >= 2 && (v & 0x1Fu) < PEI_COUNT && !u->ep[v & 0x1Fu].stopped)
            u->eprunning |= 1u << (v & 0x1Fu);
        wake(b, u, MICROFRAME);
        break;
    case R_EPENABLE:
        u->epenable &= ~v;
        u->eprunning &= ~v;
        break;
    case R_IMAN:
        u->iman = (u->iman & ~IMAN_IE) | (v & IMAN_IE);
        if (v & IMAN_IP)
            u->iman &= ~IMAN_IP;
        line_update(b, u);
        break;
    case R_ERSTBALO:
        u->regs[off / 4] = v;
        u->ev_base = 0;
        break;
    case R_ERDPLO:
        u->erdp = (v & ~0xFu) | (u->erdp & ERDP_EHB & ~v);
        if (u->evq_count)
            wake(b, u, 0);
        break;
    default:
        u->regs[off / 4] = v;
        break;
    }
    return 0;
}

void usb_update(bao_t *b)
{
    usb_t *u = b->usb;
    if (!u || b->cycles < u->at)
        return;
    u->at = NEVER;
    host_run(b, u);
}

uint64_t usb_next_event(bao_t *b)
{
    usb_t *u = b->usb;
    return u ? u->at : NEVER;
}

void usb_reset(bao_t *b)
{
    usb_t *u = b->usb;
    if (!u)
        return;
    memset(u->regs, 0, sizeof(u->regs));
    u->line = 0;
    controller_reset(b, u);
    u->tx_head = u->tx_count = 0;
}

void usb_free(bao_t *b)
{
    free(b->usb);
    b->usb = NULL;
}

BAO_API void bao_on_usb_serial(bao_t *b, bao_usb_cb cb, void *user)
{
    usb_t *u = usb(b);
    if (u) {
        u->cb = cb;
        u->cb_user = user;
    }
}

BAO_API void bao_usb_serial_rx(bao_t *b, const uint8_t *data, int len)
{
    usb_t *u = usb(b);
    int i;
    if (!u)
        return;
    for (i = 0; i < len && u->tx_count < sizeof(u->tx); i++) {
        u->tx[(u->tx_head + u->tx_count) % sizeof(u->tx)] = data[i];
        u->tx_count++;
    }
    wake(b, u, MICROFRAME);
}

BAO_API void bao_usb_attach(bao_t *b, int on)
{
    usb_t *u = usb(b);
    if (u)
        u->attached = on;
}

BAO_API int bao_usb_serial_open(bao_t *b)
{
    usb_t *u = b->usb;
    return u ? u->open : 0;
}

/* Disk blocks: 0 until the host has read the capacity */
BAO_API uint32_t bao_usb_disk_blocks(bao_t *b)
{
    usb_t *u = b->usb;
    return u && u->disk_ready ? u->disk_blocks : 0;
}

BAO_API uint32_t bao_usb_disk_block_size(bao_t *b)
{
    usb_t *u = b->usb;
    return u && u->disk_ready ? u->disk_bs : 0;
}

static int disk_op(bao_t *b, uint32_t lba, uint32_t count, const uint8_t *data)
{
    usb_t *u = b->usb;
    uint8_t cb[10] = { 0 };
    if (!u || !u->disk_ready || u->hs != H_READY || (u->m_state != M_IDLE && u->m_state != M_FIN))
        return -1;
    if (!count || count > DISK_MAX / u->disk_bs || lba >= u->disk_blocks || count > u->disk_blocks - lba)
        return -1;
    cb[0] = data ? 0x2A : 0x28;
    cb[2] = (uint8_t)(lba >> 24); cb[3] = (uint8_t)(lba >> 16); cb[4] = (uint8_t)(lba >> 8); cb[5] = (uint8_t)lba;
    cb[7] = (uint8_t)(count >> 8); cb[8] = (uint8_t)count;
    if (data)
        memcpy(u->m_buf, data, count * u->disk_bs);
    u->m_failed = 0;
    u->m_sense = 0;
    msc_start(b, u, cb, 10, data == NULL, count * u->disk_bs);
    return 0;
}

/* Start a transfer of at most 64 KB. 0 if started, -1 if the disk is not ready, busy or out of range */
BAO_API int bao_usb_disk_read(bao_t *b, uint32_t lba, uint32_t count)
{
    return disk_op(b, lba, count, NULL);
}

BAO_API int bao_usb_disk_write(bao_t *b, uint32_t lba, uint32_t count, const uint8_t *data)
{
    return data ? disk_op(b, lba, count, data) : -1;
}

/* 1 while a transfer runs, 0 once it has finished, -1 if it failed */
BAO_API int bao_usb_disk_busy(bao_t *b)
{
    usb_t *u = b->usb;
    if (!u || u->m_state == M_IDLE || u->m_state == M_FIN)
        return u && u->m_failed ? -1 : 0;
    return 1;
}

/* Bytes of the last finished read */
BAO_API uint32_t bao_usb_disk_data(bao_t *b, uint8_t *out, uint32_t len)
{
    usb_t *u = b->usb;
    if (!u || u->m_state == M_CBW || u->m_state == M_DATA_IN || u->m_state == M_DATA_OUT || u->m_state == M_CSW)
        return 0;
    if (len > u->m_got)
        len = u->m_got;
    memcpy(out, u->m_buf, len);
    return len;
}
