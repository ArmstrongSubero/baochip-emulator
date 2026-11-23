/*
 * bao_int.h: internal state shared by the emulator modules
 *
 * Copyright (c) 2026 Armstrong Subero
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef BAO_INT_H
#define BAO_INT_H

#include <stdint.h>
#include <stddef.h>
#include "baoemu.h"

#define RRAM_BASE      0x60000000u
#define RRAM_SIZE      0x00400000u
#define SRAM_BASE      0x61000000u
#define SRAM_SIZE      0x00200000u
#define IFRAM_BASE     0x50000000u
#define IFRAM_SIZE     0x00040000u

#define BOOT_END       0x60060000u
#define SECURITY_BASE  0x603DA000u

#define NEVER          UINT64_MAX

#define MSTATUS_SIE    0x00000002u
#define MSTATUS_MIE    0x00000008u
#define MSTATUS_SPIE   0x00000020u
#define MSTATUS_MPIE   0x00000080u
#define MSTATUS_SPP    0x00000100u
#define MSTATUS_MPP    0x00001800u
#define MSTATUS_MPRV   0x00020000u
#define MSTATUS_SUM    0x00040000u
#define MSTATUS_MXR    0x00080000u
#define MIP_SSIP       0x00000002u
#define MIP_STIP       0x00000020u
#define MIP_SEIP       0x00000200u
#define MIP_MEIP       0x00000800u
#define MIE_MEIE       0x00000800u

#define PRIV_U         0u
#define PRIV_S         1u
#define PRIV_M         3u

/* rd: how CFG reads back its datasize field (UDMA_DS_*) */
typedef struct {
    uint32_t saddr, size, cfg;
    uint32_t cur, left;
    int      en, rd;
} udma_ch_t;

typedef struct {
    udma_ch_t rx, tx;
    uint32_t setup, irq_en, error;
    uint8_t  fifo[256];
    int      fifo_head, fifo_count;
    uint8_t  host_q[4096];
    int      host_head, host_count;
    uint64_t tx_done_at;
    uint64_t rx_next_at;
    int      tx_busy;
    uint8_t  tx_pending[0x10000];
    int      tx_pending_len;
} uart_t;

typedef struct {
    udma_ch_t rx, tx, cmd;
    int      busy;
    uint64_t done_at;
    uint32_t clkdiv, cpol, cpha;
    int      cs;
    int      cs_active;
    uint8_t  rx_stage[0x10000];
    int      rx_stage_len;
    int      tx_used, rx_used;
    uint32_t events;
} spim_t;

typedef struct {
    udma_ch_t rx, tx, cmd;
    int      busy;
    uint64_t done_at;
    uint32_t div, setup, status, ack;
    uint8_t  rx_stage[0x10000];
    int      rx_stage_len;
    int      tx_used, rx_used;
    uint32_t events;
    int      nack, addressed;
} i2c_t;

typedef struct {
    uint32_t soft, edge, polarity, pending, enable;
} irqarr_t;

typedef struct {
    int      active, sawtooth, pending, armed, loaded;
    uint32_t tick, start, end, comp[4], op[4];
    int      val[4], second[4];
    uint64_t origin, done;
    uint64_t gated_at;    /* cycle CH_EN stopped its clock, or NEVER */
} pwm_slice_t;

#define MDMA_CHANNELS 8

/* PL230 registers and the request block of mdma.sv */
typedef struct {
    uint32_t cfg, ctrl_base, useburst, reqmask, enable, prialt, priority, err;
    uint32_t evsel[MDMA_CHANNELS], cr[MDMA_CHANNELS];
    uint8_t  quelen[MDMA_CHANNELS], queof[MDMA_CHANNELS];
    uint32_t q_burst, q_single;    /* requests that arrived while serving */
    int      busy;
} mdma_t;

/* 1024 words of FIFO RAM plus the buffered FIFO's output register */
#define MBOX_FIFO_WORDS 1025

typedef struct {
    uint32_t d[MBOX_FIFO_WORDS];
    uint32_t head, cnt, last;
} mbfifo_t;

typedef struct {
    int state, in_progress, ack, w_abort;
} mbabort_t;

/* the APB client of mbox_client.v and MB_CLIENT in cram_axi.sv */
typedef struct {
    uint32_t wdata, enable, pending, control, done;
    int      w_pending, tx_err, rx_err, err_was;
    mbabort_t ab;
} mbclient_t;

/* MAILBOX in cram_axi.sv and the two clients that can face it */
typedef struct {
    mbfifo_t tx, rx;
    uint32_t wdata, enable, pending, control, done, loopback;
    int      tx_err, rx_err, err_was;
    mbabort_t ab;
    mbclient_t cl[2];    /* 0 MBOX_APB, 1 MB_CLIENT */
} mbox_t;

/* one TLB line as MmuPlugin keeps it */
typedef struct {
    uint8_t  valid, exc, super, r, w, x, u;
    uint16_t vpn1, vpn0, ppn1, ppn0;
} tlb_t;

typedef struct sym {
    uint32_t addr;
    char    *name;
} sym_t;

struct bao {
    uint32_t x[32];
    uint32_t pc;

    uint32_t mstatus, mie, mtvec, mepc, mcause, mtval, mscratch;
    uint32_t mim, sim;
    uint32_t priv, misa, medeleg, mideleg, mip_soft;
    uint32_t stvec, sscratch, sepc, scause, stval, satp;
    int      vm_i, vm_d;
    tlb_t    tlb[2][8];
    int      tlb_next[2];
    int      wfi;
    uint64_t wfi_since;     /* cycle the core entered WFI */
    int      wfi_gate;      /* clkvex will be gated in this WFI */
    uint64_t vex_gated;     /* cycles clkvex spent gated in earlier WFIs */
    int      reserved_valid;

    /* pipeline timing in decode slots */
    uint64_t slot;          /* first slot the next instruction may leave decode */
    uint64_t fetch_ok;      /* first slot the fetched instruction is in decode */
    uint64_t last_issue;    /* slot the previous instruction left decode */
    uint64_t last_mem;      /* slot the last load or store left decode */
    uint64_t hz_ready[2];   /* first slot a reader of hz_rd may leave decode */
    uint8_t  hz_rd[2];      /* rd of the last two instructions, 32 for none */
    uint8_t  redirected;    /* the fetch was redirected before this instruction */
    uint64_t ff_pipe;       /* pipeline state when the fast forward snapshot was taken */
    uint64_t wb_hold;       /* slot after a load or AMO that held writeback, 0 for none */
    uint32_t hold_old;      /* its rd before it wrote */
    uint8_t  hold_rd;

    uint64_t cycles;
    uint64_t instret;
    uint64_t limit;
    uint64_t next_event;

    uint8_t *rram, *sram, *ifram;
    uint8_t *rdmap[65536];
    uint8_t *wrmap[65536];

    int      stop;
    char     stop_msg[512];
    int      guard_on;
    int      stop_on_exception;
    int      uart_instant;

    uint32_t rrc_cr;
    int      rrc_staged;
    uint32_t rrc_page;
    uint8_t  rrc_buf[32];
    uint8_t  rrc_mask[32];
    uint32_t rrc_commits;

    irqarr_t irq[20];

    uint32_t tt_cpt;
    uint64_t tt_base_cycle;
    uint64_t tt_base_ms;
    uint64_t tt_target;
    uint32_t tt_target1, tt_target0;
    uint32_t tt_pending, tt_enable;

    uint32_t t0_load, t0_reload, t0_en, t0_latched;
    uint64_t t0_start;
    uint32_t t0_start_val;
    uint32_t t0_pending, t0_enable;
    uint64_t t0_seen;

    uint32_t hb_count;

    uint32_t wdt_load, wdt_ctrl, wdt_ris;
    int      wdt_locked;
    uint64_t wdt_base;
    uint32_t wdt_resets;
    int      sw_reset;    /* RCURST0 or RCURST1 written, taken at the next event */

    uint32_t ao_cr, ao_wkup, ao_rstcrmask, ao_pmucrpd, ao_fr;
    int      pd_state;      /* 0 running, 1 PDAR taken, 2 held in power down, 3 waking */
    uint64_t pd_at;         /* cycle of the next power down step */
    uint32_t pd_count;
    uint32_t rtc_lr, rtc_cr, rtc_frozen, rtc_fd, rtc_mr, rtc_imsc, rtc_ris, rtc_seen;
    uint32_t at_cfg[4], at_cmp[4], at_base_val[4];
    uint64_t at_base[4];
    uint64_t rtc_base;

    uint32_t udma_cg;
    uart_t   uart[4];
    spim_t   spim[4];
    i2c_t    i2c[4];
    udma_ch_t adc_rx;
    uint32_t udma_evt, udma_rst;
    uint32_t adc_cr;
    uint64_t adc_done_at;
    uint32_t adc_mv[4];

    uint32_t iox[0x1000 / 4];
    uint32_t gpio_ext[6];
    uint32_t gpio_ext_set[6];
    uint32_t gpio_last_out[6], gpio_last_oe[6];
    uint32_t iox_last, iox_fr;

    uint32_t pwm[0x200 / 4];
    pwm_slice_t pwms[4];

    mdma_t   mdma;
    mbox_t   mbox;

    uint8_t  aes_rk[240];
    int      aes_nr;
    uint32_t aes_aram[4][4];    /* AES RAM words I, IV, O and OFB_X (aes.sv 67 to 71) */
    uint32_t hash_ram[768];     /* the hash engine's own RAM (combohasha.sv 828 to 871) */
    uint64_t rng_state;
    uint32_t rng_count;
    int      rng_on, rng_seg;
    uint64_t rng_t0;
    uint32_t rng_target, rng_done_blocks, rng_clr_blocks;

    uint32_t *mmio_pages[65536];
    uint8_t   warned[(1u << 20) / 8];

    void    *devs;
    void    *bio;
    void    *pke;
    void    *usb;
    uint32_t bio_out, bio_oe;
    int      bio_mul;

    sym_t   *syms;
    int      nsyms;
    char     where_buf[160];
    uint32_t entry;

    int      ff_on;
    uint32_t ff_pc;
    uint32_t ff_x[32];
    uint64_t ff_cycles, ff_instret, ff_side, side;
    uint32_t ff_tick;
    int      ff_ttdep, ff_hbdep, ff_block;
    uint64_t ff_skipped;

    bao_log_cb    log_cb;    void *log_user;
    bao_uart_cb   uart_cb;   void *uart_user;
    bao_gpio_cb   gpio_cb;   void *gpio_user;
    bao_spi_cb    spi_cb;    bao_spi_cs_cb spi_cs_cb; void *spi_user;
    bao_i2c_cb    i2c_cb;    void *i2c_user;
};

void bao_logf(bao_t *b, int level, const char *fmt, ...);
void bao_halt(bao_t *b, int reason, const char *fmt, ...);
void bao_kick(bao_t *b);

int  cpu_step_block(bao_t *b);
void cpu_reset(bao_t *b);
int  cpu_irq_pending(bao_t *b, int *target);
int  cpu_wake(bao_t *b);
void cpu_idle(bao_t *b);
void cpu_wfi_enter(bao_t *b);
void cpu_wfi_leave(bao_t *b);
uint64_t vex_now(bao_t *b);
void ao_por(bao_t *b);
uint64_t cpu_pipe_state(const bao_t *b);

int  bus_read(bao_t *b, uint32_t addr, int size, uint32_t *val);
int  bus_write(bao_t *b, uint32_t addr, int size, uint32_t val);
int  bus_dma_read(bao_t *b, uint32_t addr, uint8_t *dst, uint32_t len);
int  bus_dma_write(bao_t *b, uint32_t addr, const uint8_t *src, uint32_t len);

int  mmio_read(bao_t *b, uint32_t addr, int size, uint32_t *val);
int  mmio_write(bao_t *b, uint32_t addr, int size, uint32_t val);
void soc_reset(bao_t *b);
void soc_update(bao_t *b);
uint32_t soc_irq_lines(bao_t *b);
void irq_raise(bao_t *b, const char *event);

int  sce_read(bao_t *b, uint32_t addr, uint32_t *val);
int  sce_write(bao_t *b, uint32_t addr, uint32_t val);
void sce_reset(bao_t *b);
uint64_t sce_next_event(bao_t *b);
void sce_done(bao_t *b, int engine, uint32_t fr_bits);
void pke_start(bao_t *b);
void alu_start(bao_t *b);
void sdma_ich_start(bao_t *b);
uint32_t *bao_mmio_slot(bao_t *b, uint32_t addr);
int  bao_first_time(bao_t *b, uint32_t addr);

int  guard_check(bao_t *b, uint32_t addr, int size, uint32_t val, int is_write);

int  udma_read(bao_t *b, uint32_t addr, uint32_t *val);
int  udma_write(bao_t *b, uint32_t addr, uint32_t val);
void udma_update(bao_t *b);
uint64_t udma_next_event(bao_t *b);
void udma_reset(bao_t *b);

/* IOX INTCR as soc_ifsub.sv instantiates iox (IOC 96, so IOCW 7):
 * {wkupen 10, inten 9, intmode 8:7, intsel 6:0} (iox.sv 108, 137) */
#define IOX_INT_SEL(cr)  ((cr) & 0x7Fu)
#define IOX_INT_MODE(cr) (((cr) >> 7) & 3u)
#define IOX_INT_EN       0x200u

void gpio_notify(bao_t *b);
int  dev_spi_xfer(bao_t *b, int inst, int cs, int tx, int *rx);
int  dev_spi_cs(bao_t *b, int inst, int cs, int active);
int  dev_i2c(bao_t *b, int inst, int ev, uint8_t *data, int *ret);
void devices_save(bao_t *b);
void dev_gpio_changed(bao_t *b);
int  bio_read(bao_t *b, uint32_t addr, uint32_t *val);
int  bio_write(bao_t *b, uint32_t addr, uint32_t val);
void bio_catch_up(bao_t *b);
int  bio_active(bao_t *b);
int  bio_isolated(bao_t *b);
void bio_reset(bao_t *b);
void bio_free(bao_t *b);
void bio_clock_change(bao_t *b, int mul);
void bio_host_pins(bao_t *b);
void bio_dmareq(bao_t *b, int e);
uint32_t bio_irq_lines(bao_t *b);
void irq_level(bao_t *b, const char *event, int was, int now);
int  mdma_read(bao_t *b, uint32_t addr, uint32_t *val);
int  mdma_write(bao_t *b, uint32_t addr, uint32_t val);
void mdma_event(bao_t *b, int line);
void mdma_reset(bao_t *b);
int  mbox_read(bao_t *b, uint32_t addr, uint32_t *val);
int  mbox_write(bao_t *b, uint32_t addr, uint32_t val);
uint32_t mbox_irq_lines(bao_t *b);
void mbox_reset(bao_t *b);
void pke_free(bao_t *b);
int  usb_read(bao_t *b, uint32_t addr, uint32_t *val);
int  usb_write(bao_t *b, uint32_t addr, uint32_t val);
void usb_update(bao_t *b);
uint64_t usb_next_event(bao_t *b);
int  usb_irq_line(bao_t *b);
void usb_reset(bao_t *b);
void usb_free(bao_t *b);
void pwm_update(bao_t *b);
uint64_t pwm_next_event(bao_t *b);
int  pwm_write(bao_t *b, uint32_t off, uint32_t v);
int  pwm_read(bao_t *b, uint32_t off, uint32_t *v);
int  pwm_pin(bao_t *b, int port, int pin, int *level);
void pwm_reset(bao_t *b);
uint32_t gpio_eff_out(bao_t *b, int port);
uint32_t gpio_eff_oe(bao_t *b, int port);
void devices_free(bao_t *b);
void ff_backedge(bao_t *b, uint32_t pc);
int  udma_host_input_pending(bao_t *b);
void cpu_take_irq(bao_t *b, int code, int target);
extern const uint8_t bao_aes_sbox[256];
extern const uint8_t bao_aes_inv_sbox[256];
uint8_t bao_gmul(uint8_t a, uint8_t c);

#endif
