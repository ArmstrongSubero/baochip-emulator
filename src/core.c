/*
 * core.c: emulator instance, run loop, loaders and symbols
 *
 * Copyright (c) 2026 Armstrong Subero
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include "bao_int.h"

#define BAO_VERSION "0.2.0"
#define PROGRAM_ENTRY 0x60060300u

int udma_host_input_pending(bao_t *b);

const char *bao_version(void)
{
    return BAO_VERSION;
}

void bao_logf(bao_t *b, int level, const char *fmt, ...)
{
    char msg[512];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(msg, sizeof(msg), fmt, ap);
    va_end(ap);
    if (b->log_cb)
        b->log_cb(b->log_user, level, msg);
    else
        fprintf(stderr, "[%s] %s\n", level == BAO_LOG_ERROR ? "error" : level == BAO_LOG_WARN ? "warn" : "info", msg);
}

void bao_halt(bao_t *b, int reason, const char *fmt, ...)
{
    va_list ap;
    if (b->stop)
        return;
    va_start(ap, fmt);
    vsnprintf(b->stop_msg, sizeof(b->stop_msg), fmt, ap);
    va_end(ap);
    b->stop = reason;
    b->limit = 0;
}

static void map_region(bao_t *b, uint32_t base, uint32_t size, uint8_t *mem, int writable)
{
    uint32_t off;
    for (off = 0; off < size; off += 0x10000) {
        b->rdmap[(base + off) >> 16] = mem + off;
        b->wrmap[(base + off) >> 16] = writable ? mem + off : NULL;
    }
}

bao_t *bao_new(void)
{
    bao_t *b = calloc(1, sizeof(*b));
    if (!b)
        return NULL;
    b->rram = malloc(RRAM_SIZE);
    b->sram = malloc(SRAM_SIZE);
    b->ifram = calloc(1, IFRAM_SIZE);
    if (!b->rram || !b->sram || !b->ifram) {
        bao_free(b);
        return NULL;
    }
    memset(b->rram, 0xFF, RRAM_SIZE);
    /* SERIAL_NUMBER and UUID at 0x603E0000: placeholder bytes, not a real layout */
    {
        static const uint8_t serial[8] = { 'B', 'A', 'O', 'E', 'M', 'U', 0x00, 0x01 };
        static const uint8_t uuid[16] = { 0x62, 0x61, 0x6F, 0x65, 0x6D, 0x75, 0x40, 0x00,
                                          0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01 };
        memset(b->rram + 0x3E0000, 0x00, 64);
        memcpy(b->rram + 0x3E0000, serial, sizeof(serial));
        memcpy(b->rram + 0x3E0020, uuid, sizeof(uuid));
    }
    map_region(b, RRAM_BASE, RRAM_SIZE, b->rram, 0);
    map_region(b, SRAM_BASE, SRAM_SIZE, b->sram, 1);
    map_region(b, IFRAM_BASE, IFRAM_SIZE, b->ifram, 1);
    b->guard_on = 1;
    b->stop_on_exception = 1;
    b->ff_on = 1;
    b->entry = PROGRAM_ENTRY;
    /* ADC1 to ADC3 are pads PA5 to PA7, which the Dabao ties to GND, the
     * +0.9V rail and 3.3 V (dabao_v3c pads C4, F6 and D7) */
    b->adc_mv[1] = 0;
    b->adc_mv[2] = 900;
    b->adc_mv[3] = 3300;
    bao_reset(b);
    return b;
}

void bao_free(bao_t *b)
{
    int i;
    if (!b)
        return;
    devices_free(b);
    bio_free(b);
    pke_free(b);
    usb_free(b);
    for (i = 0; i < 65536; i++)
        free(b->mmio_pages[i]);
    for (i = 0; i < b->nsyms; i++)
        free(b->syms[i].name);
    free(b->syms);
    free(b->rram);
    free(b->sram);
    free(b->ifram);
    free(b);
}

void bao_reset(bao_t *b)
{
    b->cycles = 0;
    b->instret = 0;
    memset(b->sram, 0xCD, SRAM_SIZE);
    memset(b->warned, 0, sizeof(b->warned));
    b->vex_gated = 0;
    /* power on and the RESET pin reset the always on domain */
    b->rtc_lr = b->rtc_cr = b->rtc_frozen = b->rtc_fd = 0;
    b->rtc_mr = b->rtc_imsc = b->rtc_ris = b->rtc_seen = 0;
    b->rtc_cnt0 = b->rtc_lvl0 = b->rtc_pend = 0;
    b->at_cfg[2] = b->at_cfg[3] = b->at_cmp[2] = b->at_cmp[3] = 0;
    b->at_base_val[2] = b->at_base_val[3] = 0;
    b->at_base[2] = b->at_base[3] = 0;
    ao_por(b);
    cpu_reset(b);
    soc_reset(b);
    b->stop = BAO_RUNNING;
    b->stop_msg[0] = 0;
    b->rrc_commits = 0;
}

static int load_bytes(bao_t *b, uint32_t addr, const uint8_t *src, uint32_t len)
{
    uint32_t i;
    for (i = 0; i < len; i++) {
        uint32_t a = addr + i;
        uint8_t *p = b->rdmap[a >> 16];
        if (!p) {
            bao_logf(b, BAO_LOG_ERROR, "load address 0x%08X is outside RRAM, SRAM and IFRAM", a);
            return -1;
        }
        p[a & 0xFFFF] = src[i];
    }
    return 0;
}

static uint8_t *read_file(const char *path, long *len)
{
    FILE *f = fopen(path, "rb");
    uint8_t *d;
    if (!f)
        return NULL;
    fseek(f, 0, SEEK_END);
    *len = ftell(f);
    fseek(f, 0, SEEK_SET);
    d = malloc((size_t)*len + 1);
    if (d && fread(d, 1, (size_t)*len, f) != (size_t)*len) {
        free(d);
        d = NULL;
    }
    fclose(f);
    return d;
}

static uint32_t rd32(const uint8_t *p) { return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24); }
static uint16_t rd16(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8)); }

static int sym_cmp(const void *x, const void *y)
{
    const sym_t *a = x, *c = y;
    return a->addr < c->addr ? -1 : a->addr > c->addr;
}

static void clear_syms(bao_t *b)
{
    int i;
    for (i = 0; i < b->nsyms; i++)
        free(b->syms[i].name);
    free(b->syms);
    b->syms = NULL;
    b->nsyms = 0;
}

int bao_load_elf(bao_t *b, const char *path)
{
    long len;
    uint8_t *d = read_file(path, &len);
    uint32_t phoff, shoff, i;
    uint16_t phnum, phentsize, shnum, shentsize;

    if (!d) {
        bao_logf(b, BAO_LOG_ERROR, "cannot read %s", path);
        return -1;
    }
    if (len < 52 || memcmp(d, "\x7f" "ELF", 4) || d[4] != 1 || d[5] != 1 || rd16(d + 18) != 243) {
        bao_logf(b, BAO_LOG_ERROR, "%s is not a 32 bit little endian RISC-V ELF", path);
        free(d);
        return -1;
    }
    b->entry = rd32(d + 24);
    phoff = rd32(d + 28);
    shoff = rd32(d + 32);
    phentsize = rd16(d + 42);
    phnum = rd16(d + 44);
    shentsize = rd16(d + 46);
    shnum = rd16(d + 48);

    for (i = 0; i < phnum; i++) {
        const uint8_t *ph = d + phoff + i * phentsize;
        uint32_t off = rd32(ph + 4), paddr = rd32(ph + 12), filesz = rd32(ph + 16);
        if (rd32(ph) != 1 || filesz == 0)
            continue;
        if ((long)(off + filesz) > len || load_bytes(b, paddr, d + off, filesz)) {
            free(d);
            return -1;
        }
    }

    clear_syms(b);
    for (i = 0; i < shnum; i++) {
        const uint8_t *sh = d + shoff + i * shentsize;
        if (rd32(sh + 4) == 2) {
            uint32_t symoff = rd32(sh + 16), symsize = rd32(sh + 20), link = rd32(sh + 24);
            const uint8_t *strsh = d + shoff + link * shentsize;
            const char *strtab = (const char *)d + rd32(strsh + 16);
            uint32_t n = symsize / 16, k;
            b->syms = calloc(n, sizeof(sym_t));
            for (k = 0; k < n && b->syms; k++) {
                const uint8_t *s = d + symoff + k * 16;
                uint8_t type = s[12] & 0xF;
                const char *name = strtab + rd32(s);
                if ((type == 1 || type == 2) && name[0]) {
                    b->syms[b->nsyms].addr = rd32(s + 4);
                    b->syms[b->nsyms].name = malloc(strlen(name) + 1);
                    strcpy(b->syms[b->nsyms].name, name);
                    b->nsyms++;
                }
            }
            if (b->syms)
                qsort(b->syms, (size_t)b->nsyms, sizeof(sym_t), sym_cmp);
            break;
        }
    }
    free(d);
    b->pc = b->entry;
    return 0;
}

int bao_load_uf2(bao_t *b, const char *path)
{
    long len, off;
    uint8_t *d = read_file(path, &len);
    int blocks = 0;
    if (!d) {
        bao_logf(b, BAO_LOG_ERROR, "cannot read %s", path);
        return -1;
    }
    for (off = 0; off + 512 <= len; off += 512) {
        const uint8_t *blk = d + off;
        uint32_t target, size;
        if (rd32(blk) != 0x0A324655u || rd32(blk + 4) != 0x9E5D5157u || rd32(blk + 508) != 0x0AB16F30u)
            continue;
        target = rd32(blk + 12);
        size = rd32(blk + 16);
        if (size > 476 || load_bytes(b, target, blk + 32, size)) {
            free(d);
            return -1;
        }
        blocks++;
    }
    free(d);
    if (!blocks) {
        bao_logf(b, BAO_LOG_ERROR, "%s holds no UF2 blocks", path);
        return -1;
    }
    clear_syms(b);
    b->entry = PROGRAM_ENTRY;
    b->pc = b->entry;
    return 0;
}

int bao_load_bin(bao_t *b, const char *path, uint32_t addr)
{
    long len;
    uint8_t *d = read_file(path, &len);
    int r;
    if (!d) {
        bao_logf(b, BAO_LOG_ERROR, "cannot read %s", path);
        return -1;
    }
    r = load_bytes(b, addr, d, (uint32_t)len);
    free(d);
    if (r == 0) {
        clear_syms(b);
        b->entry = addr == 0x60060000u ? PROGRAM_ENTRY : addr;
        b->pc = b->entry;
    }
    return r;
}

int bao_load_file(bao_t *b, const char *path)
{
    size_t n = strlen(path);
    if (n > 4 && (strcmp(path + n - 4, ".uf2") == 0 || strcmp(path + n - 4, ".UF2") == 0))
        return bao_load_uf2(b, path);
    if (n > 4 && (strcmp(path + n - 4, ".bin") == 0 || strcmp(path + n - 4, ".BIN") == 0))
        return bao_load_bin(b, path, PROGRAM_ENTRY);
    return bao_load_elf(b, path);
}

int bao_rram_image_load(bao_t *b, const char *path)
{
    FILE *f = fopen(path, "rb");
    size_t n;
    if (!f)
        return -1;
    n = fread(b->rram, 1, RRAM_SIZE, f);
    fclose(f);
    return n == RRAM_SIZE ? 0 : -1;
}

int bao_rram_image_save(bao_t *b, const char *path)
{
    FILE *f = fopen(path, "wb");
    size_t n;
    if (!f)
        return -1;
    n = fwrite(b->rram, 1, RRAM_SIZE, f);
    fclose(f);
    return n == RRAM_SIZE ? 0 : -1;
}

int bao_run(bao_t *b, uint64_t cycles)
{
    uint64_t target = b->cycles + cycles;

    if (b->stop == BAO_STOP_LIMIT || b->stop == BAO_STOP_USER || b->stop == BAO_STOP_IDLE) {
        b->stop = BAO_RUNNING;
        b->stop_msg[0] = 0;
    }
    if (b->stop)
        return b->stop;

    while (!b->stop) {
        if (b->cycles >= target) {
            b->stop = BAO_STOP_LIMIT;
            break;
        }
        if (b->cycles >= b->next_event) {
            soc_update(b);
            /* a device may have changed state, so the idle snapshot is stale */
            b->ff_pc = 0;
        }
        if (b->pd_state >= 2) {
            if (b->next_event == NEVER) {
                bao_halt(b, BAO_STOP_IDLE, "powered down with no wake source left; "
                         "only RESET or a power cycle brings it back");
                break;
            }
            b->cycles = b->next_event < target ? b->next_event : target;
            continue;
        }
        if (b->wfi && cpu_wake(b))
            cpu_wfi_leave(b);
        {
            int tgt, code = cpu_irq_pending(b, &tgt);
            if (code >= 0) {
                cpu_take_irq(b, code, tgt);
                if (b->stop)
                    break;
            }
        }
        if (b->wfi) {
            if (bio_active(b)) {
                uint64_t t = bio_isolated(b) ? NEVER : b->cycles + 128;
                if (b->next_event < t) t = b->next_event;
                if (target < t) t = target;
                b->cycles = t;
                cpu_idle(b);
                bio_catch_up(b);
                continue;
            }
            if (b->next_event == NEVER) {
                bao_halt(b, BAO_STOP_IDLE, "wfi at %s with nothing left that can wake the core",
                         bao_where(b, b->pc));
                break;
            }
            b->cycles = b->next_event < target ? b->next_event : target;
            cpu_idle(b);
            continue;
        }
        b->limit = b->next_event < target ? b->next_event : target;
        if (bio_active(b) && !bio_isolated(b) && b->limit > b->cycles + 128)
            b->limit = b->cycles + 128;
        cpu_step_block(b);
        if (bio_active(b))
            bio_catch_up(b);
    }
    return b->stop;
}

void bao_stop(bao_t *b) { bao_halt(b, BAO_STOP_USER, "stopped by host"); }
int bao_stop_reason(bao_t *b) { return b->stop; }
const char *bao_stop_message(bao_t *b) { return b->stop_msg; }
uint64_t bao_cycles(bao_t *b) { return b->cycles; }
uint64_t bao_millis(bao_t *b) { tt_sync(b); return b->tt_base_ms + (b->cycles - b->tt_base_cycle) / b->tt_cpt; }
uint32_t bao_get_pc(bao_t *b) { return b->pc; }
void bao_set_pc(bao_t *b, uint32_t pc) { b->pc = pc; }
uint32_t bao_get_reg(bao_t *b, int r) { return (r > 0 && r < 32) ? b->x[r] : 0; }
void bao_set_reg(bao_t *b, int r, uint32_t v) { if (r > 0 && r < 32) b->x[r] = v; }
void bao_set_guard(bao_t *b, int on) { b->guard_on = on; }
void bao_set_stop_on_exception(bao_t *b, int on) { b->stop_on_exception = on; }
void bao_set_uart_instant(bao_t *b, int on) { b->uart_instant = on; }
void bao_set_fast_forward(bao_t *b, int on) { b->ff_on = on; b->ff_pc = 0; }
uint64_t bao_skipped_cycles(bao_t *b) { return b->ff_skipped; }
uint32_t bao_rram_commits(bao_t *b) { return b->rrc_commits; }

int bao_mem_read(bao_t *b, uint32_t addr, uint8_t *buf, uint32_t len)
{
    uint32_t i;
    for (i = 0; i < len; i++) {
        uint8_t *p = b->rdmap[(addr + i) >> 16];
        if (!p) return -1;
        buf[i] = p[(addr + i) & 0xFFFF];
    }
    return 0;
}

int bao_peek32(bao_t *b, uint32_t addr, uint32_t *val)
{
    return mmio_read(b, addr & ~3u, 4, val);
}

int bao_mem_write(bao_t *b, uint32_t addr, const uint8_t *buf, uint32_t len)
{
    return load_bytes(b, addr, buf, len);
}

uint32_t bao_symbol(bao_t *b, const char *name)
{
    int i;
    for (i = 0; i < b->nsyms; i++)
        if (strcmp(b->syms[i].name, name) == 0)
            return b->syms[i].addr;
    return 0;
}

const char *bao_where(bao_t *b, uint32_t pc)
{
    int lo = 0, hi = b->nsyms - 1, best = -1;
    while (lo <= hi) {
        int mid = (lo + hi) / 2;
        if (b->syms[mid].addr <= pc) { best = mid; lo = mid + 1; }
        else hi = mid - 1;
    }
    if (best >= 0)
        snprintf(b->where_buf, sizeof(b->where_buf), "pc=0x%08X (%s+0x%X)", pc,
                 b->syms[best].name, pc - b->syms[best].addr);
    else
        snprintf(b->where_buf, sizeof(b->where_buf), "pc=0x%08X", pc);
    return b->where_buf;
}

void bao_on_log(bao_t *b, bao_log_cb cb, void *user) { b->log_cb = cb; b->log_user = user; }
void bao_on_uart(bao_t *b, bao_uart_cb cb, void *user) { b->uart_cb = cb; b->uart_user = user; }
void bao_on_gpio(bao_t *b, bao_gpio_cb cb, void *user) { b->gpio_cb = cb; b->gpio_user = user; }
void bao_on_spi(bao_t *b, bao_spi_cb cb, bao_spi_cs_cb cs, void *user) { b->spi_cb = cb; b->spi_cs_cb = cs; b->spi_user = user; }
void bao_on_i2c(bao_t *b, bao_i2c_cb cb, void *user) { b->i2c_cb = cb; b->i2c_user = user; }

void bao_gpio_set_input(bao_t *b, int port, int pin, int level)
{
    if (port < 0 || port > 5 || pin < 0 || pin > 15)
        return;
    bio_catch_up(b);
    b->gpio_ext_set[port] |= 1u << pin;
    if (level) b->gpio_ext[port] |= 1u << pin;
    else b->gpio_ext[port] &= ~(1u << pin);
    bio_host_pins(b);
    gpio_notify(b);
    bao_kick(b);
}

uint32_t bao_gpio_get_out(bao_t *b, int port) { return (port >= 0 && port < 6) ? gpio_eff_out(b, port) : 0; }
uint32_t bao_gpio_get_oe(bao_t *b, int port) { return (port >= 0 && port < 6) ? gpio_eff_oe(b, port) : 0; }
uint32_t bao_gpio_get_af(bao_t *b, int port) { return (port >= 0 && port < 6) ? (b->iox[port * 2] & 0xFFFF) | (b->iox[port * 2 + 1] << 16) : 0; }

void bao_adc_set_mv(bao_t *b, int channel, uint32_t mv)
{
    if (channel >= 0 && channel < 4)
        b->adc_mv[channel] = mv;
}
