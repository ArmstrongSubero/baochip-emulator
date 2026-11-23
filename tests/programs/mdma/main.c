/*
 * main.c: PL230 MDMA test for baoemu
 *
 * Copyright (c) 2026 Armstrong Subero
 * SPDX-License-Identifier: Apache-2.0
 */

#include "bao.h"
#include "hardware/sha.h"
#include "hardware/irq.h"

#define PL230 0x40011000u
#define MREQ  0x40012000u

#define STATUS     (PL230 + 0x00)
#define CFG        (PL230 + 0x04)
#define CTRLBASE   (PL230 + 0x08)
#define ALTBASE    (PL230 + 0x0C)
#define WAITONREQ  (PL230 + 0x10)
#define SWREQ      (PL230 + 0x14)
#define BURSTSET   (PL230 + 0x18)
#define BURSTCLR   (PL230 + 0x1C)
#define MASKSET    (PL230 + 0x20)
#define MASKCLR    (PL230 + 0x24)
#define ENSET      (PL230 + 0x28)
#define ENCLR      (PL230 + 0x2C)
#define ALTSET     (PL230 + 0x30)
#define ALTCLR     (PL230 + 0x34)
#define PRISET     (PL230 + 0x38)
#define PRICLR     (PL230 + 0x3C)

#define EVSEL(c)   (MREQ + 0x00 + 4u * (c))
#define CR(c)      (MREQ + 0x20 + 4u * (c))
#define SR(c)      (MREQ + 0x40 + 4u * (c))

#define IRQ0_PENDING 0xE0004010u
#define IRQ2_PENDING 0xE0010010u
#define HASH_DONE_LINE 51u    /* IRQ array 3 bit 3 */

#define BYTE 0u
#define HALF 1u
#define WORD 2u
#define NOINC 3u

#define STOP 0u
#define BASIC 1u
#define AUTO 2u
#define PINGPONG 3u
#define MSG_PRI 4u
#define MSG_ALT 5u
#define PSG_PRI 6u
#define PSG_ALT 7u

static volatile uint32_t ctrl[64] __attribute__((aligned(256)));
static uint32_t srcw[64], dstw[64], tasks[12];
static uint8_t srcb[64], dstb[64];
static uint16_t dsth[24];
static volatile uint32_t irq_count;
static int pass, total;

/* the data cache flush the SDK's BIO DMA driver uses around a transfer,
 * so descriptors reach memory and results are read from it */
static void flush(void)
{
    __asm__ volatile ("fence\n.word 0x500F\nnop\nnop\nnop\nnop\n" ::: "memory");
}

static void swreq(uint32_t mask)
{
    flush();
    REG32(SWREQ) = mask;
    flush();
}

static void check(const char *what, uint32_t got, uint32_t want)
{
    total++;
    if (got == want)
        pass++;
    mini_printf("%s: 0x%08x %s\r\n", what, got, got == want ? "PASS" : "FAIL");
}

static uint32_t cfgw(uint32_t inc, uint32_t size, uint32_t r, uint32_t n, uint32_t cc)
{
    return (inc << 30) | (size << 28) | (inc << 26) | (size << 24) | (r << 14) |
           ((n - 1u) << 4) | cc;
}

static void desc(int alt, int ch, const volatile void *src_end, const volatile void *dst_end, uint32_t cfg)
{
    volatile uint32_t *d = &ctrl[(alt ? 32 : 0) + 4 * ch];
    d[0] = (uint32_t)src_end;
    d[1] = (uint32_t)dst_end;
    d[2] = cfg;
    d[3] = 0;
}

static uint32_t cfg_of(int alt, int ch)
{
    return ctrl[(alt ? 32 : 0) + 4 * ch + 2];
}

static void fill(void)
{
    uint32_t i;
    for (i = 0; i < 64; i++) {
        srcw[i] = 0x11110000u + i * 0x01010101u;
        dstw[i] = 0xDEADBEEFu;
        srcb[i] = (uint8_t)(0x40 + i);
        dstb[i] = 0xEE;
    }
    for (i = 0; i < 24; i++)
        dsth[i] = 0x1234;
}

static int words_equal(const uint32_t *a, const uint32_t *b, uint32_t n)
{
    uint32_t i;
    for (i = 0; i < n; i++)
        if (a[i] != b[i])
            return 0;
    return 1;
}

static uint32_t take_pending(void)
{
    uint32_t v = ((REG32(IRQ2_PENDING) & 2u) << 8) | ((REG32(IRQ0_PENDING) & 1u) << 8);
    REG32(IRQ2_PENDING) = 2u;
    REG32(IRQ0_PENDING) = 1u;
    return v;
}

static void one_hash(void)
{
    uint8_t digest[32];
    flush();
    sha256_hash((const uint8_t *)"abc", 3, digest);
    flush();
}

static void registers(void)
{
    uint32_t base = (uint32_t)ctrl;

    check("STATUS master off, 8 channels", REG32(STATUS), 0x10070000u);
    REG32(CFG) = 1;
    check("STATUS master on", REG32(STATUS), 0x10070001u);
    check("CFG write only", REG32(CFG), 0);
    REG32(CTRLBASE) = base | 0xFFu;
    check("CTRLBASEPTR bits 31:8", REG32(CTRLBASE), base);
    check("ALTCTRLBASEPTR base plus 0x80", REG32(ALTBASE), base + 0x80u);
    REG32(ENSET) = 0x1FFu;
    check("CHNLENABLESET 8 bits", REG32(ENSET), 0xFF);
    REG32(ENCLR) = 0xFFu;
    check("CHNLENABLECLR clears", REG32(ENSET), 0);
    REG32(EVSEL(0)) = 0xFFFFFFFFu;
    check("EVSEL0 8 bits", REG32(EVSEL(0)), 0xFF);
    REG32(EVSEL(0)) = 0;
    REG32(CR(0)) = 0xFFFFFFFFu;
    check("CR0 5 bits", REG32(CR(0)), 0x1F);
    REG32(CR(0)) = 0;
    REG32(SR(0)) = 0xFFFFFFFFu;
    check("SR0 read only", REG32(SR(0)), 0);
    REG32(MREQ + 0x60) = 0xFFFFFFFFu;
    check("MDMA undecoded 0x60", REG32(MREQ + 0x60), 0);
    REG32(CR(3)) = 0x10;
    check("WAITONREQ status from CR3 bit 4", REG32(WAITONREQ), 0x08);
    REG32(CR(3)) = 0;
    check("PERIPH_ID_0", REG32(PL230 + 0xFE0), 0x30);
    check("PERIPH_ID_1", REG32(PL230 + 0xFE4), 0xB2);
}

static void auto_copies(void)
{
    uint32_t cfg;
    uint32_t i, ok;

    fill();
    take_pending();
    cfg = cfgw(WORD, WORD, 3, 64, AUTO);
    desc(0, 0, &srcw[63], &dstw[63], cfg);
    REG32(ENSET) = 1;
    swreq(1);
    check("auto word copy of 64", words_equal(srcw, dstw, 64), 1);
    check("auto cfg written back stopped", cfg_of(0, 0), cfg & ~0x3FF7u);
    check("auto channel disabled", REG32(ENSET) & 1, 0);
    check("done in arrays 0 and 2", take_pending(), 0x300);

    desc(0, 0, &srcb[36], &dstb[3 + 36], cfgw(BYTE, BYTE, 10, 37, AUTO));
    REG32(ENSET) = 1;
    swreq(1);
    ok = dstb[2] == 0xEE && dstb[40] == 0xEE;
    for (i = 0; i < 37; i++)
        if (dstb[3 + i] != srcb[i])
            ok = 0;
    check("auto byte copy of 37 to an odd address", ok, 1);
    take_pending();

    /* source fixed, destination half words */
    srcw[0] = 0xA5C3A5C3u;
    cfg = (HALF << 30) | (HALF << 28) | (NOINC << 26) | (HALF << 24) | (2u << 14) | (19u << 4) | AUTO;
    desc(0, 0, &srcw[0], &dsth[19], cfg);
    REG32(ENSET) = 1;
    swreq(1);
    ok = dsth[20] == 0x1234;
    for (i = 0; i < 20; i++)
        if (dsth[i] != 0xA5C3)
            ok = 0;
    check("fill of 20 half words from a fixed source", ok, 1);
    take_pending();
}

static void basic_mode(void)
{
    int k;

    fill();
    desc(0, 1, &srcw[15], &dstw[15], cfgw(WORD, WORD, 2, 16, BASIC));
    REG32(ENSET) = 2;
    swreq(2);
    check("basic one request moves 4", words_equal(srcw, dstw, 4) && dstw[4] == 0xDEADBEEFu, 1);
    check("basic n_minus_1 after one batch", (cfg_of(0, 1) >> 4) & 0x3FF, 11);
    check("basic still enabled, no done", (REG32(ENSET) & 2) | take_pending(), 2);
    for (k = 0; k < 3; k++)
        swreq(2);
    check("basic four requests move 16", words_equal(srcw, dstw, 16) && dstw[16] == 0xDEADBEEFu, 1);
    check("basic done and disabled", (REG32(ENSET) & 2) | take_pending(), 0x300);
}

static void ping_pong(void)
{
    fill();
    desc(0, 2, &srcw[7], &dstw[7], cfgw(WORD, WORD, 3, 8, PINGPONG));
    desc(1, 2, &srcw[15], &dstw[15], cfgw(WORD, WORD, 3, 8, PINGPONG));
    REG32(ALTCLR) = 4;
    REG32(ENSET) = 4;
    swreq(4);
    check("ping pong primary half", words_equal(srcw, dstw, 8) && dstw[8] == 0xDEADBEEFu, 1);
    check("ping pong switched to alternate, still enabled", ((REG32(ALTSET) & 4) << 1) | (REG32(ENSET) & 4), 12);
    check("ping pong done after primary", take_pending(), 0x300);
    swreq(4);
    check("ping pong alternate half", words_equal(srcw, dstw, 16), 1);
    check("ping pong back to primary", REG32(ALTSET) & 4, 0);
    check("ping pong done after alternate", take_pending(), 0x300);
    swreq(4);
    check("ping pong stop disables, no done", (REG32(ENSET) & 4) | take_pending(), 0);
}

static void scatter_gather(void)
{
    uint32_t i, ok;
    volatile uint32_t *alt3 = &ctrl[32 + 12];

    fill();
    tasks[0] = (uint32_t)&srcw[4];
    tasks[1] = (uint32_t)&dstw[24];
    tasks[2] = cfgw(WORD, WORD, 2, 5, MSG_ALT);
    tasks[3] = 0;
    tasks[4] = (uint32_t)&srcb[6];
    tasks[5] = (uint32_t)&dstb[16];
    tasks[6] = cfgw(BYTE, BYTE, 3, 7, MSG_ALT);
    tasks[7] = 0;
    tasks[8] = (uint32_t)&srcw[42];
    tasks[9] = (uint32_t)&dstw[52];
    tasks[10] = cfgw(WORD, WORD, 0, 3, AUTO);
    tasks[11] = 0;
    desc(0, 3, &tasks[11], &alt3[3], cfgw(WORD, WORD, 2, 12, MSG_PRI));
    REG32(ALTCLR) = 8;
    REG32(ENSET) = 8;
    swreq(8);
    ok = words_equal(&srcw[0], &dstw[20], 5) && words_equal(&srcw[40], &dstw[50], 3) &&
         dstw[19] == 0xDEADBEEFu && dstw[25] == 0xDEADBEEFu && dstw[53] == 0xDEADBEEFu;
    for (i = 0; i < 7; i++)
        if (dstb[10 + i] != srcb[i])
            ok = 0;
    if (dstb[9] != 0xEE || dstb[17] != 0xEE)
        ok = 0;
    check("memory scatter gather, three tasks", ok, 1);
    check("scatter gather ends on the alternate", REG32(ALTSET) & 8, 8);
    check("scatter gather one done, disabled", (REG32(ENSET) & 8) | take_pending(), 0x300);
    check("primary stopped after the last copy", cfg_of(0, 3) & 0x3FF7u, 0);
    REG32(ALTCLR) = 8;
}

/* each request runs one batch of the alternate task; the next task is
 * copied as soon as one ends */
static void peripheral_scatter_gather(void)
{
    volatile uint32_t *alt3 = &ctrl[32 + 12];

    fill();
    tasks[0] = (uint32_t)&srcw[1];
    tasks[1] = (uint32_t)&dstw[1];
    tasks[2] = cfgw(WORD, WORD, 1, 2, PSG_ALT);
    tasks[3] = 0;
    tasks[4] = (uint32_t)&srcw[11];
    tasks[5] = (uint32_t)&dstw[11];
    tasks[6] = cfgw(WORD, WORD, 1, 2, BASIC);
    tasks[7] = 0;
    desc(0, 3, &tasks[7], &alt3[3], cfgw(WORD, WORD, 2, 8, PSG_PRI));
    REG32(ALTCLR) = 8;
    REG32(ENSET) = 8;
    swreq(8);
    check("peripheral scatter gather first request, first task",
          words_equal(srcw, dstw, 2) && dstw[10] == 0xDEADBEEFu, 1);
    check("second task copied, waiting on the alternate",
          ((REG32(ALTSET) & 8) << 4) | (REG32(ENSET) & 8) | take_pending() | (alt3[2] & 7), 0x80 | 8 | BASIC);
    swreq(8);
    check("second request runs the last task, done",
          (words_equal(&srcw[10], &dstw[10], 2) ? 0x1000 : 0) | (REG32(ENSET) & 8) | take_pending(), 0x1300);
    REG32(ALTCLR) = 8;
}

static void stop_and_master(void)
{
    fill();
    desc(0, 6, &srcw[3], &dstw[3], cfgw(WORD, WORD, 2, 4, STOP));
    REG32(ENSET) = 0x40;
    swreq(0x40);
    check("stop structure disables, moves nothing", (REG32(ENSET) & 0x40) | (dstw[0] != 0xDEADBEEFu) | take_pending(), 0);

    desc(0, 0, &srcw[3], &dstw[3], cfgw(WORD, WORD, 2, 4, AUTO));
    REG32(CFG) = 0;
    REG32(ENSET) = 1;
    swreq(1);
    check("master off ignores requests", dstw[0] == 0xDEADBEEFu && (REG32(ENSET) & 1), 1);
    REG32(CFG) = 1;
    swreq(1);
    check("master on serves them", words_equal(srcw, dstw, 4), 1);
    take_pending();
}

static void priority(void)
{
    uint32_t a[4], c[4], d[4];
    int i;

    for (i = 0; i < 4; i++) {
        a[i] = 0xA0 + (uint32_t)i;
        c[i] = 0xC0 + (uint32_t)i;
    }
    desc(0, 0, &a[3], &d[3], cfgw(WORD, WORD, 0, 4, AUTO));
    desc(0, 1, &c[3], &d[3], cfgw(WORD, WORD, 0, 4, AUTO));
    REG32(ENSET) = 3;
    swreq(3);
    check("same priority, channel 0 first, 1 last", words_equal(d, c, 4), 1);
    desc(0, 0, &a[3], &d[3], cfgw(WORD, WORD, 0, 4, AUTO));
    desc(0, 1, &c[3], &d[3], cfgw(WORD, WORD, 0, 4, AUTO));
    REG32(PRISET) = 2;
    check("CHNLPRIORITYSET", REG32(PRISET), 2);
    REG32(ENSET) = 3;
    swreq(3);
    check("channel 1 high priority first, 0 last", words_equal(d, a, 4), 1);
    REG32(PRICLR) = 2;
    take_pending();
}

static void hardware_requests(void)
{
    fill();
    sha256_init();
    take_pending();
    desc(0, 4, &srcw[11], &dstw[11], cfgw(WORD, WORD, 2, 12, BASIC));
    REG32(EVSEL(4)) = HASH_DONE_LINE;
    REG32(CR(4)) = 0x05;    /* en, dma_req, level */
    REG32(ENSET) = 0x10;
    one_hash();
    check("hash done requests one batch", words_equal(srcw, dstw, 4) && dstw[4] == 0xDEADBEEFu, 1);
    REG32(MASKSET) = 0x10;
    one_hash();
    check("masked request moves nothing", dstw[4], 0xDEADBEEFu);
    REG32(CR(4)) = 0x07;    /* pulse mode counts the requests */
    one_hash();
    one_hash();
    one_hash();
    check("pulse mode SR: 3 queued, overflow", REG32(SR(4)), 0x13);
    REG32(CR(4)) = 0x05;
    check("level mode clears SR", REG32(SR(4)), 0);
    REG32(MASKCLR) = 0x10;
    one_hash();
    check("unmasked again, second batch", words_equal(srcw, dstw, 8) && dstw[8] == 0xDEADBEEFu, 1);
    REG32(CR(4)) = 0x07;
    one_hash();
    check("pulse request taken leaves SR 0", REG32(SR(4)), 0);
    check("third batch completes the cycle", words_equal(srcw, dstw, 12) && !(REG32(ENSET) & 0x10), 1);
    check("hardware cycle done", take_pending(), 0x300);

    /* dma_sreq: one item per request while useburst is clear */
    desc(0, 5, &srcw[31], &dstw[31], cfgw(WORD, WORD, 2, 8, BASIC));
    REG32(EVSEL(5)) = HASH_DONE_LINE;
    REG32(CR(5)) = 0x09;    /* en, dma_sreq, level */
    REG32(ENSET) = 0x20;
    REG32(CR(4)) = 0;
    one_hash();
    check("single request moves one", dstw[24] == srcw[24] && dstw[25] == 0xDEADBEEFu, 1);
    REG32(BURSTSET) = 0x20;
    one_hash();
    check("useburst set ignores single requests", dstw[25], 0xDEADBEEFu);
    REG32(BURSTCLR) = 0x20;
    REG32(CR(5)) = 0;
    REG32(ENCLR) = 0x20;
}

static void on_array2(uint32_t pending)
{
    if (pending & 2u)
        irq_count++;
    REG32(IRQ2_PENDING) = pending;
}

static void interrupt(void)
{
    fill();
    irq_init();
    irq_set_handler(IRQ_ARRAY2, on_array2);
    irq_enable_events(IRQ_ARRAY2, 2u);
    desc(0, 7, &srcw[9], &dstw[9], cfgw(WORD, WORD, 1, 10, AUTO));
    REG32(ENSET) = 0x80;
    swreq(0x80);
    __asm__ volatile ("nop; nop; nop; nop");
    flush();
    check("interrupt handler saw one done", irq_count, 1);
    check("copy under interrupts", words_equal(srcw, dstw, 10), 1);
    irq_disable(IRQ_ARRAY2);
}

int main(void)
{
    bao_init();
    mini_printf("\r\nmdma test\r\n");
    registers();
    auto_copies();
    basic_mode();
    ping_pong();
    scatter_gather();
    peripheral_scatter_gather();
    stop_and_master();
    priority();
    hardware_requests();
    interrupt();
    mini_printf("[Done] mdma %d of %d passed\r\n", pass, total);
    while (1)
        __asm__ volatile ("wfi");
}
