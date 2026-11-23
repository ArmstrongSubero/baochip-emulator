/*
 * main.c: mailbox loopback test for baoemu
 *
 * Copyright (c) 2026 Armstrong Subero
 * SPDX-License-Identifier: Apache-2.0
 */

#include "bao.h"
#include "hardware/irq.h"

#define MBX  0xE0018000u
#define MBC  0xE0019000u
#define APB  0x40013000u

#define M_WDATA    (MBX + 0x00)
#define M_RDATA    (MBX + 0x04)
#define M_EVSTAT   (MBX + 0x08)
#define M_PENDING  (MBX + 0x0C)
#define M_ENABLE   (MBX + 0x10)
#define M_STATUS   (MBX + 0x14)
#define M_CONTROL  (MBX + 0x18)
#define M_DONE     (MBX + 0x1C)
#define M_LOOPBACK (MBX + 0x20)

#define C_WDATA    (MBC + 0x00)
#define C_RDATA    (MBC + 0x04)
#define C_STATUS   (MBC + 0x08)
#define C_PENDING  (MBC + 0x10)
#define C_ENABLE   (MBC + 0x14)
#define C_CONTROL  (MBC + 0x18)
#define C_DONE     (MBC + 0x1C)

#define A_WDATA    (APB + 0x00)
#define A_RDATA    (APB + 0x04)
#define A_STATUS   (APB + 0x08)
#define A_ABORT    (APB + 0x18)
#define A_DONE     (APB + 0x1C)

#define IRQ2_PENDING  0xE0010010u
#define IRQ19_PENDING 0xE000F010u

/* client STATUS bits */
#define RX_AVAIL 1u
#define TX_FREE  2u
#define IN_PROG  4u
#define ACK      8u

#define FIFO 1025u

static int pass, total;
static volatile uint32_t irq_seen;

static void check(const char *what, uint32_t got, uint32_t want)
{
    total++;
    if (got == want)
        pass++;
    mini_printf("%s: 0x%08x %s\r\n", what, got, got == want ? "PASS" : "FAIL");
}

static uint32_t rx_words(void) { return REG32(M_STATUS) & 0x7FFu; }
static uint32_t tx_words(void) { return (REG32(M_STATUS) >> 11) & 0x7FFu; }

static uint32_t take_irq2(void)
{
    uint32_t v = REG32(IRQ2_PENDING) & 0x3Cu;
    REG32(IRQ2_PENDING) = 0x3Cu;
    return v;
}

static uint32_t mip_vex(void)
{
    uint32_t v;
    __asm__ volatile ("csrr %0, 0xFC0" : "=r"(v));
    return v;
}

static void reset_values(void)
{
    check("MAILBOX STATUS after reset", REG32(M_STATUS), 0);
    check("MAILBOX LOOPBACK after reset", REG32(M_LOOPBACK), 0);
    check("MBOX_APB STATUS after reset", REG32(A_STATUS), TX_FREE);
    check("MB_CLIENT STATUS after reset", REG32(C_STATUS), TX_FREE);
    check("MAILBOX offset 0x24 reads 0", REG32(MBX + 0x24), 0);
    check("MBOX_APB offset 0x0C reads 0", REG32(APB + 0x0C), 0);
    REG32(M_ENABLE) = 0xFFu;
    check("MAILBOX EV_ENABLE is 4 bits", REG32(M_ENABLE), 0xF);
    REG32(M_ENABLE) = 0;
}

static void apb_words(void)
{
    uint32_t i, ok;

    take_irq2();
    REG32(A_WDATA) = 0x11223344u;
    check("APB write lands in MAILBOX", rx_words(), 1);
    check("MBOX_APB WDATA reads back", REG32(A_WDATA), 0x11223344u);
    check("MAILBOX RDATA", REG32(M_RDATA), 0x11223344u);
    check("MAILBOX empty again", rx_words(), 0);

    REG32(M_WDATA) = 0xCAFEF00Du;
    check("MAILBOX tx_words", tx_words(), 1);
    check("MBOX_APB rx_avail", REG32(A_STATUS), RX_AVAIL | TX_FREE);
    check("MBOX_APB RDATA", REG32(A_RDATA), 0xCAFEF00Du);
    check("MBOX_APB STATUS after the read", REG32(A_STATUS), TX_FREE);

    /* an empty read returns the last word and sets bit 4, which mbox.sv
     * wires to the client's rx_err */
    check("MBOX_APB empty RDATA keeps the last word", REG32(A_RDATA), 0xCAFEF00Du);
    check("APB error irq level", take_irq2(), 0x20);
    check("MBOX_APB STATUS rx_err in bit 4", REG32(A_STATUS), TX_FREE | 0x10u);
    check("MBOX_APB STATUS cleared on read", REG32(A_STATUS), TX_FREE);
    check("APB error irq gone once cleared", take_irq2(), 0);

    for (i = 0; i < 8; i++)
        REG32(A_WDATA) = 0xA0000000u + i;
    check("eight APB words queued", rx_words(), 8);
    ok = 1;
    for (i = 0; i < 8; i++)
        if (REG32(M_RDATA) != 0xA0000000u + i)
            ok = 0;
    check("eight APB words in order", ok, 1);

    for (i = 0; i < 3; i++)
        REG32(M_WDATA) = 0xB0000000u + i;
    check("three MAILBOX words queued", tx_words(), 3);
    ok = 1;
    for (i = 0; i < 3; i++)
        if (REG32(A_RDATA) != 0xB0000000u + i)
            ok = 0;
    check("three MAILBOX words in order", ok, 1);

    check("MAILBOX empty RDATA keeps the last word", REG32(M_RDATA), 0xA0000007u);
    check("MAILBOX EV_STATUS error level", REG32(M_EVSTAT), 8);
    check("MAILBOX rx_err in STATUS bit 25", REG32(M_STATUS), 1u << 25);
    check("MAILBOX error pending", REG32(M_PENDING), 8);
    REG32(M_PENDING) = 8;
    check("MAILBOX STATUS cleared on read", REG32(M_STATUS), 0);
}

static void done_events(void)
{
    take_irq2();
    REG32(IRQ19_PENDING) = 0xFu;
    REG32(M_DONE) = 1;
    check("MAILBOX DONE raises APB available", take_irq2(), 0x04);
    check("and its dupe in array 19", REG32(IRQ19_PENDING) & 0xFu, 1);
    REG32(IRQ19_PENDING) = 0xFu;
    check("MAILBOX DONE also marks MB_CLIENT available", REG32(C_PENDING), 1);
    REG32(C_PENDING) = 0xFu;

    REG32(A_DONE) = 2;
    check("APB DONE of 2 does nothing", REG32(M_PENDING), 0);
    REG32(A_DONE) = 1;
    check("APB DONE of 1 marks MAILBOX available", REG32(M_PENDING), 1);
    check("MBOX_APB DONE reads 0", REG32(A_DONE), 0);
    REG32(M_PENDING) = 1;
    check("MAILBOX pending cleared", REG32(M_PENDING), 0);
}

static void apb_full(void)
{
    uint32_t i, ok, st;

    for (i = 0; i < FIFO; i++)
        REG32(A_WDATA) = 0xC0000000u + i;
    check("MAILBOX holds 1025 words", rx_words(), FIFO);
    check("MBOX_APB still free at 1025", REG32(A_STATUS), TX_FREE);

    /* one more waits in the client and is flagged at once (mbox_client.v
     * 524), bit 5 being the client's tx_err */
    REG32(A_WDATA) = 0xC0000000u + FIFO;
    check("MAILBOX still 1025", rx_words(), FIFO);
    check("first word out", REG32(M_RDATA), 0xC0000000u);
    check("the waiting word moved in", rx_words(), FIFO);
    check("MBOX_APB tx_err kept, free again", REG32(A_STATUS), TX_FREE | 0x20u);
    ok = 1;
    for (i = 1; i <= FIFO; i++)
        if (REG32(M_RDATA) != 0xC0000000u + i)
            ok = 0;
    check("1025 words after it in order", ok, 1);
    take_irq2();

    for (i = 0; i < FIFO; i++)
        REG32(A_WDATA) = i;
    REG32(A_WDATA) = 0xDEADu;
    check("waiting word holds the client", REG32(A_STATUS), 0x20u);
    check("the STATUS read dropped it", REG32(A_STATUS), TX_FREE);
    for (i = 0; i < FIFO; i++)
        (void)REG32(M_RDATA);
    check("MAILBOX empty after the drain", rx_words(), 0);

    for (i = 0; i <= FIFO; i++)
        REG32(M_WDATA) = i;
    st = REG32(M_STATUS);
    check("MAILBOX tx_words full", (st >> 11) & 0x7FFu, FIFO);
    check("MAILBOX tx_err in bit 24", (st >> 24) & 3u, 1);
    check("MAILBOX error pending on overflow", REG32(M_PENDING), 8);
    REG32(M_PENDING) = 8;
    for (i = 0; i <= FIFO; i++)
        REG32(A_WDATA) = i;
    REG32(M_CONTROL) = 1;
    check("abort empties both FIFOs", REG32(M_STATUS) & 0x3FFFFFu, 0);
    check("waiting APB word taken and dropped", REG32(A_STATUS), TX_FREE | IN_PROG | 0x20u);
    REG32(A_ABORT) = 1;
    check("handshake finished", REG32(M_STATUS), 0);
    REG32(M_PENDING) = 0xFu;
    take_irq2();
}

static void aborts(void)
{
    take_irq2();
    REG32(M_CONTROL) = 1;
    check("MAILBOX abort in progress", REG32(M_STATUS) >> 22, 1);
    /* ack stays from the last handshake this side answered */
    check("MBOX_APB sees the abort", REG32(A_STATUS), TX_FREE | IN_PROG | ACK);
    check("APB abort_init irq", take_irq2(), 0x08);

    REG32(A_WDATA) = 0x5555u;
    check("word sent during the abort is dropped", rx_words(), 0);
    REG32(A_ABORT) = 2;
    check("APB ABORT of 2 does nothing", REG32(A_STATUS), TX_FREE | IN_PROG | ACK);
    REG32(A_ABORT) = 1;
    check("MBOX_APB acknowledged", REG32(A_STATUS), TX_FREE | ACK);
    check("MAILBOX abort done", REG32(M_STATUS) >> 22, 0);
    check("MAILBOX abort_done pending", REG32(M_PENDING), 4);
    REG32(M_PENDING) = 0xFu;

    REG32(A_ABORT) = 1;
    check("MBOX_APB abort in progress", REG32(A_STATUS), TX_FREE | IN_PROG);
    check("MAILBOX abort_init pending", REG32(M_PENDING), 2);
    check("MAILBOX in progress", REG32(M_STATUS) >> 22, 1);
    REG32(M_CONTROL) = 1;
    check("MAILBOX acknowledged", REG32(M_STATUS) >> 22, 2);
    check("MBOX_APB abort done, ack cleared", REG32(A_STATUS), TX_FREE);
    check("APB abort done irq", take_irq2(), 0x10);
    REG32(M_PENDING) = 0xFu;
}

static void loopback(void)
{
    uint32_t i, ok;

    /* MB_CLIENT saw every MAILBOX abort above, though not selected, and
     * still waits to answer the first; its answer goes nowhere */
    check("MB_CLIENT was asked to abort too", REG32(C_STATUS), TX_FREE | IN_PROG);
    check("MB_CLIENT abort_init pending", REG32(C_PENDING), 2);
    REG32(C_CONTROL) = 1;
    check("unselected MB_CLIENT answer goes nowhere", REG32(M_PENDING), 0);
    check("MB_CLIENT acknowledged", REG32(C_STATUS), TX_FREE | ACK);
    REG32(C_PENDING) = 0xFu;

    REG32(M_LOOPBACK) = 1;
    check("LOOPBACK reads back", REG32(M_LOOPBACK), 1);

    REG32(C_WDATA) = 0x12345678u;
    check("MB_CLIENT write lands in MAILBOX", REG32(M_RDATA), 0x12345678u);
    REG32(M_WDATA) = 0x9ABCDEF0u;
    check("MB_CLIENT rx_avail", REG32(C_STATUS), RX_AVAIL | TX_FREE | ACK);
    check("unselected APB sees the word", REG32(A_RDATA), 0x9ABCDEF0u);
    check("but does not take it", tx_words(), 1);
    check("MB_CLIENT RDATA", REG32(C_RDATA), 0x9ABCDEF0u);
    check("taken by MB_CLIENT", tx_words(), 0);

    REG32(A_WDATA) = 0x7777u;
    check("unselected APB write waits", REG32(A_STATUS), 0x20u);
    check("and never arrives", rx_words(), 0);

    REG32(C_DONE) = 1;
    check("MB_CLIENT DONE marks MAILBOX available", REG32(M_PENDING), 1);
    REG32(M_PENDING) = 1;
    take_irq2();
    REG32(A_DONE) = 1;
    check("unselected APB DONE is not seen", REG32(M_PENDING), 0);

    /* MB_CLIENT flags tx_err only for a write over a waiting word */
    for (i = 0; i < FIFO; i++)
        REG32(C_WDATA) = 0xE0000000u + i;
    REG32(C_WDATA) = 0xEEEEu;
    check("MB_CLIENT word waits, no error yet", REG32(C_STATUS), ACK);
    REG32(C_WDATA) = 0xFFFFu;
    check("second waiting write sets tx_err", REG32(C_STATUS), ACK | 0x10u);
    check("MB_CLIENT error pending", REG32(C_PENDING), 8);
    REG32(C_PENDING) = 8;
    ok = 1;
    for (i = 0; i < FIFO; i++)
        if (REG32(M_RDATA) != 0xE0000000u + i)
            ok = 0;
    check("1025 MB_CLIENT words in order", ok, 1);
    check("dropped by the STATUS read", rx_words(), 0);

    REG32(C_WDATA) = 1;
    REG32(C_WDATA) = 2;
    check("MB_CLIENT words after the error", REG32(M_RDATA) + REG32(M_RDATA), 3);
    (void)REG32(C_RDATA);
    check("MB_CLIENT empty read sets rx_err", REG32(C_STATUS), TX_FREE | ACK | 0x20u);
    REG32(C_PENDING) = 0xFu;

    /* both clients see the mailbox's abort; only MB_CLIENT answers it */
    REG32(M_CONTROL) = 1;
    check("MB_CLIENT asked to abort", REG32(C_STATUS), TX_FREE | IN_PROG | ACK);
    check("MB_CLIENT abort_init pending", REG32(C_PENDING), 2);
    REG32(C_CONTROL) = 1;
    check("MB_CLIENT acknowledged", REG32(C_STATUS), TX_FREE | ACK);
    check("MAILBOX abort done", REG32(M_PENDING), 4);
    check("APB still waiting to answer", REG32(A_STATUS) & IN_PROG, IN_PROG);
    REG32(A_ABORT) = 1;
    check("APB answer goes nowhere", REG32(M_PENDING), 4);
    REG32(M_PENDING) = 0xFu;
    REG32(C_PENDING) = 0xFu;
    take_irq2();
}

static void lines(void)
{
    __asm__ volatile ("csrs 0xBC0, %0" :: "r"((1u << 22) | (1u << 23)));
    check("lines 22 and 23 quiet", mip_vex() & (3u << 22), 0);
    REG32(M_ENABLE) = 1;
    REG32(C_DONE) = 1;
    check("MAILBOX available on line 22", mip_vex() & (3u << 22), 1u << 22);
    REG32(M_PENDING) = 1;
    REG32(C_ENABLE) = 1;
    REG32(M_DONE) = 1;
    check("MB_CLIENT available on line 23", mip_vex() & (3u << 22), 1u << 23);
    REG32(C_PENDING) = 0xFu;
    REG32(M_ENABLE) = 0;
    REG32(C_ENABLE) = 0;
    __asm__ volatile ("csrc 0xBC0, %0" :: "r"((1u << 22) | (1u << 23)));
}

static void on_array2(uint32_t pending)
{
    if (pending & 4u)
        irq_seen++;
    REG32(IRQ2_PENDING) = pending;
}

static void interrupt(void)
{
    REG32(M_LOOPBACK) = 0;
    take_irq2();
    irq_init();
    irq_set_handler(IRQ_ARRAY2, on_array2);
    irq_enable_events(IRQ_ARRAY2, 4u);
    REG32(M_DONE) = 1;
    __asm__ volatile ("nop; nop; nop; nop");
    check("handler saw APB available", irq_seen, 1);
    irq_disable(IRQ_ARRAY2);
}

int main(void)
{
    bao_init();
    mini_printf("\r\nmbox test\r\n");
    reset_values();
    apb_words();
    done_events();
    apb_full();
    aborts();
    loopback();
    lines();
    interrupt();
    mini_printf("[Done] mbox %d of %d passed\r\n", pass, total);
    while (1)
        __asm__ volatile ("wfi");
}
