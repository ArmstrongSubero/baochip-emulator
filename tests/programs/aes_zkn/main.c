/*
 * main.c: scalar AES instructions of the Baochip-1x VexRiscv for baoemu
 *
 * Copyright (c) 2026 Armstrong Subero
 * SPDX-License-Identifier: Apache-2.0
 *
 * aes32esi, aes32esmi, aes32dsi and aes32dsmi against FIPS-197.
 */

#include "bao.h"

#define ESI  0x11
#define ESMI 0x13
#define DSI  0x15
#define DSMI 0x17

#define AES32(f, bs, a, b) __extension__({                                     \
        uint32_t r_;                                                           \
        __asm__ volatile (".insn r 0x33, 0, %3, %0, %1, %2"                    \
                          : "=r"(r_) : "r"(a), "r"(b), "i"(((bs) << 5) | (f))); \
        r_; })

#define ROUND4(f, a, b) (AES32(f, 3, AES32(f, 2, AES32(f, 1, AES32(f, 0, (a), (b)), (b)), (b)), (b)))

static int passed, total;

static void hex32(uint32_t v)
{
    static const char d[] = "0123456789abcdef";
    char s[9];
    int i;
    for (i = 0; i < 8; i++)
        s[i] = d[(v >> (28 - 4 * i)) & 15];
    s[8] = 0;
    mini_printf("%s", s);
}

static void check(const char *name, uint32_t got, uint32_t want)
{
    total++;
    mini_printf("%s: ", name);
    hex32(got);
    if (got == want) {
        passed++;
        mini_printf(" PASS\r\n");
    } else {
        mini_printf(" FAIL want ");
        hex32(want);
        mini_printf("\r\n");
    }
}

/* reference from the Zkn formulas */

static uint8_t sbox[256], inv[256];

static uint8_t xt(uint8_t x)
{
    return (uint8_t)((x << 1) ^ ((x & 0x80) ? 0x1B : 0));
}

static uint8_t gm(uint8_t a, uint8_t c)
{
    uint8_t r = 0;
    while (c) {
        if (c & 1) r ^= a;
        a = xt(a);
        c >>= 1;
    }
    return r;
}

static void make_sbox(void)
{
    int x;
    for (x = 0; x < 256; x++) {
        uint8_t y = 0, s;
        int k;
        if (x)
            for (y = 1; gm((uint8_t)x, y) != 1; y++) {
            }
        s = y;
        for (k = 1; k < 5; k++)
            s ^= (uint8_t)((y << k) | (y >> (8 - k)));
        s ^= 0x63;
        sbox[x] = s;
        inv[s] = (uint8_t)x;
    }
}

static uint32_t rol(uint32_t v, int n)
{
    return n ? (v << n) | (v >> (32 - n)) : v;
}

static uint32_t ref(int f, int bs, uint32_t a, uint32_t b)
{
    uint8_t x = (uint8_t)(b >> (8 * bs)), s;
    uint32_t m;
    switch (f) {
    case ESI:
        m = sbox[x];
        break;
    case ESMI:
        s = sbox[x];
        m = (uint32_t)gm(s, 2) | (uint32_t)s << 8 | (uint32_t)s << 16 | (uint32_t)gm(s, 3) << 24;
        break;
    case DSI:
        m = inv[x];
        break;
    default:
        s = inv[x];
        m = (uint32_t)gm(s, 14) | (uint32_t)gm(s, 9) << 8 | (uint32_t)gm(s, 13) << 16 |
            (uint32_t)gm(s, 11) << 24;
        break;
    }
    return a ^ rol(m, 8 * bs);
}

static void per_instruction(void)
{
    static const uint32_t a = 0x01234567u, b = 0xA5C3E10Fu;
    int fail = 0;
    uint32_t r[16], i;

    r[0] = AES32(ESI, 0, a, b);  r[1] = AES32(ESI, 1, a, b);
    r[2] = AES32(ESI, 2, a, b);  r[3] = AES32(ESI, 3, a, b);
    r[4] = AES32(ESMI, 0, a, b); r[5] = AES32(ESMI, 1, a, b);
    r[6] = AES32(ESMI, 2, a, b); r[7] = AES32(ESMI, 3, a, b);
    r[8] = AES32(DSI, 0, a, b);  r[9] = AES32(DSI, 1, a, b);
    r[10] = AES32(DSI, 2, a, b); r[11] = AES32(DSI, 3, a, b);
    r[12] = AES32(DSMI, 0, a, b); r[13] = AES32(DSMI, 1, a, b);
    r[14] = AES32(DSMI, 2, a, b); r[15] = AES32(DSMI, 3, a, b);
    for (i = 0; i < 16; i++) {
        static const int fs[4] = { ESI, ESMI, DSI, DSMI };
        if (r[i] != ref(fs[i / 4], (int)(i % 4), a, b))
            fail++;
    }
    check("aes32esi bs 0", r[0], ref(ESI, 0, a, b));
    check("aes32esmi bs 1", r[5], ref(ESMI, 1, a, b));
    check("aes32dsi bs 2", r[10], ref(DSI, 2, a, b));
    check("aes32dsmi bs 3", r[15], ref(DSMI, 3, a, b));
    check("16 instruction and bs pairs wrong", (uint32_t)fail, 0);

    /* every byte value through each instruction, folded into one word */
    {
        uint32_t acc = 0, want = 0, v;
        for (v = 0; v < 256; v++) {
            uint32_t w = v * 0x01010101u;
            acc ^= rol(AES32(ESMI, 2, 0, w) ^ AES32(DSMI, 1, 0, w), (int)(v & 31)) ^
                   AES32(ESI, 3, v, w) ^ AES32(DSI, 0, v << 8, w);
            want ^= rol(ref(ESMI, 2, 0, w) ^ ref(DSMI, 1, 0, w), (int)(v & 31)) ^
                    ref(ESI, 3, v, w) ^ ref(DSI, 0, v << 8, w);
        }
        check("all 256 bytes folded", acc, want);
    }
}

/* AES with the four instructions, state and keys as little endian columns */

static uint32_t sub_word(uint32_t w)
{
    return ROUND4(ESI, 0, w);
}

static int expand(const uint8_t *key, int nk, uint32_t *rk)
{
    int nr = nk + 6, i;
    uint32_t rcon = 1;
    for (i = 0; i < nk; i++)
        rk[i] = (uint32_t)key[4 * i] | (uint32_t)key[4 * i + 1] << 8 |
                (uint32_t)key[4 * i + 2] << 16 | (uint32_t)key[4 * i + 3] << 24;
    for (i = nk; i < 4 * (nr + 1); i++) {
        uint32_t t = rk[i - 1];
        if (i % nk == 0) {
            t = sub_word((t >> 8) | (t << 24)) ^ rcon;
            rcon = xt((uint8_t)rcon);
        } else if (nk > 6 && i % nk == 4) {
            t = sub_word(t);
        }
        rk[i] = rk[i - nk] ^ t;
    }
    return nr;
}

static void encrypt(const uint32_t *rk, int nr, uint32_t *s)
{
    uint32_t s0 = s[0] ^ rk[0], s1 = s[1] ^ rk[1], s2 = s[2] ^ rk[2], s3 = s[3] ^ rk[3];
    uint32_t t0, t1, t2, t3;
    int r;
    for (r = 1; r < nr; r++) {
        const uint32_t *k = rk + 4 * r;
        t0 = AES32(ESMI, 3, AES32(ESMI, 2, AES32(ESMI, 1, AES32(ESMI, 0, k[0], s0), s1), s2), s3);
        t1 = AES32(ESMI, 3, AES32(ESMI, 2, AES32(ESMI, 1, AES32(ESMI, 0, k[1], s1), s2), s3), s0);
        t2 = AES32(ESMI, 3, AES32(ESMI, 2, AES32(ESMI, 1, AES32(ESMI, 0, k[2], s2), s3), s0), s1);
        t3 = AES32(ESMI, 3, AES32(ESMI, 2, AES32(ESMI, 1, AES32(ESMI, 0, k[3], s3), s0), s1), s2);
        s0 = t0; s1 = t1; s2 = t2; s3 = t3;
    }
    rk += 4 * nr;
    s[0] = AES32(ESI, 3, AES32(ESI, 2, AES32(ESI, 1, AES32(ESI, 0, rk[0], s0), s1), s2), s3);
    s[1] = AES32(ESI, 3, AES32(ESI, 2, AES32(ESI, 1, AES32(ESI, 0, rk[1], s1), s2), s3), s0);
    s[2] = AES32(ESI, 3, AES32(ESI, 2, AES32(ESI, 1, AES32(ESI, 0, rk[2], s2), s3), s0), s1);
    s[3] = AES32(ESI, 3, AES32(ESI, 2, AES32(ESI, 1, AES32(ESI, 0, rk[3], s3), s0), s1), s2);
}

/* equivalent inverse cipher: middle round keys go through InvMixColumns */
static void decrypt(const uint32_t *rk, int nr, uint32_t *s)
{
    uint32_t s0, s1, s2, s3, t0, t1, t2, t3, k[4];
    int r, j;
    const uint32_t *e = rk + 4 * nr;
    s0 = s[0] ^ e[0]; s1 = s[1] ^ e[1]; s2 = s[2] ^ e[2]; s3 = s[3] ^ e[3];
    for (r = nr - 1; r > 0; r--) {
        for (j = 0; j < 4; j++)
            k[j] = ROUND4(DSMI, 0, sub_word(rk[4 * r + j]));
        t0 = AES32(DSMI, 3, AES32(DSMI, 2, AES32(DSMI, 1, AES32(DSMI, 0, k[0], s0), s3), s2), s1);
        t1 = AES32(DSMI, 3, AES32(DSMI, 2, AES32(DSMI, 1, AES32(DSMI, 0, k[1], s1), s0), s3), s2);
        t2 = AES32(DSMI, 3, AES32(DSMI, 2, AES32(DSMI, 1, AES32(DSMI, 0, k[2], s2), s1), s0), s3);
        t3 = AES32(DSMI, 3, AES32(DSMI, 2, AES32(DSMI, 1, AES32(DSMI, 0, k[3], s3), s2), s1), s0);
        s0 = t0; s1 = t1; s2 = t2; s3 = t3;
    }
    s[0] = AES32(DSI, 3, AES32(DSI, 2, AES32(DSI, 1, AES32(DSI, 0, rk[0], s0), s3), s2), s1);
    s[1] = AES32(DSI, 3, AES32(DSI, 2, AES32(DSI, 1, AES32(DSI, 0, rk[1], s1), s0), s3), s2);
    s[2] = AES32(DSI, 3, AES32(DSI, 2, AES32(DSI, 1, AES32(DSI, 0, rk[2], s2), s1), s0), s3);
    s[3] = AES32(DSI, 3, AES32(DSI, 2, AES32(DSI, 1, AES32(DSI, 0, rk[3], s3), s2), s1), s0);
}

static void print_block(const char *what, const uint32_t *s)
{
    static const char d[] = "0123456789abcdef";
    char t[33];
    int i;
    for (i = 0; i < 16; i++) {
        uint8_t v = (uint8_t)(s[i / 4] >> (8 * (i % 4)));
        t[2 * i] = d[v >> 4];
        t[2 * i + 1] = d[v & 15];
    }
    t[32] = 0;
    mini_printf("%s %s\r\n", what, t);
}

static void fips197(const char *name, int nk, const uint32_t *want)
{
    static const uint32_t pt[4] = { 0x33221100u, 0x77665544u, 0xBBAA9988u, 0xFFEEDDCCu };
    uint8_t key[32];
    uint32_t rk[60], s[4];
    int i, nr, ok;

    for (i = 0; i < 4 * nk; i++)
        key[i] = (uint8_t)i;
    nr = expand(key, nk, rk);
    for (i = 0; i < 4; i++)
        s[i] = pt[i];
    encrypt(rk, nr, s);
    mini_printf("%s ", name);
    print_block("cipher", s);
    ok = s[0] == want[0] && s[1] == want[1] && s[2] == want[2] && s[3] == want[3];
    total++;
    passed += ok;
    mini_printf("%s encrypt: %s\r\n", name, ok ? "PASS" : "FAIL");
    decrypt(rk, nr, s);
    ok = s[0] == pt[0] && s[1] == pt[1] && s[2] == pt[2] && s[3] == pt[3];
    total++;
    passed += ok;
    mini_printf("%s decrypt: %s\r\n", name, ok ? "PASS" : "FAIL");
}

/* illegal encodings: the handler records mcause and mtval and steps over */
volatile uint32_t trap_cause, trap_val;
extern char aes_trap[];
__asm__(
"   .section .text.aes_trap, \"ax\"\n"
"   .option push\n"
"   .option norvc\n"
"   .balign 4\n"
"   .globl aes_trap\n"
"aes_trap:\n"
"   addi sp, sp, -16\n"
"   sw t5, 0(sp)\n"
"   sw t6, 4(sp)\n"
"   la t6, trap_cause\n"
"   csrr t5, mcause\n"
"   sw t5, 0(t6)\n"
"   la t6, trap_val\n"
"   csrr t5, mtval\n"
"   sw t5, 0(t6)\n"
"   csrr t6, mepc\n"
"   addi t6, t6, 4\n"
"   csrw mepc, t6\n"
"   lw t5, 0(sp)\n"
"   lw t6, 4(sp)\n"
"   addi sp, sp, 16\n"
"   mret\n"
"   .option pop\n"
"   .text\n"
);

/* runs .insn r 0x33, f3, f7, a0, a1, a2 and returns mcause, all ones for no trap */
#define TRY(f3, f7) __extension__({                                            \
        trap_cause = 0xFFFFFFFFu;                                              \
        __asm__ volatile ("li a1, 0x11223344\n li a2, 0x55667788\n"            \
                          ".insn r 0x33, %0, %1, a0, a1, a2\n"                 \
                          : : "i"(f3), "i"(f7) : "a0", "a1", "a2", "memory");  \
        trap_cause; })

static void legality(void)
{
    uint32_t saved, c;
    __asm__ volatile ("csrr %0, mtvec" : "=r"(saved));
    __asm__ volatile ("csrw mtvec, %0" :: "r"(aes_trap));
    check("funct3 1 with aes32esi funct7 traps", TRY(1, 0x11), 2);
    check("its mtval is the instruction", trap_val,
          (0x11u << 25) | (12u << 20) | (11u << 15) | (1u << 12) | (10u << 7) | 0x33u);
    c = TRY(2, 0x13) | TRY(3, 0x15) | TRY(4, 0x17) | TRY(5, 0x31) | TRY(6, 0x77) | TRY(7, 0x55);
    check("funct3 2 to 7 trap", c, 2);
    check("funct7 0x19 (bit 28) traps", TRY(0, 0x19), 2);
    check("funct7 0x10 (bit 25 clear) traps", TRY(0, 0x10), 2);
    check("funct7 0x21 (bit 29 clear) traps", TRY(0, 0x21), 2);
    c = TRY(0, 0x11) & TRY(0, 0x13) & TRY(0, 0x15) & TRY(0, 0x17) &
        TRY(0, 0x31) & TRY(0, 0x33) & TRY(0, 0x35) & TRY(0, 0x37) &
        TRY(0, 0x51) & TRY(0, 0x53) & TRY(0, 0x55) & TRY(0, 0x57) &
        TRY(0, 0x71) & TRY(0, 0x73) & TRY(0, 0x75) & TRY(0, 0x77);
    check("the 16 aes32 funct7 values run", c, 0xFFFFFFFFu);
    __asm__ volatile ("csrw mtvec, %0" :: "r"(saved));
}

/* result latency, timed like cpu_timing */
static uint32_t timed(int use)
{
    uint32_t d, n;
    for (n = 0; n < 2; n++) {
        if (use)
            __asm__ volatile (
                ".option push\n .option norvc\n .balign 4\n"
                "csrr t0, mcycle\n"
                ".insn r 0x33, 0, 0x11, t2, t4, t5\n"
                "addi t6, t2, 1\n"
                "nop\n nop\n nop\n nop\n"
                "csrr t1, mcycle\n"
                "sub %0, t1, t0\n"
                ".option pop\n"
                : "=r"(d) : : "t0", "t1", "t2", "t4", "t5", "t6");
        else
            __asm__ volatile (
                ".option push\n .option norvc\n .balign 4\n"
                "csrr t0, mcycle\n"
                "nop\n nop\n nop\n nop\n"
                "csrr t1, mcycle\n"
                "sub %0, t1, t0\n"
                ".option pop\n"
                : "=r"(d) : : "t0", "t1", "t2", "t4", "t5", "t6");
    }
    return d;
}

/* a0 = aes(a0, a1) held behind an uncached load */
#define HELD(name, pre, aes, next) static uint32_t name(uint32_t in)            \
    {                                                                          \
        uint32_t r;                                                            \
        __asm__ volatile (                                                     \
            ".option push\n .option norvc\n .balign 4\n"                       \
            "li t3, 0xE000F010\n"                                              \
            "li t4, 7\n li t5, 3\n li t2, 0x0A\n li a0, 0\n li a1, 0x55\n"     \
            "li a2, 0\n li a3, 0x21\n li a4, 0x5A000000\n"                     \
            pre aes next                                                       \
            "nop\n nop\n nop\n"                                                \
            "mv %0, a0\n"                                                      \
            ".option pop\n"                                                    \
            : "=r"(r) : "r"(in) : "t2", "t3", "t4", "t5", "t6", "a0", "a1",    \
              "a2", "a3", "a4", "memory");                                     \
        return r;                                                              \
    }

#define ESI_A0 ".insn r 0x33, 0, 0x11, a0, a0, a1\n"

HELD(h_gap, "lw t2, 0(t3)\n nop\n", ESI_A0, "add t6, t4, t5\n")
HELD(h_add, "lw t2, 0(t3)\n", ESI_A0, "add t6, t4, t5\n")
HELD(h_load_rd, "lw t2, 0(t3)\n", ESI_A0, "add t6, t4, t2\n")
HELD(h_aes_rd, "li a0, 0x0C\n lw t2, 0(t3)\n", ".insn r 0x33, 0, 0x11, a0, zero, a1\n",
     "add t6, t4, a0\n")
HELD(h_amo, "amoor.w zero, zero, (%1)\n", ESI_A0, "add t6, t4, t5\n")
HELD(h_amo_gap, "amoor.w zero, zero, (%1)\n nop\n", ESI_A0, "add t6, t4, t5\n")
HELD(h_bank1, "lw t2, 0(t3)\n", ".insn r 0x33, 0, 0x33, a0, zero, a1\n",
     ".insn r 0x33, 0, 0x77, t6, zero, a4\n")
HELD(h_rvc, "lw t2, 0(t3)\n", ESI_A0, ".option rvc\n c.add a2, a3\n .option norvc\n")
HELD(h_dep, "lw t2, 0(t3)\n", ".insn r 0x33, 0, 0x11, a0, zero, t2\n", "add t6, t4, t5\n")
HELD(h_load_t2, "lw t2, 0(t3)\n", "mv a0, t2\n", "nop\n")

static void held(void)
{
    volatile uint32_t word = 0;
    uint32_t p = (uint32_t)&word, v;
    uint8_t i = inv[0x5A];

    check("uncached load, nop, aes32esi", h_gap(p), sbox[0x55]);
    check("uncached load, aes32esi, add from t5", h_add(p), sbox[3]);
    check("next reads the load's rd: value before it", h_load_rd(p), sbox[0x0A]);
    check("next reads the aes rd: value before it", h_aes_rd(p), sbox[0x0C]);
    check("amoor.w, aes32esi, add from t5", h_amo(p), sbox[3]);
    check("amoor.w, nop, aes32esi", h_amo_gap(p), sbox[0x55]);
    check("next is aes32dsmi bs 3: bank 1", h_bank1(p),
          (uint32_t)gm(i, 13) | (uint32_t)gm(i, 9) << 8 | (uint32_t)gm(i, 14) << 16 |
          (uint32_t)gm(i, 14) << 24);
    check("next is c.add a2, a3", h_rvc(p), sbox[0x21]);
    v = h_load_t2(p);
    check("aes32esi waiting for the load's rd", h_dep(p), sbox[v & 0xFF]);
}

int main(void)
{
    bao_init();
    mini_printf("\r\naes_zkn: scalar AES instructions\r\n");
    make_sbox();

    per_instruction();
    {
        static const uint32_t c128[4] = { 0xD8E0C469u, 0x30047B6Au, 0x80B7CDD8u, 0x5AC5B470u };
        static const uint32_t c256[4] = { 0xCAB7A28Eu, 0xBF456751u, 0x9049FCEAu, 0x8960494Bu };
        fips197("AES-128", 4, c128);
        fips197("AES-256", 8, c256);
    }
    {
        uint32_t z;
        __asm__ volatile ("li a1, 0x1234\n .insn r 0x33, 0, 0x13, zero, a1, a1\n mv %0, zero"
                          : "=r"(z) : : "a1");
        check("rd x0 stays 0", z, 0);
    }
    legality();
    check("aes32esi then use, cycles", timed(1) - timed(0), 4);
    held();

    mini_printf("[Done] aes_zkn %d of %d passed\r\n", passed, total);
    for (;;) {
    }
}
