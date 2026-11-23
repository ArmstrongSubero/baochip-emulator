/*
 * sce.c: SCE TRNG, SHA-256, SHA-512, HMAC and AES models
 *
 * Copyright (c) 2026 Armstrong Subero
 * SPDX-License-Identifier: Apache-2.0
 *
 * Engines run at once on a write of 0x5A to their AR register.
 */

#include <string.h>
#include "bao_int.h"

#define SCE_BASE   0x40020000u
#define CTRL       0x40028000u
#define HASH       0x4002B000u
#define AES        0x4002D000u
#define TRNG       0x4002E000u

uint32_t *bao_mmio_slot(bao_t *b, uint32_t addr);

static uint32_t rd(bao_t *b, uint32_t a)
{
    uint32_t *s = bao_mmio_slot(b, a);
    return s ? *s : 0;
}

static void wr(bao_t *b, uint32_t a, uint32_t v)
{
    uint32_t *s = bao_mmio_slot(b, a);
    if (s) *s = v;
}

/* shared with the scalar AES instructions in cpu.c */
const uint8_t bao_aes_sbox[256] = {
    0x63,0x7c,0x77,0x7b,0xf2,0x6b,0x6f,0xc5,0x30,0x01,0x67,0x2b,0xfe,0xd7,0xab,0x76,
    0xca,0x82,0xc9,0x7d,0xfa,0x59,0x47,0xf0,0xad,0xd4,0xa2,0xaf,0x9c,0xa4,0x72,0xc0,
    0xb7,0xfd,0x93,0x26,0x36,0x3f,0xf7,0xcc,0x34,0xa5,0xe5,0xf1,0x71,0xd8,0x31,0x15,
    0x04,0xc7,0x23,0xc3,0x18,0x96,0x05,0x9a,0x07,0x12,0x80,0xe2,0xeb,0x27,0xb2,0x75,
    0x09,0x83,0x2c,0x1a,0x1b,0x6e,0x5a,0xa0,0x52,0x3b,0xd6,0xb3,0x29,0xe3,0x2f,0x84,
    0x53,0xd1,0x00,0xed,0x20,0xfc,0xb1,0x5b,0x6a,0xcb,0xbe,0x39,0x4a,0x4c,0x58,0xcf,
    0xd0,0xef,0xaa,0xfb,0x43,0x4d,0x33,0x85,0x45,0xf9,0x02,0x7f,0x50,0x3c,0x9f,0xa8,
    0x51,0xa3,0x40,0x8f,0x92,0x9d,0x38,0xf5,0xbc,0xb6,0xda,0x21,0x10,0xff,0xf3,0xd2,
    0xcd,0x0c,0x13,0xec,0x5f,0x97,0x44,0x17,0xc4,0xa7,0x7e,0x3d,0x64,0x5d,0x19,0x73,
    0x60,0x81,0x4f,0xdc,0x22,0x2a,0x90,0x88,0x46,0xee,0xb8,0x14,0xde,0x5e,0x0b,0xdb,
    0xe0,0x32,0x3a,0x0a,0x49,0x06,0x24,0x5c,0xc2,0xd3,0xac,0x62,0x91,0x95,0xe4,0x79,
    0xe7,0xc8,0x37,0x6d,0x8d,0xd5,0x4e,0xa9,0x6c,0x56,0xf4,0xea,0x65,0x7a,0xae,0x08,
    0xba,0x78,0x25,0x2e,0x1c,0xa6,0xb4,0xc6,0xe8,0xdd,0x74,0x1f,0x4b,0xbd,0x8b,0x8a,
    0x70,0x3e,0xb5,0x66,0x48,0x03,0xf6,0x0e,0x61,0x35,0x57,0xb9,0x86,0xc1,0x1d,0x9e,
    0xe1,0xf8,0x98,0x11,0x69,0xd9,0x8e,0x94,0x9b,0x1e,0x87,0xe9,0xce,0x55,0x28,0xdf,
    0x8c,0xa1,0x89,0x0d,0xbf,0xe6,0x42,0x68,0x41,0x99,0x2d,0x0f,0xb0,0x54,0xbb,0x16
};

const uint8_t bao_aes_inv_sbox[256] = {
    0x52,0x09,0x6a,0xd5,0x30,0x36,0xa5,0x38,0xbf,0x40,0xa3,0x9e,0x81,0xf3,0xd7,0xfb,
    0x7c,0xe3,0x39,0x82,0x9b,0x2f,0xff,0x87,0x34,0x8e,0x43,0x44,0xc4,0xde,0xe9,0xcb,
    0x54,0x7b,0x94,0x32,0xa6,0xc2,0x23,0x3d,0xee,0x4c,0x95,0x0b,0x42,0xfa,0xc3,0x4e,
    0x08,0x2e,0xa1,0x66,0x28,0xd9,0x24,0xb2,0x76,0x5b,0xa2,0x49,0x6d,0x8b,0xd1,0x25,
    0x72,0xf8,0xf6,0x64,0x86,0x68,0x98,0x16,0xd4,0xa4,0x5c,0xcc,0x5d,0x65,0xb6,0x92,
    0x6c,0x70,0x48,0x50,0xfd,0xed,0xb9,0xda,0x5e,0x15,0x46,0x57,0xa7,0x8d,0x9d,0x84,
    0x90,0xd8,0xab,0x00,0x8c,0xbc,0xd3,0x0a,0xf7,0xe4,0x58,0x05,0xb8,0xb3,0x45,0x06,
    0xd0,0x2c,0x1e,0x8f,0xca,0x3f,0x0f,0x02,0xc1,0xaf,0xbd,0x03,0x01,0x13,0x8a,0x6b,
    0x3a,0x91,0x11,0x41,0x4f,0x67,0xdc,0xea,0x97,0xf2,0xcf,0xce,0xf0,0xb4,0xe6,0x73,
    0x96,0xac,0x74,0x22,0xe7,0xad,0x35,0x85,0xe2,0xf9,0x37,0xe8,0x1c,0x75,0xdf,0x6e,
    0x47,0xf1,0x1a,0x71,0x1d,0x29,0xc5,0x89,0x6f,0xb7,0x62,0x0e,0xaa,0x18,0xbe,0x1b,
    0xfc,0x56,0x3e,0x4b,0xc6,0xd2,0x79,0x20,0x9a,0xdb,0xc0,0xfe,0x78,0xcd,0x5a,0xf4,
    0x1f,0xdd,0xa8,0x33,0x88,0x07,0xc7,0x31,0xb1,0x12,0x10,0x59,0x27,0x80,0xec,0x5f,
    0x60,0x51,0x7f,0xa9,0x19,0xb5,0x4a,0x0d,0x2d,0xe5,0x7a,0x9f,0x93,0xc9,0x9c,0xef,
    0xa0,0xe0,0x3b,0x4d,0xae,0x2a,0xf5,0xb0,0xc8,0xeb,0xbb,0x3c,0x83,0x53,0x99,0x61,
    0x17,0x2b,0x04,0x7e,0xba,0x77,0xd6,0x26,0xe1,0x69,0x14,0x63,0x55,0x21,0x0c,0x7d
};

static uint8_t xt(uint8_t x) { return (uint8_t)((x << 1) ^ ((x & 0x80) ? 0x1B : 0)); }

uint8_t bao_gmul(uint8_t a, uint8_t c)
{
    uint8_t r = 0;
    while (c) {
        if (c & 1) r ^= a;
        a = xt(a);
        c >>= 1;
    }
    return r;
}

/* sub4 is the extra SubWord of the 256 bit schedule, which AesCtrl.v only
 * gives for key length 2 (Nk8Mod4); length 3 runs the 256 bit schedule
 * without it */
static int aes_expand(const uint8_t *key, int nk, int sub4, uint8_t *rk)
{
    int nr = nk + 6, i;
    uint8_t rcon = 1, t[4];
    memcpy(rk, key, (size_t)(4 * nk));
    for (i = nk; i < 4 * (nr + 1); i++) {
        memcpy(t, rk + 4 * (i - 1), 4);
        if (i % nk == 0) {
            uint8_t u = t[0];
            t[0] = bao_aes_sbox[t[1]] ^ rcon; t[1] = bao_aes_sbox[t[2]]; t[2] = bao_aes_sbox[t[3]]; t[3] = bao_aes_sbox[u];
            rcon = xt(rcon);
        } else if (sub4 && i % nk == 4) {
            t[0] = bao_aes_sbox[t[0]]; t[1] = bao_aes_sbox[t[1]]; t[2] = bao_aes_sbox[t[2]]; t[3] = bao_aes_sbox[t[3]];
        }
        rk[4 * i + 0] = rk[4 * (i - nk) + 0] ^ t[0];
        rk[4 * i + 1] = rk[4 * (i - nk) + 1] ^ t[1];
        rk[4 * i + 2] = rk[4 * (i - nk) + 2] ^ t[2];
        rk[4 * i + 3] = rk[4 * (i - nk) + 3] ^ t[3];
    }
    return nr;
}

static void aes_block(const uint8_t *rk, int nr, uint8_t *s, int dec)
{
    int r, i, c;
    uint8_t t[16];

    if (!dec) {
        for (i = 0; i < 16; i++) s[i] ^= rk[i];
        for (r = 1; r <= nr; r++) {
            for (i = 0; i < 16; i++) t[i] = bao_aes_sbox[s[(i + 4 * (i % 4)) % 16]];
            if (r != nr) {
                for (c = 0; c < 4; c++) {
                    uint8_t *p = t + 4 * c, a0 = p[0], a1 = p[1], a2 = p[2], a3 = p[3];
                    p[0] = (uint8_t)(xt(a0) ^ xt(a1) ^ a1 ^ a2 ^ a3);
                    p[1] = (uint8_t)(a0 ^ xt(a1) ^ xt(a2) ^ a2 ^ a3);
                    p[2] = (uint8_t)(a0 ^ a1 ^ xt(a2) ^ xt(a3) ^ a3);
                    p[3] = (uint8_t)(xt(a0) ^ a0 ^ a1 ^ a2 ^ xt(a3));
                }
            }
            for (i = 0; i < 16; i++) s[i] = t[i] ^ rk[16 * r + i];
        }
    } else {
        for (i = 0; i < 16; i++) s[i] ^= rk[16 * nr + i];
        for (r = nr - 1; r >= 0; r--) {
            for (i = 0; i < 16; i++) t[(i + 4 * (i % 4)) % 16] = bao_aes_inv_sbox[s[i]];
            for (i = 0; i < 16; i++) t[i] ^= rk[16 * r + i];
            if (r != 0) {
                for (c = 0; c < 4; c++) {
                    uint8_t *p = t + 4 * c, a0 = p[0], a1 = p[1], a2 = p[2], a3 = p[3];
                    p[0] = bao_gmul(a0, 14) ^ bao_gmul(a1, 11) ^ bao_gmul(a2, 13) ^ bao_gmul(a3, 9);
                    p[1] = bao_gmul(a0, 9) ^ bao_gmul(a1, 14) ^ bao_gmul(a2, 11) ^ bao_gmul(a3, 13);
                    p[2] = bao_gmul(a0, 13) ^ bao_gmul(a1, 9) ^ bao_gmul(a2, 14) ^ bao_gmul(a3, 11);
                    p[3] = bao_gmul(a0, 11) ^ bao_gmul(a1, 13) ^ bao_gmul(a2, 9) ^ bao_gmul(a3, 14);
                }
            }
            memcpy(s, t, 16);
        }
    }
}

/* AKEY, AIB and AOB segments, 64 words each (scedma_pkg.sv 226 to 228) */
#define SEG_AKEY (SCE_BASE + 0x1C00)
#define SEG_AIB  (SCE_BASE + 0x1D00)
#define SEG_AOB  (SCE_BASE + 0x1E00)

enum { AR_I, AR_IV, AR_O, AR_X };

static uint32_t bswap32(uint32_t v)
{
    return (v >> 24) | ((v >> 8) & 0xFF00u) | ((v << 8) & 0xFF0000u) | (v << 24);
}

static void words_to_bytes(const uint32_t *w, uint8_t *s, int n)
{
    int i;
    for (i = 0; i < n; i++) {
        s[4 * i] = (uint8_t)(w[i] >> 24); s[4 * i + 1] = (uint8_t)(w[i] >> 16);
        s[4 * i + 2] = (uint8_t)(w[i] >> 8); s[4 * i + 3] = (uint8_t)w[i];
    }
}

/* one channel run between a segment and the AES RAM (sce_dmachnl.sv 123 to 129) */
static uint32_t seg_move(bao_t *b, uint32_t seg, uint32_t p, int swap, uint32_t *w, int n, int store)
{
    int i;
    if (p > 63) {
        /* the channel's reset of an out of range pointer depends on bus timing */
        if (bao_first_time(b, AES + 0x30))
            bao_logf(b, BAO_LOG_WARN, "AES segment pointer %u is past the segment end, taken modulo 64", p);
        p &= 63;
    }
    for (i = 0; i < n; i++) {
        if (i) p = p == 63 ? 0 : p + 1;
        if (store) wr(b, seg + 4 * p, swap ? bswap32(w[i]) : w[i]);
        else w[i] = swap ? bswap32(rd(b, seg + 4 * p)) : rd(b, seg + 4 * p);
    }
    return p;
}

/* the aes.sv block steps and the core's mode steps, in RTL order */
static void aes_run(bao_t *b)
{
    uint32_t fn = rd(b, AES + 0x00) & 0xFF, opt = rd(b, AES + 0x10), ltx = rd(b, AES + 0x18);
    uint32_t klen = opt & 15, mode = (opt >> 4) & 15, sp[4], i;
    uint32_t (*m)[4] = b->aes_aram;

    for (i = 0; i < 4; i++)
        sp[i] = rd(b, AES + 0x30 + 4 * i) & 0xFFF;

    if (fn == 0) {
        /* aes.sv 314 loads 4, 6 or 8 words; the core takes AesLen from bits 1:0 */
        uint32_t kw[8];
        uint8_t key[32];
        int len = (int)(klen & 3), nk = len == 0 ? 4 : len == 1 ? 6 : 8;
        seg_move(b, SEG_AKEY, sp[1], (ltx >> 1) & 1, kw, klen == 0 ? 4 : klen == 1 ? 6 : 8, 0);
        words_to_bytes(kw, key, nk);
        b->aes_nr = aes_expand(key, nk, len == 2, b->aes_rk);
        sce_done(b, 1, 0x0B);
        return;
    }
    if (fn != 1 && fn != 2) {
        bao_logf(b, BAO_LOG_WARN, "AES function 0x%02X does not exist, the engine stays idle", fn);
        return;
    }
    {
        /* ECB and CBC use the inverse cipher, other modes the forward (aes.sv 247, 292) */
        int ecb_cbc = mode == 0 || mode == 1, core_dec = fn == 2 && ecb_cbc;
        uint32_t amode = (mode + 1) & 7, cnt = rd(b, AES + 0x14) & 0xFFFF, n, dr = 0, dw = 0, last;

        for (n = 0; n <= cnt; n++) {
            uint32_t t[4];
            uint8_t s[16];
            const uint32_t *src;

            seg_move(b, SEG_AKEY, sp[0], ltx & 1, m[ecb_cbc ? AR_IV : AR_I], 4, 0);
            last = seg_move(b, SEG_AIB, sp[2] + dr, (ltx >> 2) & 1, m[ecb_cbc ? AR_I : AR_IV], 4, 0);
            dr = last == 63 ? 0 : last + 1;

            if (amode == 2 && !core_dec)        /* AesCtrl.v 371, 426 to 445 */
                for (i = 0; i < 4; i++) m[AR_I][i] ^= m[AR_IV][i];
            words_to_bytes(m[AR_I], s, 4);
            aes_block(b->aes_rk, b->aes_nr, s, core_dec);
            for (i = 0; i < 4; i++)
                m[AR_O][i] = ((uint32_t)s[4 * i] << 24) | ((uint32_t)s[4 * i + 1] << 16) |
                             ((uint32_t)s[4 * i + 2] << 8) | s[4 * i + 3];
            /* AesCtrl.v 483, 555 to 570: OFB leaves the cipher output in O */
            if ((amode == 2 && core_dec) || amode == 3 || amode == 4 || amode == 5)
                for (i = 0; i < 4; i++) m[amode == 4 ? AR_X : AR_O][i] = m[AR_O][i] ^ m[AR_IV][i];

            /* WB_IV source (aes.sv 397 to 402) */
            src = mode == 0 ? m[AR_IV] : mode == 1 ? m[fn == 1 ? AR_O : AR_I] : mode == 2 ? m[AR_I] :
                  mode == 4 ? m[fn == 1 ? AR_O : AR_IV] : m[AR_O];
            memcpy(t, src, sizeof(t));
            if (mode == 2) {
                /* CTR adds one to word 0, or word 3 with optltx bit 5, read
                 * byte reversed with bit 4 (aes.sv 456 to 465) */
                int w = (ltx >> 5) & 1 ? 3 : 0;
                t[w] = (ltx >> 4) & 1 ? bswap32(bswap32(t[w]) + 1) : t[w] + 1;
            }
            seg_move(b, SEG_AKEY, sp[0], ltx & 1, t, 4, 1);
            last = seg_move(b, SEG_AOB, sp[3] + dw, (ltx >> 3) & 1, m[mode == 3 ? AR_X : AR_O], 4, 1);
            dw = last == 63 ? 0 : last + 1;
        }
        sce_done(b, 1, 0x0F);
    }
}

/* Hash engine (combohasha.sv, hashcore.sv) with its own 768 word HRAM
 * (hash_pkg.sv 92 to 108). A RAM clear only clears from word 428 up. */

#define HR_SHA256_H  0u
#define HR_SHA256_K  8u
#define HR_SHA512_H  72u
#define HR_SHA512_K  88u
#define HR_INITSIZE  428u
#define HR_ST        0x200u
#define HR_MSG       0x220u

/* SCE RAM segments the hash channels use, in words (scedma_pkg.sv 211 to
 * 217), indexed by segment id; the segment pointer registers at 0x20 are
 * cr_segptrstart[id] and optltx bit id is that channel's byte reversal */
enum { HS_LKEY, HS_KEY, HS_SKEY, HS_SCRT, HS_MSG, HS_HOUT, HS_SOB, HS_HOUT2 };
static const uint16_t hseg_addr[7] = {0, 64, 128, 192, 256, 384, 448};
static const uint16_t hseg_size[7] = {64, 64, 64, 64, 128, 64, 64};

#define ROR(x, n)   (((x) >> (n)) | ((x) << (32 - (n))))
#define ROR64(x, n) (((x) >> (n)) | ((x) << (64 - (n))))

typedef struct {
    uint32_t fn, cnt, opt2, ltx, ptr[8];
    int is512;
    uint32_t msgsize, stsize;     /* combohasha.sv 525 to 535 */
    uint32_t fr;                  /* FR bits the run passed through */
} hrun_t;

/* A channel start pointer outside its segment is reset by the channel at
 * a time that depends on the bus (sce_dmachnl.sv 123); taken modulo here */
static uint32_t hseg_ptr(bao_t *b, int seg, uint32_t p)
{
    if (p >= hseg_size[seg]) {
        if (bao_first_time(b, HASH + 0x20))
            bao_logf(b, BAO_LOG_WARN, "hash segment pointer %u is past the segment end, taken modulo %u",
                     p, hseg_size[seg]);
        p %= hseg_size[seg];
    }
    return p;
}

/* chnli: n words from segment seg at p into HRAM at dst */
static uint32_t hseg_load(bao_t *b, hrun_t *r, int seg, int ltxbit, uint32_t p, uint32_t dst, uint32_t n)
{
    uint32_t i, v;
    int swap = (int)((r->ltx >> ltxbit) & 1);
    p = hseg_ptr(b, seg, p);
    for (i = 0; i < n; i++) {
        if (i) p = p == hseg_size[seg] - 1u ? 0 : p + 1;
        v = rd(b, SCE_BASE + 4u * (hseg_addr[seg] + p));
        b->hash_ram[dst + i] = swap ? bswap32(v) : v;
    }
    r->fr |= 0x08;                                  /* chnli_done */
    return p;
}

/* chnlo: STSIZE words of ST into segment seg (combohasha.sv 738 to 776) */
static void hseg_store(bao_t *b, hrun_t *r, int seg, int pre)
{
    uint32_t i, v, p = hseg_ptr(b, seg, r->ptr[HS_HOUT2]);
    int swap = (int)((r->ltx >> HS_HOUT2) & 1);
    for (i = 0; i < r->stsize; i++) {
        if (i) p = p == hseg_size[seg] - 1u ? 0 : p + 1;
        v = b->hash_ram[HR_ST + i];
        if (swap) v = bswap32(v);
        if (pre && i == 0) v = (v & 0xF0FFFFFFu) | 0x08000000u;
        if (pre && i == 7) v = (v & ~0xC0u) | 0x40u;
        wr(b, SCE_BASE + 4u * (hseg_addr[seg] + p), v);
    }
    r->fr |= 0x04;                                  /* chnlo_done */
}

static uint64_t hr64(const uint32_t *m, uint32_t i)
{
    return ((uint64_t)m[2 * i] << 32) | m[2 * i + 1];
}

static void hw64(uint32_t *m, uint32_t i, uint64_t v)
{
    m[2 * i] = (uint32_t)(v >> 32);
    m[2 * i + 1] = (uint32_t)v;
}

/* one hashcore run on HRAM (hashcore.sv 544 to 675) */
static void hash_core(bao_t *b, hrun_t *r, int first)
{
    uint32_t *h = b->hash_ram, *m = h + HR_MSG, i;
    uint32_t base = first ? (r->is512 ? HR_SHA512_H : HR_SHA256_H) : HR_ST;

    if (!r->is512) {
        uint32_t v[8], t1, t2, *k = h + HR_SHA256_K;
        for (i = 0; i < 8; i++) v[i] = h[base + i];
        for (i = 0; i < 64; i++) {
            uint32_t a1 = m[i + 1], a14 = m[i + 14];
            m[i + 16] = m[i] + (ROR(a1, 7) ^ ROR(a1, 18) ^ (a1 >> 3)) + m[i + 9] +
                        (ROR(a14, 17) ^ ROR(a14, 19) ^ (a14 >> 10));
            t1 = v[7] + (ROR(v[4], 6) ^ ROR(v[4], 11) ^ ROR(v[4], 25)) +
                 ((v[4] & v[5]) | (~v[4] & v[6])) + k[i] + m[i];
            t2 = (ROR(v[0], 2) ^ ROR(v[0], 13) ^ ROR(v[0], 22)) +
                 ((v[0] & v[1]) ^ (v[0] & v[2]) ^ (v[1] & v[2]));
            v[7] = v[6]; v[6] = v[5]; v[5] = v[4]; v[4] = v[3] + t1;
            v[3] = v[2]; v[2] = v[1]; v[1] = v[0]; v[0] = t1 + t2;
        }
        for (i = 0; i < 8; i++) h[HR_ST + i] = v[i] + h[base + i];
    } else {
        uint64_t v[8], t1, t2;
        const uint32_t *k = h + HR_SHA512_K;
        for (i = 0; i < 8; i++) v[i] = hr64(h + base, i);
        for (i = 0; i < 80; i++) {
            uint64_t a1 = hr64(m, i + 1), a14 = hr64(m, i + 14);
            hw64(m, i + 16, hr64(m, i) + (ROR64(a1, 1) ^ ROR64(a1, 8) ^ (a1 >> 7)) + hr64(m, i + 9) +
                            (ROR64(a14, 19) ^ ROR64(a14, 61) ^ (a14 >> 6)));
            t1 = v[7] + (ROR64(v[4], 14) ^ ROR64(v[4], 18) ^ ROR64(v[4], 41)) +
                 ((v[4] & v[5]) | (~v[4] & v[6])) + hr64(k, i) + hr64(m, i);
            t2 = (ROR64(v[0], 28) ^ ROR64(v[0], 34) ^ ROR64(v[0], 39)) +
                 ((v[0] & v[1]) ^ (v[0] & v[2]) ^ (v[1] & v[2]));
            v[7] = v[6]; v[6] = v[5]; v[5] = v[4]; v[4] = v[3] + t1;
            v[3] = v[2]; v[2] = v[1]; v[1] = v[0]; v[0] = t1 + t2;
        }
        for (i = 0; i < 8; i++) hw64(h + HR_ST, i, v[i] + hr64(h + base, i));
    }
    r->fr |= 0x02;                                  /* hash_done */
}

/* LD_MSG and LD_LKEY read MSGSIZE words at the segment pointer plus
 * segptrdyna, which is the last pointer read plus one (combohasha.sv 565
 * to 570, 611, 629) */
static void hash_ld_blk(bao_t *b, hrun_t *r, int seg, uint32_t *dyna)
{
    uint32_t last = hseg_load(b, r, seg, seg, r->ptr[seg] + *dyna, HR_MSG, r->msgsize);
    *dyna = last == hseg_size[seg] - 1u ? 0 : last + 1;
}

/* LD_KEY: a whole block from KEY (SKEY with ifskey) at the KEY pointer and
 * optltx bit 1, each word XORed with the HMAC pad on its way into HRAM
 * (combohasha.sv 643 to 651, 881 to 886) */
static void hash_ld_key(bao_t *b, hrun_t *r, uint32_t pad)
{
    uint32_t i;
    hseg_load(b, r, (r->opt2 >> 6) & 1 ? HS_SKEY : HS_KEY, HS_KEY, r->ptr[HS_KEY], HR_MSG, r->msgsize);
    for (i = 0; i < r->msgsize; i++)
        b->hash_ram[HR_MSG + i] ^= pad;
}

static void hash_run(bao_t *b)
{
    hrun_t r;
    uint32_t i, hc = 0, dyna = 0;
    int ifstart, ifsob, chk, tsmode, schnr;

    memset(&r, 0, sizeof(r));
    r.fn = rd(b, HASH + 0x00) & 0xFF;
    r.cnt = rd(b, HASH + 0x10) & 0xFFFF;
    r.opt2 = rd(b, HASH + 0x14);
    r.ltx = rd(b, HASH + 0x18);
    for (i = 0; i < 8; i++)
        r.ptr[i] = rd(b, HASH + 0x20 + 4 * i) & 0xFFF;
    /* OPT2 is {ifskey, tsmode, schnr[1:0], ifstart, ifsob, scrtchk}
     * (combohasha.sv 218); ifsob is ignored with ifskey (230) */
    ifstart = (int)((r.opt2 >> 2) & 1);
    ifsob = (int)((r.opt2 >> 1) & 1) && !((r.opt2 >> 6) & 1);
    chk = (int)(r.opt2 & 1);
    tsmode = (int)((r.opt2 >> 5) & 1);
    schnr = (int)((r.opt2 >> 3) & 3);
    r.is512 = r.fn == 0x01 || r.fn == 0x41 || r.fn == 0x51 || r.fn == 0x61;
    r.msgsize = r.is512 ? 32 : 16;
    r.stsize = r.is512 ? 16 : 8;

    switch (r.fn) {
    case 0xFF:                      /* HF_INIT: LD_INIT, DONE (257 to 268, 580 to 588) */
        r.fr |= 0x08;
        for (i = 0; i < HR_INITSIZE; i++) {
            uint32_t v = rd(b, SCE_BASE + 4 * i);
            b->hash_ram[i] = (r.ltx & 1) ? bswap32(v) : v;
        }
        break;

    case 0x00: case 0x01:           /* SHA-256, SHA-512 (269 to 310) */
        if (!ifstart)
            hseg_load(b, &r, HS_HOUT, HS_HOUT, r.ptr[HS_HOUT], HR_ST, r.stsize);
        for (;;) {
            hash_ld_blk(b, &r, HS_MSG, &dyna);
            hash_core(b, &r, ifstart && hc == 0);
            if (hc == r.cnt) break;
            hc++;
        }
        hseg_store(b, &r, HS_HOUT, schnr & 1);
        if (ifsob)
            hseg_store(b, &r, HS_SOB, schnr & 2);
        break;

    case 0x40: case 0x41:           /* HMAC key hash (335 to 364) */
        if (!ifstart)
            hseg_load(b, &r, HS_KEY, HS_KEY, r.ptr[HS_KEY], HR_ST, r.stsize);
        for (;;) {
            hash_ld_blk(b, &r, HS_LKEY, &dyna);
            hash_core(b, &r, ifstart && hc == 0);
            if (hc == r.cnt) break;
            hc++;
        }
        hseg_store(b, &r, HS_KEY, 0);
        break;

    case 0x50: case 0x51: {         /* HMAC pass 1 (365 to 401) */
        /* pass1cnt is 16 bits, so a count of 0xFFFF with ifstart hashes
         * the key block alone (248) */
        uint32_t last = ifstart ? (r.cnt + 1) & 0xFFFF : r.cnt;
        if (ifstart) {
            hash_ld_key(b, &r, 0x36363636u);
        } else {
            hseg_load(b, &r, HS_HOUT, HS_HOUT, r.ptr[HS_HOUT], HR_ST, r.stsize);
            hash_ld_blk(b, &r, HS_MSG, &dyna);
        }
        for (;;) {
            hash_core(b, &r, ifstart && hc == 0);
            if (hc == last) break;
            hc = (hc + 1) & 0xFFFF;
            hash_ld_blk(b, &r, HS_MSG, &dyna);
        }
        hseg_store(b, &r, HS_HOUT, schnr & 1);
        break;
    }

    case 0x60: case 0x61:           /* HMAC pass 2 (402 to 442) */
        hash_ld_key(b, &r, 0x5C5C5C5Cu);
        hash_core(b, &r, ifstart);
        /* LD_PASS1: the inner hash from HOUT, then 0x80000000, zeros and
         * the bit length of one block plus one digest in the last word
         * (634 to 642, 874 to 879) */
        hseg_load(b, &r, HS_HOUT, HS_HOUT, r.ptr[HS_HOUT], HR_MSG, r.msgsize);
        for (i = r.stsize; i < r.msgsize; i++)
            b->hash_ram[HR_MSG + i] = i == r.stsize ? 0x80000000u :
                                      i == r.msgsize - 1 ? (r.fn == 0x60 ? 0x300u : 0x600u) : 0;
        hash_core(b, &r, 0);
        if (chk) {
            /* LD_SECRET XORs SCRT into ST and passes when every word
             * written is 0; there is no write back (452 to 490, 684) */
            int pass = 1;
            uint32_t p;
            if (tsmode) {
                /* the key store load goes through the SDMA SCH channel */
                if (bao_first_time(b, HASH + 0x14))
                    bao_logf(b, BAO_LOG_WARN, "hash secret check from the key store is not modelled, the engine stays busy");
                return;
            }
            p = hseg_ptr(b, HS_SCRT, r.ptr[HS_SCRT]);
            for (i = 0; i < r.stsize; i++) {
                uint32_t v = rd(b, SCE_BASE + 4u * (hseg_addr[HS_SCRT] + p));
                if ((r.ltx >> HS_SCRT) & 1) v = bswap32(v);
                b->hash_ram[HR_ST + i] ^= v;
                if (b->hash_ram[HR_ST + i]) pass = 0;
                p = p == hseg_size[HS_SCRT] - 1u ? 0 : p + 1;
            }
            r.fr |= 0x08;
            r.fr |= 0x10 | (pass ? 0x20u : 0x40u);
            break;
        }
        hseg_store(b, &r, HS_HOUT, schnr & 1);
        if (ifsob)
            hseg_store(b, &r, HS_SOB, schnr & 2);
        break;

    case 0x02: case 0x03: case 0x04: case 0x05: case 0x06:
        bao_logf(b, BAO_LOG_WARN, "hash function 0x%02X is not modelled", r.fn);
        return;

    default:
        bao_logf(b, BAO_LOG_WARN, "hash function 0x%02X does not exist, the engine stays idle", r.fn);
        return;
    }
    sce_done(b, 3, r.fr | 0x01);                    /* mfsm_done */
}

static uint32_t rng_next(bao_t *b)
{
    b->rng_state ^= b->rng_state << 13;
    b->rng_state ^= b->rng_state >> 7;
    b->rng_state ^= b->rng_state << 17;
    return (uint32_t)(b->rng_state >> 16);
}

/* register blocks built from the apb_sfr.sv primitives */
enum { CR, SR, SRZ, SRX, FR, AR };

typedef struct {
    uint16_t off;
    uint8_t  kind, count;
    uint32_t mask, iv;
} sfr_t;

typedef struct {
    uint32_t base;
    const char *name;
    int clk;              /* SUBEN bit of its gated clock, or -1 */
    const sfr_t *r;
    int n;
} blk_t;

#define M(w) ((w) >= 32 ? 0xFFFFFFFFu : ((1u << (w)) - 1u))

static const sfr_t glb_r[] = {          /* sce_glbsfra.sv 75 to 117 */
    {0x00, CR, 1, M(2), 0}, {0x04, CR, 1, M(5), 0x1F}, {0x08, CR, 1, M(5), 0},
    {0x10, SRZ, 1, M(9), 0}, {0x14, FR, 1, M(8), 0}, {0x18, FR, 1, M(8), 0},
    {0x1C, AR, 1, 0, 0}, {0x20, CR, 1, M(8), 0}, {0x24, SRZ, 1, M(32), 0},
    {0x30, CR, 1, M(6), 0}, {0x34, AR, 1, 0, 0}, {0x40, SR, 6, M(16), 0},
    {0x60, FR, 1, M(8), 0}, {0xE0, SR, 4, M(32), 0},
};
static const sfr_t sdma_r[] = {         /* scedma.sv 94 to 119 */
    {0x00, AR, 1, 0, 0},
    {0x10, CR, 1, M(1), 0}, {0x14, CR, 1, M(10), 0}, {0x18, CR, 1, M(32), 0},
    {0x1C, CR, 1, M(8), 0}, {0x20, CR, 1, M(12), 0}, {0x24, CR, 1, M(30), 0},
    {0x30, CR, 1, M(1), 0}, {0x34, CR, 1, M(10), 0}, {0x38, CR, 1, M(32), 0},
    {0x3C, CR, 1, M(8), 0}, {0x40, CR, 1, M(12), 0}, {0x44, CR, 1, M(30), 0},
    {0x50, CR, 1, M(4), 0}, {0x54, CR, 1, M(16), 0}, {0x58, CR, 1, M(12), 0},
    {0x5C, CR, 1, M(12), 0}, {0x60, CR, 1, M(12), 0},
    {0x70, CR, 1, M(2), 0}, {0x74, CR, 1, M(32), 0},
};
static const sfr_t hash_r[] = {         /* combohasha.sv 212 to 223 */
    {0x00, CR, 1, M(8), 0}, {0x04, AR, 1, 0, 0}, {0x08, SR, 1, M(8), 0},
    {0x0C, FR, 1, M(7), 0}, {0x10, CR, 1, M(16), 0}, {0x14, CR, 1, M(7), 0},
    {0x18, CR, 1, M(8), 0}, {0x1C, CR, 1, M(8), 0x40}, {0x20, CR, 8, M(12), 0},
    {0x60, CR, 1, M(10), 0},
};
static const sfr_t pke_r[] = {          /* pke.sv 298 to 313 */
    {0x00, CR, 1, M(16), 0}, {0x04, AR, 1, 0, 0}, {0x08, SR, 1, M(9), 0},
    {0x0C, FR, 1, M(5), 0}, {0x10, CR, 1, M(14), 0}, {0x14, CR, 1, M(14), 0},
    {0x18, CR, 1, M(10), 0}, {0x1C, CR, 1, M(5), 0}, {0x20, CR, 1, M(16), 0},
    {0x24, CR, 1, M(9), 0}, {0x30, CR, 5, M(12), 0}, {0x50, CR, 1, M(8), 0},
    {0x54, SRZ, 1, M(32), 0}, {0x60, CR, 1, M(32), 0}, {0x64, AR, 1, 0, 0},
};
static const sfr_t aes_r[] = {          /* aes.sv 144 to 157 */
    {0x00, CR, 1, M(8), 0}, {0x04, AR, 1, 0, 0}, {0x08, SR, 1, M(8), 0},
    {0x0C, FR, 1, M(4), 0}, {0x10, CR, 1, M(9), 0}, {0x14, CR, 1, M(16), 0},
    {0x18, CR, 1, M(6), 0}, {0x20, CR, 1, M(32), 0}, {0x24, AR, 1, 0, 0},
    {0x30, CR, 4, M(12), 0},
};
static const sfr_t trng_r[] = {         /* trng.sv 108 to 120 */
    {0x00, CR, 1, M(13), 0}, {0x04, CR, 1, M(16), 0}, {0x08, CR, 1, M(17), 0},
    {0x0C, CR, 1, M(17), 0}, {0x10, SR, 1, M(32), 0}, {0x14, AR, 1, 0, 0},
    {0x18, FR, 1, M(3), 0}, {0x40, CR, 4, M(32), 0},
};
static const sfr_t alu_r[] = {          /* alu.sv 140 to 150 */
    {0x00, CR, 1, M(8), 0}, {0x04, AR, 1, 0, 0}, {0x08, SR, 1, M(9), 0},
    {0x0C, FR, 1, M(6), 0}, {0x10, CR, 1, M(16), 0}, {0x14, SR, 1, M(16), 0},
    {0x18, CR, 1, M(32), 0}, {0x1C, CR, 1, M(8), 0}, {0x30, CR, 4, M(20), 0},
};

/* SUBEN bits from the clksub index each engine takes (sce.sv 333 to 553) */
static const blk_t blks[] = {
    {0x40028000u, "SCE", -1, glb_r, (int)(sizeof(glb_r) / sizeof(glb_r[0]))},
    {0x40029000u, "SDMA", 0, sdma_r, (int)(sizeof(sdma_r) / sizeof(sdma_r[0]))},
    {0x4002B000u, "HASH", 3, hash_r, (int)(sizeof(hash_r) / sizeof(hash_r[0]))},
    {0x4002C000u, "PKE", 2, pke_r, (int)(sizeof(pke_r) / sizeof(pke_r[0]))},
    {0x4002D000u, "AES", 1, aes_r, (int)(sizeof(aes_r) / sizeof(aes_r[0]))},
    {0x4002E000u, "TRNG", -1, trng_r, (int)(sizeof(trng_r) / sizeof(trng_r[0]))},
    {0x4002F000u, "ALU", 4, alu_r, (int)(sizeof(alu_r) / sizeof(alu_r[0]))},
};
#define NBLK ((int)(sizeof(blks) / sizeof(blks[0])))

static const blk_t *find_blk(uint32_t a)
{
    int i;
    for (i = 0; i < NBLK; i++)
        if ((a & ~0xFFFu) == blks[i].base)
            return &blks[i];
    return NULL;
}

static const sfr_t *find_sfr(const blk_t *k, uint32_t a)
{
    uint32_t off = a & 0xFFF;
    int i;
    for (i = 0; i < k->n; i++)
        if (off >= k->r[i].off && off < k->r[i].off + 4u * k->r[i].count)
            return &k->r[i];
    return NULL;
}

static void once(bao_t *b, uint32_t a, const char *fmt, const char *what)
{
    if (bao_first_time(b, a))
        bao_logf(b, BAO_LOG_WARN, fmt, what, a, bao_where(b, b->pc));
}

/* reset values; the glbsfr FIFO registers sit on resetn0 and survive the
 * 0x5A reset, and the TRNG is not on sceresetn (sce.sv 224, 533) */
static void reset_blocks(bao_t *b, int soft)
{
    int i, j, c;
    for (i = 0; i < NBLK; i++) {
        if (soft && blks[i].base == TRNG)
            continue;
        for (j = 0; j < blks[i].n; j++) {
            const sfr_t *s = &blks[i].r[j];
            if (soft && blks[i].base == CTRL && (s->off == 0x30 || s->off == 0x40))
                continue;
            for (c = 0; c < s->count; c++)
                wr(b, blks[i].base + s->off + 4u * (uint32_t)c, s->iv);
        }
    }
    /* HRAM is RAM and keeps its words through the 0x5A reset; at power up
     * the emulator starts it at 0 */
    if (!soft)
        memset(b->hash_ram, 0, sizeof(b->hash_ram));
    memset(b->aes_rk, 0, sizeof(b->aes_rk));
    memset(b->aes_aram, 0, sizeof(b->aes_aram));
    b->aes_nr = 0;
}

static void ram_clear(bao_t *b)
{
    uint32_t i;
    for (i = 0; i < 0x2800; i += 4) wr(b, SCE_BASE + i, 0);
    for (i = 0; i < 0x2000; i += 4) wr(b, SCE_BASE + 0x4000 + i, 0);
    /* the hash RAM clears from INITSIZE to its last word (cryptoram.sv 25,
     * 62; combohasha.sv 857), so the INIT constants stay */
    for (i = HR_INITSIZE; i < 768; i++) b->hash_ram[i] = 0;
}

/* sceramclr also fires once after reset (sce_sec.sv initregs) */
void sce_reset(bao_t *b)
{
    reset_blocks(b, 0);
    ram_clear(b);
    /* boot1 leaves the TRNG in raw sampling */
    wr(b, TRNG + 0x00, 0x1Fu);
    wr(b, TRNG + 0x04, 0xFFFFu);
    wr(b, TRNG + 0x08, 0x2u);
    wr(b, TRNG + 0x0C, 0x10000u);
    wr(b, TRNG + 0x40, 0xFFFFFFFFu);
    wr(b, TRNG + 0x44, 0xFFFFFFFFu);
    wr(b, TRNG + 0x48, 0xFFFFFFFFu);
    wr(b, TRNG + 0x4C, 0xFFFFFFFFu);
    b->rng_count = 0;
    b->rng_on = 0;
    b->rng_done_blocks = b->rng_clr_blocks = 0;
}

/* FR bit of the engine and its bit in the global FRDONE and interrupt
 * vector (sce.sv 128, 248) */
void sce_done(bao_t *b, int engine, uint32_t fr_bits)
{
    static const char *const ev[6] = {"trng_done", "aes_done", "pke_done", "hash_done",
                                      "alu_done", "sdma_ichdone"};
    static const uint32_t fr[6] = {TRNG + 0x18, AES + 0x0C, 0x4002C00Cu, HASH + 0x0C,
                                   0x4002F00Cu, 0};
    if (fr[engine])
        wr(b, fr[engine], rd(b, fr[engine]) | fr_bits);
    wr(b, CTRL + 0x14, rd(b, CTRL + 0x14) | (1u << engine));
    irq_raise(b, ev[engine]);
}

/* TRNG output (trng.sv 266 to 380): 128 bit blocks into RNGA or RNGB at the
 * board's measured rate. The words come from a seeded generator. */
#define RNG_BLOCK_CYCLES 2734u

static uint32_t rng_blocks(bao_t *b)
{
    uint64_t n;
    if (!b->rng_on)
        return b->rng_done_blocks;
    n = (b->cycles - b->rng_t0) / RNG_BLOCK_CYCLES;
    if (n >= b->rng_target) {
        n = b->rng_target;
        b->rng_on = 0;
    }
    b->rng_done_blocks = (uint32_t)n;
    return (uint32_t)n;
}

/* the next block boundary, so polling loops see the count move */
uint64_t sce_next_event(bao_t *b)
{
    if (!b->rng_on)
        return NEVER;
    return b->rng_t0 + ((uint64_t)rng_blocks(b) + 1u) * RNG_BLOCK_CYCLES;
}

static uint32_t rng_fifo_words(bao_t *b, int fifo)
{
    uint32_t n;
    if (fifo != 4 + b->rng_seg || !((rd(b, CTRL + 0x30) >> fifo) & 1))
        return 0;
    n = (rng_blocks(b) - b->rng_clr_blocks) * 4u;
    return n > 256u ? 256u : n;
}

static void trng_fill(bao_t *b)
{
    uint32_t i, opt = rd(b, TRNG + 0x0C);
    uint32_t base = (opt >> 16) & 1 ? 0x2300u : 0x1F00u;
    b->rng_seg = (int)((opt >> 16) & 1);
    b->rng_target = (opt & 0xFFFFu) ? (opt & 0xFFFFu) : 0x10000u;
    b->rng_t0 = b->cycles;
    b->rng_on = 1;
    b->rng_done_blocks = b->rng_clr_blocks = 0;
    for (i = 0; i < 256; i++)
        wr(b, SCE_BASE + base + 4 * i, rng_next(b));
    b->rng_count = 0;
    sce_done(b, 0, 1);
    bao_kick(b);
}

static void action(bao_t *b, uint32_t a, uint32_t v)
{
    switch (a) {
    case CTRL + 0x1C:
        if (v == 0x5A) reset_blocks(b, 1);
        else if (v == 0xA5) ram_clear(b);
        break;
    case CTRL + 0x34:
        if (v == 0xFF05 || v == 0xFF04) {
            b->rng_count = 0;
            if ((int)(v & 0xFF) == 4 + b->rng_seg)
                b->rng_clr_blocks = rng_blocks(b);
        }
        break;
    case 0x40029000u:
        if (v == 0x5A) sdma_ich_start(b);
        else if (v == 0xA5 || v == 0xAA)
            once(b, a, "%s channel start 0x%08X at %s is not modelled",
                 v == 0xA5 ? "SDMA XCH" : "SDMA SCH");
        break;
    case HASH + 0x04: if (v == 0x5A) hash_run(b); break;
    case AES + 0x04: if (v == 0x5A) aes_run(b); break;
    case 0x4002C004u:
        if (v == 0x5A) pke_start(b);
        else if (v == 0xFF) once(b, a, "%s RAM clear 0x%08X at %s is not modelled", "PKE");
        break;
    case 0x4002F004u: if (v == 0x5A) alu_start(b); break;
    case TRNG + 0x14:
        if (v == 0x5A) trng_fill(b);
        else if (v == 0xA5) { rng_blocks(b); b->rng_on = 0; }
        else if (v == 0x55) once(b, a, "%s generate 0x%08X at %s is not modelled", "TRNG");
        break;
    default:
        break;
    }
}

int sce_read(bao_t *b, uint32_t a, uint32_t *v)
{
    const blk_t *k;
    const sfr_t *s;

    if (a < CTRL)
        return -1;
    *v = 0;
    k = find_blk(a);
    if (!k || !(s = find_sfr(k, a))) {
        once(b, a, "%s read 0x%08X at %s has no register there and reads 0", k ? k->name : "SCE");
        return 0;
    }
    switch (s->kind) {
    case AR: case SRZ:
        break;
    case SRX:
        once(b, a, "%s status 0x%08X at %s is not modelled and reads 0", k->name);
        break;
    default:
        *v = rd(b, a) & s->mask;
    }
    if (a == CTRL + 0x50 || a == CTRL + 0x54)
        *v = ((rng_fifo_words(b, a == CTRL + 0x50 ? 4 : 5) << 4) | 2u) & s->mask;
    else if (a == TRNG + 0x10)
        *v = (1u << 28) | (rng_blocks(b) & 0xFFFFu);
    else if (a == CTRL + 0xE0)
        *v = 0x80000000u;
    return 0;
}

int sce_write(bao_t *b, uint32_t a, uint32_t v)
{
    const blk_t *k;
    const sfr_t *s;

    if (a < CTRL)
        return -1;
    k = find_blk(a);
    if (!k || !(s = find_sfr(k, a))) {
        once(b, a, "%s write 0x%08X at %s has no register there and is dropped", k ? k->name : "SCE");
        return 0;
    }
    if (k->clk >= 0 && !((rd(b, CTRL + 0x04) >> k->clk) & 1)) {
        once(b, a, "%s write 0x%08X at %s is lost, its clock is off in SUBEN", k->name);
        return 0;
    }
    switch (s->kind) {
    case CR:
        /* SCEMODE locks once it is not zero, outside devmode (sce_glbsfra.sv 75) */
        if (a == CTRL && (rd(b, a) & 3))
            break;
        wr(b, a, v & s->mask);
        break;
    case FR:
        wr(b, a, rd(b, a) & ~v);
        break;
    case AR:
        action(b, a, v);
        break;
    default:
        break;
    }
    return 0;
}
