/*
 * pke.c: SCE public key engine, big number ALU and internal DMA channel
 *
 * Copyright (c) 2026 Armstrong Subero
 * SPDX-License-Identifier: Apache-2.0
 *
 * C port of tools/emu/sce_model.py from the Dabao SDK. Results are computed
 * directly.
 */

#include <stdlib.h>
#include <string.h>
#include "bao_int.h"

#define SCE_BASE 0x40020000u
#define SDMA     0x40029000u
#define PKE      0x4002C000u
#define ALU      0x4002F000u
#define PCON     (SCE_BASE + 0x0800u)
#define PKB      (SCE_BASE + 0x0800u)
#define PIB      (SCE_BASE + 0x0C00u)
#define POB      (SCE_BASE + 0x1400u)

#define BN_MAX 420

typedef struct {
    int n;
    uint32_t d[BN_MAX];
} bn_t;

uint32_t *bao_mmio_slot(bao_t *b, uint32_t addr);

static uint32_t rd(bao_t *b, uint32_t a) { uint32_t *s = bao_mmio_slot(b, a); return s ? *s : 0; }
static void wr(bao_t *b, uint32_t a, uint32_t v) { uint32_t *s = bao_mmio_slot(b, a); if (s) *s = v; }

static void bn_norm(bn_t *a) { while (a->n > 0 && a->d[a->n - 1] == 0) a->n--; }
static void bn_zero(bn_t *a) { a->n = 0; }
static void bn_u32(bn_t *a, uint32_t v) { a->d[0] = v; a->n = 1; bn_norm(a); }
static void bn_copy(bn_t *r, const bn_t *a) { r->n = a->n; memcpy(r->d, a->d, sizeof(uint32_t) * (size_t)a->n); }
static int bn_is_zero(const bn_t *a) { return a->n == 0; }

static int bn_cmp(const bn_t *a, const bn_t *c)
{
    int i;
    if (a->n != c->n) return a->n < c->n ? -1 : 1;
    for (i = a->n - 1; i >= 0; i--)
        if (a->d[i] != c->d[i]) return a->d[i] < c->d[i] ? -1 : 1;
    return 0;
}

static int bn_bits(const bn_t *a)
{
    uint32_t t;
    int n = 0;
    if (!a->n) return 0;
    t = a->d[a->n - 1];
    while (t) { n++; t >>= 1; }
    return 32 * (a->n - 1) + n;
}

static int bn_bit(const bn_t *a, int i)
{
    if (i / 32 >= a->n) return 0;
    return (a->d[i / 32] >> (i % 32)) & 1;
}

static void bn_add(bn_t *r, const bn_t *a, const bn_t *c)
{
    uint64_t cy = 0;
    int i, n = a->n > c->n ? a->n : c->n;
    for (i = 0; i < n; i++) {
        cy += (uint64_t)(i < a->n ? a->d[i] : 0) + (i < c->n ? c->d[i] : 0);
        r->d[i] = (uint32_t)cy;
        cy >>= 32;
    }
    if (cy && n < BN_MAX) r->d[n++] = (uint32_t)cy;
    r->n = n;
    bn_norm(r);
}

static void bn_sub(bn_t *r, const bn_t *a, const bn_t *c)
{
    int64_t br = 0;
    int i;
    for (i = 0; i < a->n; i++) {
        int64_t t = (int64_t)a->d[i] - (i < c->n ? c->d[i] : 0) - br;
        br = t < 0;
        r->d[i] = (uint32_t)(t + (br ? ((int64_t)1 << 32) : 0));
    }
    r->n = a->n;
    bn_norm(r);
}

static void bn_mul(bn_t *r, const bn_t *a, const bn_t *c)
{
    bn_t t;
    int i, j;
    if (!a->n || !c->n) { bn_zero(r); return; }
    t.n = a->n + c->n;
    if (t.n > BN_MAX) t.n = BN_MAX;
    memset(t.d, 0, sizeof(uint32_t) * (size_t)t.n);
    for (i = 0; i < a->n; i++) {
        uint64_t cy = 0;
        for (j = 0; j < c->n && i + j < BN_MAX; j++) {
            cy += (uint64_t)a->d[i] * c->d[j] + t.d[i + j];
            t.d[i + j] = (uint32_t)cy;
            cy >>= 32;
        }
        if (i + j < BN_MAX) t.d[i + j] = (uint32_t)cy;
    }
    bn_norm(&t);
    bn_copy(r, &t);
}

static void bn_shr(bn_t *r, const bn_t *a, int s)
{
    int w = s / 32, k = s % 32, i;
    if (w >= a->n) { bn_zero(r); return; }
    for (i = 0; i < a->n - w; i++) {
        uint32_t lo = a->d[i + w] >> k;
        uint32_t hi = (k && i + w + 1 < a->n) ? a->d[i + w + 1] << (32 - k) : 0;
        r->d[i] = lo | hi;
    }
    r->n = a->n - w;
    bn_norm(r);
}

/* q = a / m, r = a mod m, either may be NULL (Knuth algorithm D) */
static void bn_divmod(bn_t *q, bn_t *r, const bn_t *a, const bn_t *m)
{
    static const uint64_t B = 0x100000000ull;
    uint32_t u[BN_MAX + 1], v[BN_MAX], qd[BN_MAX];
    int an = a->n, n = m->n, s, i, j;

    while (an > 0 && a->d[an - 1] == 0) an--;
    while (n > 0 && m->d[n - 1] == 0) n--;
    if (n == 0) { if (q) bn_zero(q); if (r) bn_zero(r); return; }
    if (an < n) {
        bn_t rr;
        rr.n = an;
        memcpy(rr.d, a->d, sizeof(uint32_t) * (size_t)an);
        if (q) bn_zero(q);
        if (r) bn_copy(r, &rr);
        return;
    }
    if (n == 1) {
        uint64_t rem = 0;
        for (i = an - 1; i >= 0; i--) {
            uint64_t cur = (rem << 32) | a->d[i];
            qd[i] = (uint32_t)(cur / m->d[0]);
            rem = cur % m->d[0];
        }
        if (q) { q->n = an; memcpy(q->d, qd, sizeof(uint32_t) * (size_t)an); bn_norm(q); }
        if (r) bn_u32(r, (uint32_t)rem);
        return;
    }

    /* D1: normalise so the divisor's top bit is set */
    s = 0;
    while (!(m->d[n - 1] & (0x80000000u >> s))) s++;
    for (i = n - 1; i > 0; i--)
        v[i] = (m->d[i] << s) | (s ? (uint32_t)((uint64_t)m->d[i - 1] >> (32 - s)) : 0);
    v[0] = m->d[0] << s;
    u[an] = s ? (uint32_t)((uint64_t)a->d[an - 1] >> (32 - s)) : 0;
    for (i = an - 1; i > 0; i--)
        u[i] = (a->d[i] << s) | (s ? (uint32_t)((uint64_t)a->d[i - 1] >> (32 - s)) : 0);
    u[0] = a->d[0] << s;

    for (j = an - n; j >= 0; j--) {
        /* D3: estimate the quotient word */
        uint64_t num = ((uint64_t)u[j + n] << 32) | u[j + n - 1];
        uint64_t qhat = num / v[n - 1], rhat = num % v[n - 1];
        int64_t t, k;
        while (qhat >= B || qhat * v[n - 2] > ((rhat << 32) | u[j + n - 2])) {
            qhat--;
            rhat += v[n - 1];
            if (rhat >= B) break;
        }
        /* D4: multiply and subtract */
        k = 0;
        for (i = 0; i < n; i++) {
            uint64_t p = qhat * v[i];
            t = (int64_t)u[i + j] - k - (int64_t)(p & 0xFFFFFFFFull);
            u[i + j] = (uint32_t)t;
            k = (int64_t)(p >> 32) - (t >> 32);
        }
        t = (int64_t)u[j + n] - k;
        u[j + n] = (uint32_t)t;
        qd[j] = (uint32_t)qhat;
        /* D6: add back when the estimate was one too large */
        if (t < 0) {
            uint64_t c = 0;
            qd[j]--;
            for (i = 0; i < n; i++) {
                uint64_t w = (uint64_t)u[i + j] + v[i] + c;
                u[i + j] = (uint32_t)w;
                c = w >> 32;
            }
            u[j + n] += (uint32_t)c;
        }
    }

    if (q) {
        q->n = an - n + 1;
        memcpy(q->d, qd, sizeof(uint32_t) * (size_t)q->n);
        bn_norm(q);
    }
    if (r) {
        /* D8: the remainder is u shifted back */
        for (i = 0; i < n; i++)
            r->d[i] = (u[i] >> s) | (s ? (uint32_t)((uint64_t)u[i + 1] << (32 - s)) : 0);
        r->n = n;
        bn_norm(r);
    }
}

static void bn_mod(bn_t *r, const bn_t *a, const bn_t *m) { bn_divmod(NULL, r, a, m); }

static void bn_mulmod(bn_t *r, const bn_t *a, const bn_t *c, const bn_t *m)
{
    bn_t t;
    bn_mul(&t, a, c);
    bn_mod(r, &t, m);
}

static void bn_addmod(bn_t *r, const bn_t *a, const bn_t *c, const bn_t *m)
{
    bn_t t;
    bn_add(&t, a, c);
    bn_mod(r, &t, m);
}

static void bn_submod(bn_t *r, const bn_t *a, const bn_t *c, const bn_t *m)
{
    bn_t x, y;
    bn_mod(&x, a, m);
    bn_mod(&y, c, m);
    if (bn_cmp(&x, &y) >= 0) {
        bn_sub(r, &x, &y);
    } else {
        bn_sub(&y, &y, &x);
        bn_sub(r, m, &y);
    }
}

static void bn_powmod(bn_t *r, const bn_t *x, const bn_t *e, const bn_t *m)
{
    bn_t acc, base;
    int i, nb = bn_bits(e);
    bn_u32(&acc, 1);
    bn_mod(&acc, &acc, m);
    bn_mod(&base, x, m);
    for (i = nb - 1; i >= 0; i--) {
        bn_mulmod(&acc, &acc, &acc, m);
        if (bn_bit(e, i)) bn_mulmod(&acc, &acc, &base, m);
    }
    bn_copy(r, &acc);
}

static void bn_gcd(bn_t *r, const bn_t *a, const bn_t *c)
{
    bn_t x, y, t;
    bn_copy(&x, a);
    bn_copy(&y, c);
    while (!bn_is_zero(&y)) {
        bn_mod(&t, &x, &y);
        bn_copy(&x, &y);
        bn_copy(&y, &t);
    }
    bn_copy(r, &x);
}

static int bn_inv(bn_t *r, const bn_t *a, const bn_t *m)
{
    bn_t r0, r1, t0, t1, q, tmp, rem;
    bn_copy(&r0, m);
    bn_mod(&r1, a, m);
    bn_zero(&t0);
    bn_u32(&t1, 1);
    while (!bn_is_zero(&r1)) {
        bn_divmod(&q, &rem, &r0, &r1);
        bn_copy(&r0, &r1);
        bn_copy(&r1, &rem);
        bn_mulmod(&tmp, &q, &t1, m);
        bn_submod(&tmp, &t0, &tmp, m);
        bn_copy(&t0, &t1);
        bn_copy(&t1, &tmp);
    }
    if (!(r0.n == 1 && r0.d[0] == 1)) return -1;
    bn_mod(r, &t0, m);
    return 0;
}

static void bn_mask(bn_t *a, int bits)
{
    int w = bits / 32, k = bits % 32;
    if (w >= a->n) return;
    if (k) { a->d[w] &= (1u << k) - 1; a->n = w + 1; }
    else a->n = w;
    bn_norm(a);
}

static void bn_load(bao_t *b, bn_t *r, uint32_t addr, int n, int swap_pairs, int reverse)
{
    int i;
    if (n > BN_MAX) n = BN_MAX;
    for (i = 0; i < n; i++) {
        int src = i;
        if (reverse) src = n - 1 - i;
        else if (swap_pairs && (i ^ 1) < n) src = i ^ 1;
        r->d[i] = rd(b, addr + 4u * (uint32_t)src);
    }
    r->n = n;
    bn_norm(r);
}

static void bn_store(bao_t *b, uint32_t addr, const bn_t *a, int n, int reverse)
{
    int i;
    for (i = 0; i < n; i++) {
        uint32_t v = i < a->n ? a->d[i] : 0;
        wr(b, addr + 4u * (uint32_t)(reverse ? n - 1 - i : i), v);
    }
}

typedef struct {
    bn_t n, inv_p, p, c;
    int has_n, has_inv, curve, cw;
} pke_t;

enum { CURVE_NONE, CURVE_W, CURVE_ED, CURVE_X };

static pke_t *pk(bao_t *b)
{
    if (!b->pke) {
        b->pke = calloc(1, sizeof(pke_t));
    }
    return b->pke;
}

static int is_p25519(const bn_t *p)
{
    int i;
    if (p->n != 8 || p->d[0] != 0xFFFFFFEDu || p->d[7] != 0x7FFFFFFFu) return 0;
    for (i = 1; i < 7; i++) if (p->d[i] != 0xFFFFFFFFu) return 0;
    return 1;
}

static void pib(bao_t *b, bn_t *r, uint32_t off, int n) { bn_load(b, r, PIB + 4 * off, n, 0, 0); }
static void put_pob(bao_t *b, uint32_t off, const bn_t *v, int n) { bn_store(b, POB + 4 * off, v, n, 0); }

typedef struct { bn_t x, y; int inf; } pt_t;

static void w_add(pke_t *k, pt_t *r, const pt_t *P, const pt_t *Q)
{
    bn_t l, t, u, x, y;
    const bn_t *p = &k->p;
    if (P->inf) { *r = *Q; return; }
    if (Q->inf) { *r = *P; return; }
    bn_addmod(&t, &P->y, &Q->y, p);
    if (bn_cmp(&P->x, &Q->x) == 0 && bn_is_zero(&t)) { r->inf = 1; return; }
    if (bn_cmp(&P->x, &Q->x) == 0 && bn_cmp(&P->y, &Q->y) == 0) {
        bn_mulmod(&t, &P->x, &P->x, p);
        bn_u32(&u, 3);
        bn_mulmod(&t, &t, &u, p);
        bn_addmod(&t, &t, &k->c, p);
        bn_addmod(&u, &P->y, &P->y, p);
    } else {
        bn_submod(&t, &Q->y, &P->y, p);
        bn_submod(&u, &Q->x, &P->x, p);
    }
    if (bn_inv(&u, &u, p)) { r->inf = 1; return; }
    bn_mulmod(&l, &t, &u, p);
    bn_mulmod(&x, &l, &l, p);
    bn_submod(&x, &x, &P->x, p);
    bn_submod(&x, &x, &Q->x, p);
    bn_submod(&t, &P->x, &x, p);
    bn_mulmod(&y, &l, &t, p);
    bn_submod(&y, &y, &P->y, p);
    bn_copy(&r->x, &x);
    bn_copy(&r->y, &y);
    r->inf = 0;
}

static void ed_add(pke_t *k, pt_t *r, const pt_t *P, const pt_t *Q)
{
    bn_t t, num, den, one, x, y;
    const bn_t *p = &k->p;
    bn_u32(&one, 1);
    bn_mulmod(&t, &k->c, &P->x, p);
    bn_mulmod(&t, &t, &Q->x, p);
    bn_mulmod(&t, &t, &P->y, p);
    bn_mulmod(&t, &t, &Q->y, p);
    bn_mulmod(&num, &P->x, &Q->y, p);
    bn_mulmod(&den, &Q->x, &P->y, p);
    bn_addmod(&num, &num, &den, p);
    bn_addmod(&den, &one, &t, p);
    bn_inv(&den, &den, p);
    bn_mulmod(&x, &num, &den, p);
    bn_mulmod(&num, &P->y, &Q->y, p);
    bn_mulmod(&den, &P->x, &Q->x, p);
    bn_addmod(&num, &num, &den, p);
    bn_submod(&den, &one, &t, p);
    bn_inv(&den, &den, p);
    bn_mulmod(&y, &num, &den, p);
    bn_copy(&r->x, &x);
    bn_copy(&r->y, &y);
    r->inf = 0;
}

static void pt_mul(pke_t *k, pt_t *r, const bn_t *s, const pt_t *P, int ed)
{
    pt_t R, Q, T;
    int i, nb = bn_bits(s);
    Q = *P;
    if (ed) { bn_zero(&R.x); bn_u32(&R.y, 1); R.inf = 0; }
    else R.inf = 1;
    for (i = 0; i < nb; i++) {
        if (bn_bit(s, i)) {
            if (ed) ed_add(k, &T, &R, &Q); else w_add(k, &T, &R, &Q);
            R = T;
        }
        if (ed) ed_add(k, &T, &Q, &Q); else w_add(k, &T, &Q, &Q);
        Q = T;
    }
    *r = R;
}

static void w_read(bao_t *b, pke_t *k, pt_t *P, uint32_t off)
{
    pib(b, &P->x, off, k->cw);
    pib(b, &P->y, off + (uint32_t)k->cw, k->cw);
    P->inf = 0;
}

static void w_write(bao_t *b, pke_t *k, const pt_t *P)
{
    bn_t one;
    bn_u32(&one, 1);
    if (P->inf) { bn_t z; bn_zero(&z); put_pob(b, 0, &z, k->cw); put_pob(b, (uint32_t)k->cw, &z, k->cw); put_pob(b, 2u * (uint32_t)k->cw, &z, k->cw); return; }
    put_pob(b, 0, &P->x, k->cw);
    put_pob(b, (uint32_t)k->cw, &P->y, k->cw);
    put_pob(b, 2u * (uint32_t)k->cw, &one, k->cw);
}

static void ed_read(bao_t *b, pt_t *P, uint32_t off)
{
    pib(b, &P->x, off, 8);
    pib(b, &P->y, off + 8, 8);
    P->inf = 0;
}

static void ed_write(bao_t *b, pke_t *k, const pt_t *P)
{
    bn_t one, xy;
    bn_u32(&one, 1);
    bn_mulmod(&xy, &P->x, &P->y, &k->p);
    put_pob(b, 0, &P->x, 8);
    put_pob(b, 8, &P->y, 8);
    put_pob(b, 16, &one, 8);
    put_pob(b, 24, &xy, 8);
}

static int pke_run(bao_t *b)
{
    pke_t *k = pk(b);
    uint32_t fn = rd(b, PKE + 0x00) & 0xFF;
    int nw = (int)(rd(b, PKE + 0x10) & 0x3FFF), ew = (int)(rd(b, PKE + 0x14) & 0x3FFF);
    uint32_t pcon = rd(b, PKE + 0x30), pib0 = rd(b, PKE + 0x34), pib1 = rd(b, PKE + 0x38);
    uint32_t pkb = rd(b, PKE + 0x3C), pob = rd(b, PKE + 0x40);
    bn_t x, y, z, e;
    pt_t P, Q, R;

    if (!k) return -1;
    switch (fn) {
    case 0x11:
        bn_load(b, &k->n, PCON + 4 * pcon, (nw + 31) / 32, 0, 0);
        bn_mask(&k->n, nw);
        k->has_n = 1;
        if (k->n.n && !(k->n.d[0] & 1))
            bao_logf(b, BAO_LOG_WARN, "RSAINIT with an even modulus");
        break;
    case 0x12: case 0x19: case 0x1A: {
        int kw;
        if (fn == 0x12 && nw == 6144 && rd(b, PKE + 0x24) == 0x100) break;
        if (!k->has_n || bn_is_zero(&k->n)) break;
        kw = (bn_bits(&k->n) + 31) / 32;
        pib(b, &x, pib0, kw);
        pib(b, &y, pib1, kw);
        if (fn == 0x12) bn_mulmod(&z, &x, &y, &k->n);
        else if (fn == 0x19) bn_addmod(&z, &x, &y, &k->n);
        else bn_submod(&z, &x, &y, &k->n);
        put_pob(b, pob, &z, kw);
        break;
    }
    case 0x13: {
        int kw;
        if (!k->has_n || bn_is_zero(&k->n)) break;
        kw = (bn_bits(&k->n) + 31) / 32;
        pib(b, &x, pib0, kw);
        bn_load(b, &e, PKB + 4 * pkb, (ew + 31) / 32, 0, 0);
        bn_mask(&e, ew);
        bn_powmod(&z, &x, &e, &k->n);
        put_pob(b, pob, &z, kw);
        break;
    }
    case 0xFE:
        bn_load(b, &k->inv_p, PCON + 4 * pcon, (nw + 31) / 32, 0, 0);
        bn_mask(&k->inv_p, nw);
        k->has_inv = 1;
        break;
    case 0x18: {
        int kw = (nw + 31) / 32;
        if (!k->has_inv) break;
        pib(b, &x, pib0, kw);
        bn_gcd(&z, &x, &k->inv_p);
        if (!(z.n == 1 && z.d[0] == 1)) break;
        bn_inv(&z, &x, &k->inv_p);
        put_pob(b, pob, &z, kw);
        wr(b, PKE + 0x08, rd(b, PKE + 0x08) | 0x100);
        break;
    }
    case 0x51: {
        int d = nw / 32;
        pib(b, &x, pib0, d);
        pib(b, &y, pib1, d);
        bn_gcd(&z, &x, &y);
        put_pob(b, pob, &z, d);
        break;
    }
    case 0x01: {
        int pw = ((nw + 63) / 64) * 2;
        bn_load(b, &k->p, PCON + 4 * pcon, pw, 0, 0);
        bn_load(b, &k->c, PCON + 4 * (pcon + (uint32_t)pw), pw, 0, 0);
        if (bn_bits(&k->p) != nw)
            bao_logf(b, BAO_LOG_WARN, "ECINIT bit length %d does not match p", nw);
        if (is_p25519(&k->p))
            k->curve = (k->c.n == 1 && k->c.d[0] == 121665) ? CURVE_X : CURVE_ED;
        else {
            k->curve = CURVE_W;
            bn_mod(&k->c, &k->c, &k->p);
        }
        k->cw = pw;
        break;
    }
    case 0x04: case 0x24:
        if (k->curve == CURVE_W) {
            w_read(b, k, &P, pib0);
            w_write(b, k, &P);
        } else if (k->curve == CURVE_ED) {
            ed_read(b, &P, pib0);
            ed_write(b, k, &P);
        } else {
            bn_zero(&z);
            pib(b, &x, pib0, 8);
            put_pob(b, 0, &x, 8);
            put_pob(b, 8, &z, 8); put_pob(b, 16, &z, 8); put_pob(b, 24, &z, 8);
        }
        break;
    case 0x06:
        w_read(b, k, &P, pib0);
        w_add(k, &R, &P, &P);
        w_write(b, k, &R);
        break;
    case 0x05:
        w_read(b, k, &P, pib0);
        w_read(b, k, &Q, pib1);
        w_add(k, &R, &P, &Q);
        w_write(b, k, &R);
        break;
    case 0x0B:
        bn_load(b, &e, PKB + 4 * pkb, (ew + 31) / 32, 0, 0);
        bn_mask(&e, ew);
        if (bn_bits(&e) != ew)
            bao_logf(b, BAO_LOG_WARN, "ECPM OPTEW %d does not match k", ew);
        w_read(b, k, &P, pib0);
        w_read(b, k, &Q, pib1);
        w_add(k, &R, &P, &P);
        if (R.inf || bn_cmp(&R.x, &Q.x) || bn_cmp(&R.y, &Q.y))
            bao_logf(b, BAO_LOG_WARN, "ECPM PIB1 does not hold 2P");
        pt_mul(k, &R, &e, &P, 0);
        w_write(b, k, &R);
        break;
    case 0x07:
        w_read(b, k, &P, pib0);
        put_pob(b, 0, &P.x, k->cw);
        put_pob(b, (uint32_t)k->cw, &P.y, k->cw);
        wr(b, PKE + 0x08, rd(b, PKE + 0x08) | 0x100);
        break;
    case 0x26:
        ed_read(b, &P, pib0);
        ed_add(k, &R, &P, &P);
        ed_write(b, k, &R);
        break;
    case 0x25:
        ed_read(b, &P, pib0);
        ed_read(b, &Q, pib1);
        ed_add(k, &R, &P, &Q);
        ed_write(b, k, &R);
        break;
    case 0x2B:
        bn_load(b, &e, PKB + 4 * pkb, 8, 0, 0);
        bn_mask(&e, ew);
        ed_read(b, &P, pib0);
        ed_read(b, &Q, pib1);
        ed_add(k, &R, &P, &P);
        if (bn_cmp(&R.x, &Q.x) || bn_cmp(&R.y, &Q.y))
            bao_logf(b, BAO_LOG_WARN, "EDPM PIB1 does not hold 2P");
        pt_mul(k, &R, &e, &P, 1);
        ed_write(b, k, &R);
        break;
    case 0x27:
        pib(b, &x, pib0, 8);
        pib(b, &y, pib0 + 8, 8);
        put_pob(b, 0, &x, 8);
        if (k->curve == CURVE_ED) put_pob(b, 8, &y, 8);
        wr(b, PKE + 0x08, rd(b, PKE + 0x08) | 0x100);
        break;
    case 0x3B: {
        bn_t x1, x2, z2, x3, z3, A, AA, B, BB, E, C, D, DA, CB, t, a24, p;
        int i, sw = 0;
        static const uint32_t p25519[8] = { 0xFFFFFFEDu, 0xFFFFFFFFu, 0xFFFFFFFFu, 0xFFFFFFFFu,
                                            0xFFFFFFFFu, 0xFFFFFFFFu, 0xFFFFFFFFu, 0x7FFFFFFFu };
        memcpy(p.d, p25519, sizeof(p25519)); p.n = 8;
        bn_load(b, &e, PKB + 4 * pkb, 8, 0, 0);
        bn_mask(&e, 255);
        pib(b, &x1, pib0, 8);
        bn_copy(&x2, &x1); bn_u32(&x2, 1);
        bn_zero(&z2);
        bn_copy(&x3, &x1);
        bn_u32(&z3, 1);
        bn_u32(&a24, 121665);
        for (i = 254; i >= 0; i--) {
            int kt = bn_bit(&e, i);
            sw ^= kt;
            if (sw) { bn_t s; bn_copy(&s, &x2); bn_copy(&x2, &x3); bn_copy(&x3, &s);
                      bn_copy(&s, &z2); bn_copy(&z2, &z3); bn_copy(&z3, &s); }
            sw = kt;
            bn_addmod(&A, &x2, &z2, &p); bn_mulmod(&AA, &A, &A, &p);
            bn_submod(&B, &x2, &z2, &p); bn_mulmod(&BB, &B, &B, &p);
            bn_submod(&E, &AA, &BB, &p);
            bn_addmod(&C, &x3, &z3, &p); bn_submod(&D, &x3, &z3, &p);
            bn_mulmod(&DA, &D, &A, &p); bn_mulmod(&CB, &C, &B, &p);
            bn_addmod(&t, &DA, &CB, &p); bn_mulmod(&x3, &t, &t, &p);
            bn_submod(&t, &DA, &CB, &p); bn_mulmod(&t, &t, &t, &p); bn_mulmod(&z3, &x1, &t, &p);
            bn_mulmod(&x2, &AA, &BB, &p);
            bn_mulmod(&t, &a24, &E, &p); bn_addmod(&t, &AA, &t, &p); bn_mulmod(&z2, &E, &t, &p);
        }
        if (sw) { bn_copy(&x2, &x3); bn_copy(&z2, &z3); }
        if (bn_is_zero(&z2)) bn_zero(&z);
        else { bn_inv(&t, &z2, &p); bn_mulmod(&z, &x2, &t, &p); }
        put_pob(b, 0, &z, 8);
        bn_zero(&t);
        put_pob(b, 8, &t, 8); put_pob(b, 16, &t, 8); put_pob(b, 24, &t, 8);
        break;
    }
    default:
        bao_logf(b, BAO_LOG_WARN, "PKE function 0x%02X is not modelled", fn);
        return -1;
    }
    return 0;
}

static uint32_t seg_addr(bao_t *b, uint32_t reg)
{
    uint32_t v = rd(b, ALU + reg), sid = (v >> 12) & 0xFF, off = v & 0xFFF;
    switch (sid) {
    case 9: return SCE_BASE + 0x0C00u + 4 * off;
    case 11: return SCE_BASE + 0x1400u + 4 * off;
    case 12: return SCE_BASE + 0x1800u + 4 * off;
    default:
        bao_logf(b, BAO_LOG_WARN, "ALU pointer to unknown segment %u", sid);
        return SCE_BASE + 0x0C00u;
    }
}

static void alu_run(bao_t *b)
{
    uint32_t fn = rd(b, ALU + 0x00) & 0xFF, lens = rd(b, ALU + 0x10), opt = rd(b, ALU + 0x18);
    int de_w = (int)(lens & 0xFF) * 2, ds_w = (int)((lens >> 8) & 0xFF) * 2, n;
    uint32_t de = seg_addr(b, 0x30), ds = seg_addr(b, 0x34), qt = seg_addr(b, 0x38), rm = seg_addr(b, 0x3C);
    bn_t x, y, q, r;

    n = de_w > ds_w ? de_w : ds_w;
    if (fn == 0x01 || fn == 0x02 || fn == 0x03) {
        bn_load(b, &x, de, de_w, 1, 0);
        bn_load(b, &y, ds, ds_w, 1, 0);
        if (fn == 0x01) {
            int qw, rw;
            bn_divmod(&q, &r, &x, &y);
            qw = ((bn_bits(&q) + 63) / 64) * 2; if (qw < 2) qw = 2;
            rw = ((bn_bits(&r) + 63) / 64) * 2; if (rw < 2) rw = 2;
            bn_store(b, qt, &q, qw, 0);
            bn_store(b, rm, &r, rw, 0);
            wr(b, ALU + 0x14, (uint32_t)(qw / 2) | ((uint32_t)(rw / 2) << 8));
        } else {
            int flag;
            uint32_t st;
            if (fn == 0x02) {
                bn_add(&q, &x, &y);
                flag = bn_bit(&q, 32 * n);
                bn_mask(&q, 32 * n);
            } else if (bn_cmp(&x, &y) >= 0) {
                bn_sub(&q, &x, &y);
                flag = 0;
            } else {
                bn_t m;
                bn_zero(&m);
                m.n = n + 1;
                memset(m.d, 0, sizeof(uint32_t) * (size_t)(n + 1));
                m.d[n] = 1;
                bn_sub(&q, &y, &x);
                bn_sub(&q, &m, &q);
                flag = 1;
            }
            bn_store(b, qt, &q, n, 0);
            st = rd(b, ALU + 0x08) & ~0x100u;
            wr(b, ALU + 0x08, st | (flag ? 0x100u : 0));
        }
    } else if (fn == 0x10 || fn == 0x11 || fn == 0x20) {
        bn_load(b, &x, de, de_w, 0, 1);
        if (fn == 0x10) {
            bn_shr(&q, &x, (int)((opt >> 8) & 0xFF));
            bn_store(b, qt, &q, de_w, 1);
        } else if (fn == 0x11) {
            bn_shr(&q, &x, 32 * (int)((opt >> 8) & 0xFF));
            bn_store(b, qt, &q, de_w, 1);
        } else {
            int i;
            bn_load(b, &y, ds, ds_w, 0, 1);
            q.n = n;
            memset(q.d, 0, sizeof(uint32_t) * (size_t)n);
            for (i = 0; i < 32 * n; i++) {
                int xb = bn_bit(&x, i), yb = bn_bit(&y, i);
                int bit = xb ? (yb ? 7 : 5) : (yb ? 6 : 4);
                if (opt & (1u << bit)) q.d[i / 32] |= 1u << (i % 32);
            }
            bn_norm(&q);
            bn_store(b, qt, &q, n, 1);
        }
    } else if (fn == 0x30) {
        bao_logf(b, BAO_LOG_WARN, "ALU function 0x%02X is not modelled", fn);
        return;
    } else {
        bao_logf(b, BAO_LOG_WARN, "ALU function 0x%02X does not exist, the engine stays idle", fn);
        return;
    }
    /* DIV adds div_done; comm_done is not in FR (alu.sv 143, 163 to 260) */
    sce_done(b, 4, fn == 0x01 ? 0x0F : 0x0D);
}

static void sdma_ich(bao_t *b)
{
    uint32_t segid = rd(b, SDMA + 0x54), sw = segid & 0xFF, sr = (segid >> 8) & 0xFF;
    uint32_t off_r = rd(b, SDMA + 0x58), off_w = rd(b, SDMA + 0x5C), n = rd(b, SDMA + 0x60), i, src, dst;
    uint32_t base[3] = { SCE_BASE + 0x0C00u, SCE_BASE + 0x1400u, SCE_BASE + 0x1800u };
    int ir = sr == 9 ? 0 : sr == 11 ? 1 : sr == 12 ? 2 : -1;
    int iw = sw == 9 ? 0 : sw == 11 ? 1 : sw == 12 ? 2 : -1;
    if (ir < 0 || iw < 0) {
        bao_logf(b, BAO_LOG_WARN, "ICH with unknown segment %u or %u", sr, sw);
        return;
    }
    src = base[ir] + 4 * off_r;
    dst = base[iw] + 4 * off_w;
    if (n > 0x400) n = 0x400;
    for (i = 0; i < n; i++) {
        uint32_t v = rd(b, src + 4 * i);
        wr(b, dst + 4 * i, v);
    }
    sce_done(b, 5, 0);
}

/* FR bits each function raises, from the states its mfsm walks through
 * (pke.sv 350 to 727): bit 0 mfsm, 1 pcore, 2 chnlo, 3 chnli, 4 chnlx.
 * A code the mfsm does not know leaves it idle. */
static uint32_t pke_fr_bits(uint32_t fn)
{
    switch (fn) {
    case 0x11: case 0x01: return 0x0B;
    case 0xFE: return 0x09;
    case 0x04: case 0x24: case 0x06: case 0x26: case 0x0B: case 0x2B: return 0x1F;
    case 0x02: case 0x09: case 0x0A: case 0x12: case 0x19: case 0x1A: case 0x18:
    case 0x03: case 0x23: case 0x07: case 0x27: case 0x05: case 0x25: case 0x08:
    case 0x3B: case 0x51: case 0x13: case 0x81: return 0x0F;
    case 0xFF: return 0x03;
    default: return 0;
    }
}

void pke_start(bao_t *b)
{
    uint32_t fn = rd(b, PKE + 0x00) & 0xFF, bits = pke_fr_bits(fn);
    if (!bits) {
        bao_logf(b, BAO_LOG_WARN, "PKE function 0x%02X does not exist, the engine stays idle", fn);
        return;
    }
    /* modinvready clears when an inverse starts (PkeCtrl.sv 1875, 2138) */
    if (fn == 0x18 || fn == 0x08 || fn == 0x07 || fn == 0x27)
        wr(b, PKE + 0x08, rd(b, PKE + 0x08) & ~0x100u);
    if (pke_run(b) == 0)
        sce_done(b, 2, bits);
}

void alu_start(bao_t *b)
{
    alu_run(b);
}

void sdma_ich_start(bao_t *b)
{
    sdma_ich(b);
}

void pke_free(bao_t *b)
{
    free(b->pke);
    b->pke = NULL;
}
