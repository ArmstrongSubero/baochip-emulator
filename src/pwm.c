/*
 * pwm.c: PWM slices, modelled on the PULP apb_adv_timer RTL
 *
 * Copyright (c) 2026 Armstrong Subero
 * SPDX-License-Identifier: Apache-2.0
 *
 * Four slices of four channels, ticking at PCLK divided by the prescaler.
 */

#include <string.h>
#include "bao_int.h"

enum { OP_SET, OP_TOGRST, OP_SETRST, OP_TOG, OP_RST, OP_TOGSET, OP_RSTSET };

static void load_cfg(bao_t *b, int s)
{
    pwm_slice_t *p = &b->pwms[s];
    uint32_t *r = &b->pwm[(s * 0x40) >> 2];
    uint32_t cfg = r[1], th = r[2];
    int c;
    p->sawtooth = (cfg >> 12) & 1;
    p->tick = 8u * (((cfg >> 16) & 0xFF) + 1);
    if ((cfg >> 11) & 1)
        p->tick = BAO_ACLK_HZ / 32768u;
    p->start = th & 0xFFFF;
    p->end = th >> 16;
    for (c = 0; c < 4; c++) {
        p->comp[c] = r[3 + c] & 0xFFFF;
        p->op[c] = (r[3 + c] >> 16) & 7;
    }
    p->pending = 0;
}

static uint32_t span(pwm_slice_t *p)
{
    uint32_t d = p->end >= p->start ? p->end - p->start : 0;
    if (p->sawtooth) return d + 1;
    return d ? 2 * d : 1;
}

static uint32_t counter_at(pwm_slice_t *p, uint64_t k)
{
    uint32_t P = span(p), ph = (uint32_t)(k % P), d = P / 2;
    if (p->sawtooth) return p->start + ph;
    if (ph <= d) return p->start + ph;
    return p->end - (ph - d);
}

static uint64_t next_hit(pwm_slice_t *p, uint64_t after, uint32_t x)
{
    uint32_t P = span(p), d = p->end - p->start;
    uint64_t best = UINT64_MAX;
    uint32_t phases[2];
    int n = 0, i;
    if (x < p->start || x > p->end)
        return UINT64_MAX;
    phases[n++] = x - p->start;
    if (!p->sawtooth && x != p->start && x != p->end)
        phases[n++] = 2 * d - (x - p->start);
    for (i = 0; i < n; i++) {
        uint64_t base = (after / P) * P + phases[i];
        if (base <= after) base += P;
        if (base < best) best = base;
    }
    return best;
}

static uint64_t slice_next_tick(pwm_slice_t *p)
{
    uint64_t after = p->done, n = UINT64_MAX, t;
    int c;
    for (c = 0; c < 4; c++) {
        t = next_hit(p, after, p->comp[c]);
        if (t < n) n = t;
    }
    t = next_hit(p, after, p->end);
    if (t < n) n = t;
    return n;
}

static void tick_event(bao_t *b, int s, uint64_t k)
{
    pwm_slice_t *p = &b->pwms[s];
    uint32_t cnt = counter_at(p, k);
    int is_end = p->sawtooth && cnt == p->end, c;

    for (c = 0; c < 4; c++) {
        int match = cnt == p->comp[c], v = p->val[c];
        int second = p->sawtooth ? is_end : match;
        switch (p->op[c]) {
        case OP_SET: if (match) v = 1; break;
        case OP_TOG: if (match) v = !v; break;
        case OP_RST: if (match) v = 0; break;
        case OP_TOGRST: case OP_SETRST: case OP_TOGSET: case OP_RSTSET: {
            int first = p->op[c] == OP_TOGRST || p->op[c] == OP_TOGSET ? -1 :
                        p->op[c] == OP_SETRST ? 1 : 0;
            int after = p->op[c] == OP_TOGRST || p->op[c] == OP_SETRST ? 0 : 1;
            if (p->sawtooth) {
                if (match) v = first < 0 ? !v : first;
                else if (second) v = after;
            } else if (match) {
                if (!p->second[c]) { v = first < 0 ? !v : first; p->second[c] = 1; }
                else { v = after; p->second[c] = 0; }
            }
            break;
        }
        default: break;
        }
        p->val[c] = v;
    }
    if (is_end) {
        char ev[12] = "pwm0_ev";
        ev[3] = (char)('0' + s);
        if (b->pwm[0x100 >> 2] & (1u << s))
            irq_raise(b, ev);
        if (p->pending) {
            load_cfg(b, s);
            p->origin += (k + 1) * p->tick;
            p->done = UINT64_MAX;
            return;
        }
    }
    p->done = k;
}

static uint64_t tick_cycle(pwm_slice_t *p, uint64_t k)
{
    return p->origin + k * p->tick;
}

void pwm_update(bao_t *b)
{
    int s, changed = 0;
    for (s = 0; s < 4; s++) {
        pwm_slice_t *p = &b->pwms[s];
        int guard = 0;
        while (p->active && p->gated_at == NEVER && guard++ < 100000) {
            uint64_t k;
            if (p->done == UINT64_MAX) {
                k = 0;
                p->done = UINT64_MAX;
                if (tick_cycle(p, 0) > b->cycles) break;
                {
                    uint32_t before = p->val[0] | (p->val[1] << 1) | (p->val[2] << 2) | (p->val[3] << 3);
                    uint32_t cnt = counter_at(p, 0);
                    int c, hit = cnt == p->end && p->sawtooth;
                    for (c = 0; c < 4; c++) if (p->comp[c] == cnt) hit = 1;
                    if (hit) tick_event(b, s, 0);
                    else p->done = 0;
                    if ((uint32_t)(p->val[0] | (p->val[1] << 1) | (p->val[2] << 2) | (p->val[3] << 3)) != before) changed = 1;
                }
                continue;
            }
            k = slice_next_tick(p);
            if (k == UINT64_MAX || tick_cycle(p, k) > b->cycles)
                break;
            {
                uint32_t before = p->val[0] | (p->val[1] << 1) | (p->val[2] << 2) | (p->val[3] << 3);
                tick_event(b, s, k);
                if ((uint32_t)(p->val[0] | (p->val[1] << 1) | (p->val[2] << 2) | (p->val[3] << 3)) != before) changed = 1;
            }
        }
    }
    if (changed)
        gpio_notify(b);
}

uint64_t pwm_next_event(bao_t *b)
{
    uint64_t n = NEVER;
    int s;
    for (s = 0; s < 4; s++) {
        pwm_slice_t *p = &b->pwms[s];
        uint64_t k, t;
        if (!p->active || p->gated_at != NEVER) continue;
        if (p->done == UINT64_MAX) t = tick_cycle(p, 0);
        else {
            k = slice_next_tick(p);
            if (k == UINT64_MAX) continue;
            t = tick_cycle(p, k);
        }
        if (t < n) n = t;
    }
    return n;
}

static void restart(bao_t *b, int s)
{
    pwm_slice_t *p = &b->pwms[s];
    p->origin = b->cycles + 1;
    p->done = UINT64_MAX;
}

/* a gated slice keeps its phase: moving the origin by the time it was
 * stopped resumes the count where it froze */
static void pwm_gate(bao_t *b, uint32_t en)
{
    int s;
    for (s = 0; s < 4; s++) {
        pwm_slice_t *p = &b->pwms[s];
        int on = (int)((en >> s) & 1);
        if (!on && p->gated_at == NEVER)
            p->gated_at = b->cycles;
        else if (on && p->gated_at != NEVER) {
            p->origin += b->cycles - p->gated_at;
            p->gated_at = NEVER;
        }
    }
}

int pwm_write(bao_t *b, uint32_t off, uint32_t v)
{
    int s = (int)(off / 0x40), c;
    pwm_update(b);
    b->pwm[off >> 2] = v;
    if (off == 0x104) {
        pwm_gate(b, v & 0xF);
        bao_kick(b);
        return 0;
    }
    if (off < 0x100 && (off & 0x3F) == 0 && b->pwms[s].gated_at != NEVER)
        return 0;
    if (off < 0x100 && (off & 0x3F) == 0) {
        pwm_slice_t *p = &b->pwms[s];
        if (v & 8) {
            load_cfg(b, s);
            for (c = 0; c < 4; c++) { p->val[c] = 0; p->second[c] = 0; }
            restart(b, s);
        }
        if (v & 16) p->armed = 1;
        if (v & 4) {
            if (!p->active) { load_cfg(b, s); restart(b, s); }
            else p->pending = 1;
        }
        if (v & 1) {
            if (p->armed || !p->loaded) { load_cfg(b, s); p->armed = 0; }
            p->loaded = 1;
            if (!p->active) restart(b, s);
            p->active = 1;
        }
        if (v & 2) p->active = 0;
        gpio_notify(b);
        bao_kick(b);
    }
    return 0;
}

int pwm_read(bao_t *b, uint32_t off, uint32_t *v)
{
    static const uint32_t slice_mask[16] = {
        0, 0x00FF1FFFu, 0xFFFFFFFFu, 0x7FFFFu, 0x7FFFFu, 0x7FFFFu, 0x7FFFFu,
        0x3FFFFu, 0x3FFFFu, 0x3FFFFu, 0x3FFFFu, 0, 0, 0, 0, 0
    };
    uint32_t m = 0;
    pwm_update(b);
    if (off < 0x100 && (off & 0x3F) == 0x2C) {
        pwm_slice_t *p = &b->pwms[off / 0x40];
        uint64_t now = p->gated_at != NEVER ? p->gated_at : b->cycles;
        if (p->active && p->tick && now >= p->origin)
            *v = counter_at(p, (now - p->origin) / p->tick);
        else
            *v = p->start;
        return 0;
    }
    if (off < 0x100) m = slice_mask[(off & 0x3F) >> 2];
    else if (off == 0x100) m = 0xFFFFFu;
    else if (off == 0x104) m = 0xFu;
    else if (off >= 0x140 && off < 0x150) m = 0x3FFu;
    *v = b->pwm[off >> 2] & m;
    return 0;
}

int pwm_pin(bao_t *b, int port, int pin, int *level)
{
    uint32_t af;
    int s;
    if (pin > 3 || (port != 1 && port != 2))
        return 0;
    af = (b->iox[port * 2] >> (2 * pin)) & 3;
    if (af != 3)
        return 0;
    s = port == 1 ? 1 : 2;
    *level = b->pwms[s].val[pin];
    return 1;
}

BAO_API int bao_pwm_get(bao_t *b, int slice, int ch, uint32_t *period_cycles, uint32_t *high_cycles)
{
    pwm_slice_t *p;
    uint32_t P;
    if (slice < 0 || slice > 3 || ch < 0 || ch > 3)
        return -1;
    p = &b->pwms[slice];
    if (!p->active)
        return -1;
    P = span(p);
    *period_cycles = P * p->tick;
    if (p->op[ch] == OP_SETRST && p->sawtooth && p->comp[ch] >= p->start && p->comp[ch] <= p->end)
        *high_cycles = (p->end - p->comp[ch]) * p->tick;
    else if (p->op[ch] == OP_RSTSET && p->sawtooth && p->comp[ch] >= p->start && p->comp[ch] <= p->end)
        *high_cycles = (P - (p->end - p->comp[ch])) * p->tick;
    else
        *high_cycles = p->val[ch] ? *period_cycles : 0;
    return 0;
}

void pwm_reset(bao_t *b)
{
    int s;
    memset(b->pwms, 0, sizeof(b->pwms));
    for (s = 0; s < 4; s++)
        b->pwms[s].gated_at = 0;
}
