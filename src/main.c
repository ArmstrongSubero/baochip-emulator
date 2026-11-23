/*
 * main.c: baoemu command line front end
 *
 * Copyright (c) 2026 Armstrong Subero
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _WIN32
#define _POSIX_C_SOURCE 200809L
#endif
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include "baoemu.h"

#ifdef _WIN32
#include <conio.h>
#include <windows.h>
static int stdin_byte(void)
{
    if (!_kbhit()) return -1;
    {
        int c = _getch();
        return c == '\r' ? '\r' : c;
    }
}
static void nap(void) { Sleep(10); }
static double wall_ms(void) { return (double)GetTickCount64(); }
#else
#include <sys/select.h>
#include <unistd.h>
static int stdin_byte(void)
{
    fd_set s;
    struct timeval tv = { 0, 0 };
    unsigned char c;
    FD_ZERO(&s);
    FD_SET(0, &s);
    if (select(1, &s, NULL, NULL, &tv) <= 0) return -1;
    if (read(0, &c, 1) != 1) return -2;
    return c == '\n' ? '\r' : c;
}
static void nap(void)
{
    struct timespec ts = { 0, 10000000 };
    nanosleep(&ts, NULL);
}
static double wall_ms(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * 1000.0 + ts.tv_nsec / 1e6;
}
#endif

static const char *stop_names[] = {
    "running", "cycle limit", "guard", "exception", "breakpoint", "idle", "halt", "stopped"
};

typedef struct {
    bao_t *b;
    int trace_gpio, trace_spi, trace_i2c, quiet, uart_all;
    int usb_console, uart_mid;
    char usb_held[8192];
    int usb_len;
    uint32_t last_out[6], last_oe[6];
    int spi_col;
} cli_t;

static void usb_show(cli_t *c, int all);

static double now_ms(cli_t *c)
{
    return (double)bao_cycles(c->b) / (BAO_ACLK_HZ / 1000.0);
}

static void on_log(void *u, int level, const char *msg)
{
    cli_t *c = u;
    if (c->quiet && level < BAO_LOG_ERROR)
        return;
    fprintf(stderr, "[%10.3f ms] %s: %s\n", now_ms(c),
            level == BAO_LOG_ERROR ? "error" : level == BAO_LOG_WARN ? "warn" : "info", msg);
}

static void on_uart(void *u, int inst, int byte)
{
    cli_t *c = u;
    if (inst == 2 && c->usb_console && !c->uart_all)
        return;
    if (inst == 2 || c->uart_all) {
        if (c->uart_all && inst != 2)
            fprintf(stdout, "[UART%d]", inst);
        fputc(byte, stdout);
        if (inst == 2) {
            c->uart_mid = byte != '\n';
            if (!c->uart_mid && c->usb_len)
                usb_show(c, 0);
        }
    }
}

/* held USB serial text goes out as tagged whole lines once the UART is between lines */
static void usb_show(cli_t *c, int all)
{
    int i, bol = 1, end = c->usb_len;
    if (!all)
        while (end && c->usb_held[end - 1] != '\n')
            end--;
    for (i = 0; i < end; i++) {
        if (bol)
            fputs("[USB] ", stdout);
        fputc(c->usb_held[i], stdout);
        bol = c->usb_held[i] == '\n';
    }
    if (!bol)
        fputc('\n', stdout);
    memmove(c->usb_held, c->usb_held + end, (size_t)(c->usb_len - end));
    c->usb_len -= end;
}

/* the serial port the PC sees on USB: raw as the console, otherwise tagged lines */
static void on_usb(void *u, const uint8_t *data, int len)
{
    cli_t *c = u;
    int i;
    if (c->usb_console) {
        fwrite(data, 1, (size_t)len, stdout);
        return;
    }
    for (i = 0; i < len; i++) {
        if (c->usb_len == (int)sizeof(c->usb_held))
            usb_show(c, 1);
        c->usb_held[c->usb_len++] = (char)data[i];
    }
    if (!c->uart_mid)
        usb_show(c, 0);
}

static void on_gpio(void *u, int port, uint32_t out, uint32_t oe)
{
    cli_t *c = u;
    uint32_t changed = ((out ^ c->last_out[port]) & oe) | (oe ^ c->last_oe[port]);
    int pin;
    if (c->trace_gpio) {
        for (pin = 0; pin < 16; pin++) {
            if (!(changed & (1u << pin)))
                continue;
            if (oe & (1u << pin))
                fprintf(stderr, "[%10.3f ms] P%c%d = %d\n", now_ms(c), 'A' + port, pin, (out >> pin) & 1);
            else
                fprintf(stderr, "[%10.3f ms] P%c%d input\n", now_ms(c), 'A' + port, pin);
        }
    }
    c->last_out[port] = out;
    c->last_oe[port] = oe;
}

static int on_spi(void *u, int inst, int cs, int tx)
{
    cli_t *c = u;
    (void)cs;
    if (c->trace_spi) {
        fprintf(stderr, "%02X ", tx);
        if (++c->spi_col == 16) { fputc('\n', stderr); c->spi_col = 0; }
    }
    (void)inst;
    return -1;
}

static void on_spi_cs(void *u, int inst, int cs, int active)
{
    cli_t *c = u;
    if (!c->trace_spi)
        return;
    if (c->spi_col) { fputc('\n', stderr); c->spi_col = 0; }
    fprintf(stderr, "[%10.3f ms] SPIM%d CS%d %s\n", now_ms(c), inst, cs, active ? "low" : "high");
}

static int on_i2c(void *u, int inst, int ev, uint8_t *data)
{
    cli_t *c = u;
    static const char *names[] = { "START", "ADDR", "WRITE", "READ", "STOP" };
    if (ev == BAO_I2C_READ)
        *data = 0xFF;
    if (c->trace_i2c) {
        if (ev == BAO_I2C_ADDR || ev == BAO_I2C_WRITE)
            fprintf(stderr, "[%10.3f ms] I2C%d %s 0x%02X (no device, NACK)\n", now_ms(c), inst, names[ev], *data);
        else
            fprintf(stderr, "[%10.3f ms] I2C%d %s\n", now_ms(c), inst, names[ev]);
    }
    return 0;
}

static void usage(void)
{
    fprintf(stderr,
        "baoemu %s, Baochip-1x emulator\n"
        "usage: baoemu [options] program.elf|program.uf2|program.bin\n"
        "  --ms N             stop after N emulated milliseconds\n"
        "  --cycles N         stop after N cycles (350 MHz)\n"
        "  --input TEXT       queue TEXT on the console UART receive line\n"
        "  --usb-input TEXT   type TEXT into the USB serial port once the PC side has opened it\n"
        "  --usb-console      show the USB serial port as the console and send typed keys to it\n"
        "  --no-usb           leave the USB port unplugged\n"
        "  --button PIN=LVL   drive an input pin, for example PB3=0\n"
        "  --adc CH=MV        set an ADC channel in millivolts\n"
        "  --rram FILE        load RRAM from FILE if it exists and save it on exit\n"
        "  --spi-loopback     wire SPIM2 MISO to MOSI\n"
        "  --mpu6050          MPU6050 on I2C0 at 0x68\n"
        "  --w25q FILE        W25Q128 flash on SPIM1 CS0, kept in FILE\n"
        "  --sd IMAGE         SD card on SPIM2 CS0 backed by a disk image\n"
        "  --attach K:B:S[:O]  attach part K on bus B, select S, options O,\n"
        "                     e.g. st7796:2:0:dc=PC10,cs=PC6 or ft6336:0:0x38:int=PB4\n"
        "  --touch X,Y        hold a touch at X,Y on the first touch controller\n"
        "  --screenshot FILE  write the display to FILE (PPM) at the end\n"
        "  --trace-gpio       print output pin changes\n"
        "  --trace-spi        print SPI bytes and chip select changes\n"
        "  --trace-i2c        print I2C bus activity\n"
        "  --uart-all         also print UART0, UART1 and UART3\n"
        "  --uart-instant     skip UART bit timing\n"
        "  --no-guard         allow accesses that brick a real board\n"
        "  --keep-going       deliver exceptions to mtvec or stvec instead of stopping\n"
        "  --no-fast-forward  run idle loops instruction by instruction\n"
        "  --interactive      send typed keys to UART2, wait on idle, run in real time\n"
        "  --realtime         never run ahead of the wall clock\n"
        "  --quiet            hide warnings\n", bao_version());
}

int main(int argc, char **argv)
{
    cli_t c;
    const char *prog = NULL, *rram = NULL, *input = NULL, *touch_spec = NULL, *shot = NULL;
    const char *usb_input = NULL;
    uint64_t limit = 0;
    int interactive = 0, eof = 0, realtime = 0;
    double w0;
    int i, r;
    clock_t t0;
    double host;

    memset(&c, 0, sizeof(c));
    c.b = bao_new();
    if (!c.b) {
        fprintf(stderr, "out of memory\n");
        return 2;
    }
    bao_on_log(c.b, on_log, &c);
    bao_on_uart(c.b, on_uart, &c);
    bao_on_gpio(c.b, on_gpio, &c);
    bao_on_spi(c.b, on_spi, on_spi_cs, &c);
    bao_on_i2c(c.b, on_i2c, &c);
    bao_on_usb_serial(c.b, on_usb, &c);

    for (i = 1; i < argc; i++) {
        const char *a = argv[i];
        if (!strcmp(a, "--ms") && i + 1 < argc) limit = (uint64_t)atof(argv[++i]) * (BAO_ACLK_HZ / 1000);
        else if (!strcmp(a, "--cycles") && i + 1 < argc) limit = strtoull(argv[++i], NULL, 0);
        else if (!strcmp(a, "--input") && i + 1 < argc) input = argv[++i];
        else if (!strcmp(a, "--rram") && i + 1 < argc) rram = argv[++i];
        else if (!strcmp(a, "--usb-input") && i + 1 < argc) usb_input = argv[++i];
        else if (!strcmp(a, "--usb-console")) c.usb_console = 1;
        else if (!strcmp(a, "--no-usb")) bao_usb_attach(c.b, 0);
        else if (!strcmp(a, "--trace-gpio")) c.trace_gpio = 1;
        else if (!strcmp(a, "--trace-spi")) c.trace_spi = 1;
        else if (!strcmp(a, "--trace-i2c")) c.trace_i2c = 1;
        else if (!strcmp(a, "--uart-all")) c.uart_all = 1;
        else if (!strcmp(a, "--uart-instant")) bao_set_uart_instant(c.b, 1);
        else if (!strcmp(a, "--no-guard")) bao_set_guard(c.b, 0);
        else if (!strcmp(a, "--keep-going")) bao_set_stop_on_exception(c.b, 0);
        else if (!strcmp(a, "--quiet")) c.quiet = 1;
        else if (!strcmp(a, "--interactive")) interactive = realtime = 1;
        else if (!strcmp(a, "--realtime")) realtime = 1;
        else if (!strcmp(a, "--no-fast-forward")) bao_set_fast_forward(c.b, 0);
        else if (!strcmp(a, "--spi-loopback")) bao_attach(c.b, "spi-loopback", 2, 0, NULL);
        else if (!strcmp(a, "--mpu6050")) bao_attach(c.b, "mpu6050", 0, 0x68, NULL);
        else if (!strcmp(a, "--w25q") && i + 1 < argc) bao_attach(c.b, "w25q", 1, 0, argv[++i]);
        else if (!strcmp(a, "--sd") && i + 1 < argc) bao_attach(c.b, "sd", 2, 0, argv[++i]);
        else if (!strcmp(a, "--attach") && i + 1 < argc) {
            char spec[300], *kind = spec, *bus, *sel, *opt;
            snprintf(spec, sizeof(spec), "%s", argv[++i]);
            bus = strchr(kind, ':'); if (bus) *bus++ = 0;
            sel = bus ? strchr(bus, ':') : NULL; if (sel) *sel++ = 0;
            opt = sel ? strchr(sel, ':') : NULL; if (opt) *opt++ = 0;
            if (bao_attach(c.b, kind, bus ? atoi(bus) : 0, sel ? (int)strtol(sel, NULL, 0) : 0, opt)) return 2;
        }
        else if (!strcmp(a, "--touch") && i + 1 < argc) touch_spec = argv[++i];
        else if (!strcmp(a, "--screenshot") && i + 1 < argc) shot = argv[++i];
        else if (!strcmp(a, "--button") && i + 1 < argc) {
            const char *s = argv[++i];
            if (s[0] == 'P' && s[1] >= 'A' && s[1] <= 'F') {
                int pin = atoi(s + 2);
                const char *eq = strchr(s, '=');
                bao_gpio_set_input(c.b, s[1] - 'A', pin, eq ? atoi(eq + 1) : 0);
            }
        } else if (!strcmp(a, "--adc") && i + 1 < argc) {
            const char *s = argv[++i], *eq = strchr(s, '=');
            if (eq) bao_adc_set_mv(c.b, atoi(s), (uint32_t)atoi(eq + 1));
        } else if (a[0] == '-') {
            usage();
            return 2;
        } else prog = a;
    }
    if (!prog) {
        usage();
        return 2;
    }
    if (rram) {
        FILE *f = fopen(rram, "rb");
        if (f) { fclose(f); bao_rram_image_load(c.b, rram); }
    }
    if (bao_load_file(c.b, prog))
        return 2;
    if (input)
        bao_uart_rx(c.b, 2, (const uint8_t *)input, (int)strlen(input));
    if (touch_spec) {
        const char *cm = strchr(touch_spec, ',');
        bao_touch(c.b, 0, 1, atoi(touch_spec), cm ? atoi(cm + 1) : 0);
    }

    t0 = clock();
    w0 = wall_ms();
    for (;;) {
        uint64_t chunk = BAO_ACLK_HZ / 100;
        if (limit) {
            uint64_t left = limit - bao_cycles(c.b);
            if (left == 0) break;
            if (left < chunk) chunk = left;
        }
        while (realtime && (double)bao_cycles(c.b) / (BAO_ACLK_HZ / 1000.0) > wall_ms() - w0)
            nap();
        if (interactive && !eof) {
            int k;
            while ((k = stdin_byte()) >= 0) {
                uint8_t ch = (uint8_t)k;
                if (c.usb_console)
                    bao_usb_serial_rx(c.b, &ch, 1);
                else
                    bao_uart_rx(c.b, 2, &ch, 1);
            }
            if (k == -2) eof = 1;
        }
        if (usb_input && bao_usb_serial_open(c.b)) {
            bao_usb_serial_rx(c.b, (const uint8_t *)usb_input, (int)strlen(usb_input));
            usb_input = NULL;
        }
        r = bao_run(c.b, chunk);
        fflush(stdout);
        if (r == BAO_STOP_IDLE && interactive && !eof) {
            nap();
            continue;
        }
        if (r == BAO_STOP_IDLE && interactive)
            break;
        if (r != BAO_STOP_LIMIT)
            break;
        if (limit && bao_cycles(c.b) >= limit)
            break;
    }
    host = (double)(clock() - t0) / CLOCKS_PER_SEC;
    r = bao_stop_reason(c.b);

    if (c.usb_len) {
        if (c.uart_mid)
            fputc('\n', stdout);
        usb_show(&c, 1);
    }
    fflush(stdout);
    fprintf(stderr, "\n[%10.3f ms] stop: %s%s%s\n", now_ms(&c), stop_names[r],
            bao_stop_message(c.b)[0] ? ", " : "", bao_stop_message(c.b));
    fprintf(stderr, "             %llu cycles (%llu skipped in idle loops), %.2f s host time, %u RRAM page commits\n",
            (unsigned long long)bao_cycles(c.b), (unsigned long long)bao_skipped_cycles(c.b), host,
            bao_rram_commits(c.b));
    if (rram)
        bao_rram_image_save(c.b, rram);
    if (shot) {
        static uint8_t px[320 * 480 * 3];
        FILE *f = fopen(shot, "wb");
        if (f && bao_display_rgb(c.b, px, NULL) == 0) {
            fprintf(f, "P6\n320 480\n255\n");
            fwrite(px, 1, sizeof(px), f);
        }
        if (f) fclose(f);
    }
    bao_free(c.b);
    return (r == BAO_STOP_GUARD || r == BAO_STOP_EXCEPTION) ? 1 : 0;
}
