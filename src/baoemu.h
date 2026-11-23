/*
 * baoemu.h: public API of the Baochip-1x emulator
 *
 * Copyright (c) 2026 Armstrong Subero
 * SPDX-License-Identifier: Apache-2.0
 *
 * No global state. Time is counted in ACLK cycles (350 MHz).
 */

#ifndef BAOEMU_H
#define BAOEMU_H

#include <stdint.h>

#ifdef _WIN32
#define BAO_API __declspec(dllexport)
#else
#define BAO_API __attribute__((visibility("default")))
#endif

#ifdef __cplusplus
extern "C" {
#endif

#define BAO_ACLK_HZ        350000000u
#define BAO_PERCLK_HZ      99000000u

enum bao_stop {
    BAO_RUNNING = 0,
    BAO_STOP_LIMIT,      /* cycle budget used up */
    BAO_STOP_GUARD,      /* touched a region that bricks a real board */
    BAO_STOP_EXCEPTION,  /* synchronous trap with stop on exception set */
    BAO_STOP_BREAK,      /* ebreak */
    BAO_STOP_IDLE,       /* wfi with nothing left that can wake the core */
    BAO_STOP_HALT,       /* tight self loop, such as the SDK fault halt */
    BAO_STOP_USER        /* bao_stop() from a callback */
};

enum bao_log_level { BAO_LOG_INFO = 0, BAO_LOG_WARN = 1, BAO_LOG_ERROR = 2 };

enum bao_i2c_event {
    BAO_I2C_START = 0,   /* data: none, return ignored */
    BAO_I2C_ADDR,        /* data: address byte with R/W bit, return 1 for ACK */
    BAO_I2C_WRITE,       /* data: byte written, return 1 for ACK */
    BAO_I2C_READ,        /* data: fill with byte read, return ignored */
    BAO_I2C_STOP
};

typedef struct bao bao_t;

typedef void (*bao_log_cb)(void *user, int level, const char *msg);
typedef void (*bao_uart_cb)(void *user, int inst, int byte);
typedef void (*bao_gpio_cb)(void *user, int port, uint32_t out, uint32_t oe);
typedef int  (*bao_spi_cb)(void *user, int inst, int cs, int tx);
typedef void (*bao_spi_cs_cb)(void *user, int inst, int cs, int active);
typedef int  (*bao_i2c_cb)(void *user, int inst, int event, uint8_t *data);
typedef void (*bao_usb_cb)(void *user, const uint8_t *data, int len);

BAO_API bao_t      *bao_new(void);
BAO_API void        bao_free(bao_t *b);
BAO_API void        bao_reset(bao_t *b);
BAO_API const char *bao_version(void);

BAO_API int  bao_load_elf(bao_t *b, const char *path);
BAO_API int  bao_load_uf2(bao_t *b, const char *path);
BAO_API int  bao_load_bin(bao_t *b, const char *path, uint32_t addr);
BAO_API int  bao_load_file(bao_t *b, const char *path);
BAO_API int  bao_rram_image_load(bao_t *b, const char *path);
BAO_API int  bao_rram_image_save(bao_t *b, const char *path);

BAO_API int      bao_run(bao_t *b, uint64_t cycles);
BAO_API void     bao_stop(bao_t *b);
BAO_API int      bao_stop_reason(bao_t *b);
BAO_API const char *bao_stop_message(bao_t *b);
BAO_API uint64_t bao_cycles(bao_t *b);
BAO_API uint64_t bao_millis(bao_t *b);

BAO_API uint32_t bao_get_pc(bao_t *b);
BAO_API void     bao_set_pc(bao_t *b, uint32_t pc);
BAO_API uint32_t bao_get_reg(bao_t *b, int r);
BAO_API void     bao_set_reg(bao_t *b, int r, uint32_t v);
BAO_API int      bao_mem_read(bao_t *b, uint32_t addr, uint8_t *buf, uint32_t len);
BAO_API int      bao_mem_write(bao_t *b, uint32_t addr, const uint8_t *buf, uint32_t len);
/* a peripheral word read the way the CPU reads it, side effects included */
BAO_API int      bao_peek32(bao_t *b, uint32_t addr, uint32_t *val);
BAO_API uint32_t bao_symbol(bao_t *b, const char *name);
BAO_API const char *bao_where(bao_t *b, uint32_t pc);

BAO_API void bao_set_guard(bao_t *b, int on);
BAO_API void bao_set_stop_on_exception(bao_t *b, int on);
BAO_API void bao_set_uart_instant(bao_t *b, int on);
BAO_API void bao_set_fast_forward(bao_t *b, int on);
BAO_API uint64_t bao_skipped_cycles(bao_t *b);
BAO_API uint32_t bao_rram_commits(bao_t *b);

BAO_API void bao_on_log(bao_t *b, bao_log_cb cb, void *user);
BAO_API void bao_on_uart(bao_t *b, bao_uart_cb cb, void *user);
BAO_API void bao_on_gpio(bao_t *b, bao_gpio_cb cb, void *user);
BAO_API void bao_on_spi(bao_t *b, bao_spi_cb cb, bao_spi_cs_cb cs, void *user);
BAO_API void bao_on_i2c(bao_t *b, bao_i2c_cb cb, void *user);

/* bus settings, for a host that draws the bus lines: SPI clkdiv in bits 7:0, CPHA bit 8, CPOL bit 9
 * (one bit takes 2 * (clkdiv + 1) peripheral clocks); the I2C divider (one bit takes 5 * (div + 1)) */
BAO_API uint32_t bao_spi_config(bao_t *b, int inst);
BAO_API uint32_t bao_i2c_div(bao_t *b, int inst);

BAO_API int  bao_attach(bao_t *b, const char *kind, int bus, int sel, const char *path);
BAO_API int  bao_touch(bao_t *b, int id, int down, int x, int y);
BAO_API int  bao_display_rgb(bao_t *b, uint8_t *out, uint32_t *frames);
BAO_API int  bao_display_rgb_rot(bao_t *b, uint8_t *out, int rot);
BAO_API int  bao_mpu6050_set(bao_t *b, int16_t ax, int16_t ay, int16_t az, int16_t gx, int16_t gy, int16_t gz);
BAO_API void bao_uart_rx(bao_t *b, int inst, const uint8_t *data, int len);
BAO_API void bao_gpio_set_input(bao_t *b, int port, int pin, int level);
BAO_API uint32_t bao_gpio_get_out(bao_t *b, int port);
BAO_API uint32_t bao_gpio_get_oe(bao_t *b, int port);
BAO_API uint32_t bao_gpio_get_af(bao_t *b, int port);
BAO_API int  bao_pwm_get(bao_t *b, int slice, int ch, uint32_t *period_cycles, uint32_t *high_cycles);
BAO_API void bao_adc_set_mv(bao_t *b, int channel, uint32_t mv);

/* the PC on the USB port: attached by default, bytes of its serial port (CDC) */
BAO_API void bao_usb_attach(bao_t *b, int on);
BAO_API void bao_on_usb_serial(bao_t *b, bao_usb_cb cb, void *user);
BAO_API void bao_usb_serial_rx(bao_t *b, const uint8_t *data, int len);
BAO_API int  bao_usb_serial_open(bao_t *b);

/* the device's USB disk as the PC sees it: block count is 0 until the PC has found it.
 * read and write start a transfer of at most 64 KB and return 0, or -1 if the disk is
 * not ready, busy or the range is bad. busy is 1 while it runs, 0 when done, -1 on failure. */
BAO_API uint32_t bao_usb_disk_blocks(bao_t *b);
BAO_API uint32_t bao_usb_disk_block_size(bao_t *b);
BAO_API int  bao_usb_disk_read(bao_t *b, uint32_t lba, uint32_t count);
BAO_API int  bao_usb_disk_write(bao_t *b, uint32_t lba, uint32_t count, const uint8_t *data);
BAO_API int  bao_usb_disk_busy(bao_t *b);
BAO_API uint32_t bao_usb_disk_data(bao_t *b, uint8_t *out, uint32_t len);

#ifdef __cplusplus
}
#endif

#endif
