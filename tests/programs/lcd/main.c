/*
 * main.c: ST7796S and FT6336U test for baoemu, DC on PC10, CS on PC6
 *
 * Copyright (c) 2026 Armstrong Subero
 * SPDX-License-Identifier: Apache-2.0
 */

#include "bao.h"
#include "hardware/spi.h"
#include "hardware/i2c.h"

#define DC_PIN 10
#define CS_PIN 6

static void cmd(uint8_t c, const uint8_t *p, uint32_t n)
{
    gpio_put(GPIO_PORT_C, CS_PIN, false);
    gpio_put(GPIO_PORT_C, DC_PIN, false);
    spi_write_blocking(2, &c, 1, 1);
    gpio_put(GPIO_PORT_C, DC_PIN, true);
    if (n) spi_write_blocking(2, p, n, 1);
    gpio_put(GPIO_PORT_C, CS_PIN, true);
}

static void window(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
    uint8_t a[4] = { x0 >> 8, x0, x1 >> 8, x1 }, b[4] = { y0 >> 8, y0, y1 >> 8, y1 };
    cmd(0x2A, a, 4);
    cmd(0x2B, b, 4);
}

static void fill(uint16_t x0, uint16_t y0, uint16_t w, uint16_t h, uint16_t c)
{
    static uint8_t line[256];
    window(x0, y0, x0 + w - 1, y0 + h - 1);
    for (int i = 0; i < 256; i += 2) { line[i] = c >> 8; line[i + 1] = c; }
    uint32_t left = (uint32_t)w * h * 2;
    gpio_put(GPIO_PORT_C, CS_PIN, false);
    gpio_put(GPIO_PORT_C, DC_PIN, false);
    uint8_t c2c = 0x2C;
    spi_write_blocking(2, &c2c, 1, 1);
    gpio_put(GPIO_PORT_C, DC_PIN, true);
    while (left) {
        uint32_t n = left > 256 ? 256 : left;
        spi_write_blocking(2, line, n, 1);
        left -= n;
    }
    gpio_put(GPIO_PORT_C, CS_PIN, true);
}

int main(void)
{
    bao_init();
    gpio_init(GPIO_PORT_C, DC_PIN); gpio_set_dir(GPIO_PORT_C, DC_PIN, true);
    gpio_init(GPIO_PORT_C, CS_PIN); gpio_set_dir(GPIO_PORT_C, CS_PIN, true);
    gpio_put(GPIO_PORT_C, CS_PIN, true);
    spi_init(2);
    i2c_init(0, 400000);
    uint8_t p;
    cmd(0x01, 0, 0); delay_ms(5);
    cmd(0x11, 0, 0); delay_ms(5);
    p = 0x55; cmd(0x3A, &p, 1);
    p = 0x48; cmd(0x36, &p, 1);
    cmd(0x29, 0, 0);
    fill(0, 0, 320, 480, 0x0000);
    fill(0, 0, 320, 160, 0xF800);
    fill(0, 160, 320, 160, 0x07E0);
    fill(0, 320, 320, 160, 0x001F);
    fill(140, 220, 40, 40, 0xFFFF);
    mini_printf("panel drawn\r\n");
    uint8_t reg = 0xA3, id = 0;
    i2c_write_read_blocking(0, 0x38, &reg, 1, &id, 1);
    mini_printf("touch chip id 0x%02x\r\n", id);
    int shown = 0;
    while (1) {
        uint8_t r = 0x02, t[5];
        i2c_write_read_blocking(0, 0x38, &r, 1, t, 5);
        if ((t[0] & 0x0F) && !shown) {
            uint16_t x = ((t[1] & 0x0F) << 8) | t[2], y = ((t[3] & 0x0F) << 8) | t[4];
            mini_printf("touch at %u,%u\r\n", x, y);
            fill(x - 10, y - 10, 20, 20, 0xFFE0);
            shown = 1;
        }
        delay_ms(10);
    }
}
