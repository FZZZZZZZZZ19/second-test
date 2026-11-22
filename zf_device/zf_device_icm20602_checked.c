/* SPDX-License-Identifier: GPL-3.0-or-later */
#include "zf_device_icm20602_checked.h"
#include "zf_common_headfile.h"
#include <math.h>
static uint32_t (*clock_us)(void);
static uint32_t began;
static bool bad;
static bool budget(void) {
    if (clock_us() - began >= 1800)
        bad = true;
    return !bad;
}
/* TIM7 bounds transfers in the foreground. GPIO access uses the board's IMU pins. */
static void wait_bit(void) {
    uint32_t start = clock_us();
    while (clock_us() - start < 2u) {
    }
}
static void low(void) {
    gpio_set_level(ICM20602_SCL_PIN, 0);
    wait_bit();
}
static bool high(void) {
    gpio_set_level(ICM20602_SCL_PIN, 1);
    while (!gpio_get_level(ICM20602_SCL_PIN))
        if (!budget())
            return false;
    wait_bit();
    return budget();
}
static bool start(void) {
    gpio_set_level(ICM20602_SDA_PIN, 1);
    wait_bit();
    if (!high() || !gpio_get_level(ICM20602_SDA_PIN)) {
        bad = true;
        return false;
    }
    gpio_set_level(ICM20602_SDA_PIN, 0);
    wait_bit();
    low();
    return true;
}
static void stop(void) {
    low();
    gpio_set_level(ICM20602_SDA_PIN, 0);
    wait_bit();
    (void)high();
    gpio_set_level(ICM20602_SDA_PIN, 1);
    wait_bit();
}
static bool send(uint8_t v) {
    for (unsigned i = 0; i < 8; i++) {
        gpio_set_level(ICM20602_SDA_PIN, (v & 128) != 0);
        wait_bit();
        if (!high())
            return false;
        low();
        v <<= 1;
    }
    gpio_set_level(ICM20602_SDA_PIN, 1);
    wait_bit();
    if (!high())
        return false;
    bool ack = !gpio_get_level(ICM20602_SDA_PIN);
    low();
    return ack;
}
static uint8_t receive(bool ack) {
    uint8_t v = 0;
    gpio_set_level(ICM20602_SDA_PIN, 1);
    for (unsigned i = 0; i < 8; i++) {
        if (!high())
            return 0;
        v = (uint8_t)((v << 1) | gpio_get_level(ICM20602_SDA_PIN));
        low();
    }
    gpio_set_level(ICM20602_SDA_PIN, ack ? 0 : 1);
    wait_bit();
    (void)high();
    low();
    gpio_set_level(ICM20602_SDA_PIN, 1);
    return v;
}
static bool read_regs(uint8_t reg, uint8_t *data, unsigned n) {
    bool ok = start() && send(ICM20602_DEV_ADDR << 1) && send(reg) && start() &&
              send((ICM20602_DEV_ADDR << 1) | 1);
    if (ok)
        for (unsigned i = 0; i < n && !bad; i++)
            data[i] = receive(i + 1 < n);
    stop();
    return ok && budget();
}
static bool write_reg(uint8_t reg, uint8_t value) {
    bool ok = start() && send(ICM20602_DEV_ADDR << 1) && send(reg) && send(value);
    stop();
    return ok && budget();
}
static void begin(void) {
    began = clock_us();
    bad = false;
}
bool icm20602_checked_init(uint32_t (*micros)(void)) {
    clock_us = micros;
    if (!micros)
        return false;
    gpio_init(ICM20602_SCL_PIN, GPO, 1, GPO_OPEN_DTAIN);
    gpio_init(ICM20602_SDA_PIN, GPO, 1, GPO_OPEN_DTAIN);
    begin();
    uint8_t id;
    if (!read_regs(ICM20602_WHO_AM_I, &id, 1) || id != 0x12)
        return false;
    begin();
    if (!write_reg(ICM20602_PWR_MGMT_1, 0x80))
        return false;
    system_delay_ms(100);
    const uint8_t config[][2] = {
        {ICM20602_PWR_MGMT_1, 1}, {ICM20602_PWR_MGMT_2, 0},
        {ICM20602_CONFIG, 3}, {ICM20602_GYRO_CONFIG, 0x18},
        {ICM20602_ACCEL_CONFIG, 0x10}, {ICM20602_SMPLRT_DIV, 0},
        {ICM20602_INT_ENABLE, 1}
    };
    for (unsigned i = 0; i < sizeof config / sizeof config[0]; i++) {
        uint8_t v;
        begin();
        if (!write_reg(config[i][0], config[i][1]))
            return false;
        begin();
        if (!read_regs(config[i][0], &v, 1) || v != config[i][1])
            return false;
    }
    return true;
}
static int16_t signed_word(const uint8_t *p) {
    return (int16_t)(((uint16_t)p[0] << 8) | p[1]);
}
int icm20602_checked_sample(bool full, float *z, float *norm, float *accel) {
    uint8_t ready, b[14] = {0};
    begin();
    if (!read_regs(ICM20602_INT_STATUS, &ready, 1))
        return -1;
    if (!(ready & 1))
        return 0;
    if (!read_regs(full ? ICM20602_ACCEL_XOUT_H : ICM20602_GYRO_ZOUT_H, b, full ? 14 : 2))
        return -1;
    const float scale = .0174532925199433f / 16.4f;
    int16_t raw = signed_word(b + (full ? 12 : 0));
    if (raw == INT16_MIN || raw == INT16_MAX)
        return -1;
    *z = raw * scale;
    *norm = fabsf(*z);
    *accel = 1;
    if (full) {
        float gx = signed_word(b + 8) * scale, gy = signed_word(b + 10) * scale;
        float ax = signed_word(b) / 4096.0f, ay = signed_word(b + 2) / 4096.0f,
              az = signed_word(b + 4) / 4096.0f;
        *norm = sqrtf(gx * gx + gy * gy + (*z) * (*z));
        *accel = sqrtf(ax * ax + ay * ay + az * az);
    }
    return budget() ? 1 : -1;
}
