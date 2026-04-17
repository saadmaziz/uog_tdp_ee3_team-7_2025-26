#include "mbed.h"
#include <cstdio>
#include <cstring>

I2C i2c(PB_7, PB_6);
static BufferedSerial pc(USBTX, USBRX, 115200);

static void print_line(const char *s) {
    pc.write(s, strlen(s));
}

// ── TMF8806 register addresses ───────────────────────────────────────────────
static constexpr uint8_t TMF_ADDR       = 0x41 << 1;
static constexpr uint8_t R_APPID        = 0x00;
static constexpr uint8_t R_CMD_STAT     = 0x08;
static constexpr uint8_t R_MEAS_CFG     = 0x10;
static constexpr uint8_t R_RESULT       = 0x1C;
static constexpr uint8_t APPID_BL       = 0x80;
static constexpr uint8_t APPID_MEAS     = 0x03;
static constexpr uint8_t CMD_LOAD_APP   = 0x14;
static constexpr uint8_t CMD_MEASURE    = 0x02;
static constexpr uint8_t STAT_OK        = 0x0F;

// ── I2C helpers ──────────────────────────────────────────────────────────────
static void write_reg(uint8_t reg, uint8_t val) {
    char buf[2] = { (char)reg, (char)val };
    i2c.write(TMF_ADDR, buf, 2);
}

static uint8_t read_reg(uint8_t reg) {
    char r = (char)reg, v = 0;
    i2c.write(TMF_ADDR, &r, 1, true);
    i2c.read(TMF_ADDR, &v, 1);
    return (uint8_t)v;
}

static void read_block(uint8_t reg, uint8_t *dst, int len) {
    char r = (char)reg;
    i2c.write(TMF_ADDR, &r, 1, true);
    i2c.read(TMF_ADDR, (char*)dst, len);
}

// ── Sensor init ──────────────────────────────────────────────────────────────
static bool tmf_init() {
    ThisThread::sleep_for(2ms);
    uint8_t appid = read_reg(R_APPID);

    if (appid == APPID_MEAS) return true;
    if (appid != APPID_BL)   return false;

    write_reg(R_CMD_STAT, CMD_LOAD_APP);
    ThisThread::sleep_for(5ms);
    return read_reg(R_APPID) == APPID_MEAS;
}

static bool tmf_start() {
    // 5 m mode: spad_map=6, kilo_iter=4000, period=300 ms
    const uint8_t cfg[] = {
        R_MEAS_CFG,
        0x00, 0x00, 0x00, 0x00,   // capture_mode, algo_state, gpio, dax
        0x06,                      // spad_map_id  (5 m narrow-beam)
        0xA0, 0x0F,                // kilo_iter = 4000 (little-endian)
        0x2C, 0x01,                // period_ms  = 300 (little-endian)
    };
    i2c.write(TMF_ADDR, (const char*)cfg, sizeof(cfg));
    write_reg(R_CMD_STAT, CMD_MEASURE);
    ThisThread::sleep_for(5ms);
    return read_reg(R_CMD_STAT) == STAT_OK;
}

// ── main ─────────────────────────────────────────────────────────────────────
int main() {
    i2c.frequency(400000);

    print_line("TMF8806 -- Nucleo L432KC\r\n");

    while (!tmf_init()) {
        print_line("[ERROR] Init failed, retrying...\r\n");
        ThisThread::sleep_for(500ms);
    }
    print_line("[OK] Sensor ready\r\n");

    if (!tmf_start()) {
        print_line("[ERROR] Start measurement failed\r\n");
        while (true) ThisThread::sleep_for(1000ms);
    }
    print_line("[OK] Measuring -- 5 m mode, 200/min\r\n\r\n");

    char line[64];
    while (true) {
        ThisThread::sleep_for(300ms);

        uint8_t buf[8] = {};
        read_block(R_RESULT, buf, 8);

        uint16_t dist_mm   = (uint16_t)buf[2] | ((uint16_t)buf[3] << 8);
        uint8_t  conf      = buf[4];
        uint8_t  result_n  = buf[0];

        if (conf >= 30 && dist_mm > 0 && dist_mm <= 5000) {
            int n = snprintf(line, sizeof(line),
                             "[%03u] %4u mm  conf=%u\r\n",
                             result_n, dist_mm, conf);
            pc.write(line, n);
        } else {
            int n = snprintf(line, sizeof(line),
                             "[%03u] invalid  dist=%u mm  conf=%u\r\n",
                             result_n, dist_mm, conf);
            pc.write(line, n);
        }
    }
}