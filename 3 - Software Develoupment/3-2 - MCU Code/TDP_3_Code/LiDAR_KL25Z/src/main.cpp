#include "mbed.h"
#include <cstring>
#include <cstdio>

// ── Serial ────────────────────────────────────────────────────────────────────
static BufferedSerial pc(USBTX, USBRX, 115200);

static void print_line(const char *s) {
    pc.write(s, strlen(s));
}

// ── LSM6DSO constants ─────────────────────────────────────────────────────────
// SDO/SA0 pulled LOW via R1 (10 kΩ to GND) → I2C address 0x6A
#define LSM6DSO_ADDR        (0x6A << 1)   // 8-bit form for mbed I2C API

// Register map (LSM6DSO datasheet)
#define REG_WHO_AM_I        0x0F   // Should return 0x6C
#define REG_CTRL1_XL        0x10   // Accelerometer ODR / full-scale
#define REG_CTRL2_G         0x11   // Gyroscope ODR / full-scale
#define REG_STATUS          0x1E   // Data-ready flags
#define REG_OUTX_L_G        0x22   // Gyroscope X low byte  (6 bytes: XL XH YL YH ZL ZH)
#define REG_OUTX_L_A        0x28   // Accelerometer X low byte (6 bytes)

// CTRL1_XL: ODR_XL = 104 Hz (0100), FS_XL = ±2 g (00), LPF2 disabled
#define CTRL1_XL_104HZ_2G   0x40

// CTRL2_G:  ODR_G  = 104 Hz (0100), FS_G = 250 dps (00)
#define CTRL2_G_104HZ_250   0x40

// Sensitivity constants (from datasheet Table 2)
#define SENS_XL_2G          0.061f   // mg / LSB  → divide by 1000 for g
#define SENS_G_250DPS       8.75f    // mdps / LSB → divide by 1000 for dps

// ── I2C bus ───────────────────────────────────────────────────────────────────
// FRDM-KL25Z:
//   PTE25 = SDA  →  J1 pin 7 (I2C_DATA)   [J2 header pin 12 on the KL25Z]
//   PTE24 = SCL  →  J1 pin 6 (I2C_CLK)    [J2 header pin 11 on the KL25Z]
//   3.3 V        →  J1 pin 3 (+3V3)        [J9 header pin 8  on the KL25Z]
//   GND          →  J1 pin 5 (GND)         [J9 header pin 12 on the KL25Z]
static I2C i2c(PTC2, PTC1);   // (SDA, SCL)

// ── Low-level register helpers ────────────────────────────────────────────────
static bool reg_write(uint8_t reg, uint8_t value)
{
    char buf[2] = { (char)reg, (char)value };
    return i2c.write(LSM6DSO_ADDR, buf, 2) == 0;
}

static bool reg_read(uint8_t reg, uint8_t *dst, int len)
{
    char r = (char)reg;
    if (i2c.write(LSM6DSO_ADDR, &r, 1, true) != 0) {
        return false;
    }
    return i2c.read(LSM6DSO_ADDR, (char *)dst, len) == 0;
}

// ── Sensor init ───────────────────────────────────────────────────────────────
static bool lsm6dso_init(void)
{
    uint8_t id = 0;
    if (!reg_read(REG_WHO_AM_I, &id, 1) || id != 0x6C) {
        return false;   // wrong device or bus fault
    }

    // Enable accelerometer: 104 Hz, ±2 g
    if (!reg_write(REG_CTRL1_XL, CTRL1_XL_104HZ_2G)) return false;

    // Enable gyroscope: 104 Hz, 250 dps
    if (!reg_write(REG_CTRL2_G,  CTRL2_G_104HZ_250))  return false;

    return true;
}

// ── Data structs ──────────────────────────────────────────────────────────────
struct Vec3f { float x, y, z; };

static bool read_accel(Vec3f &out)
{
    uint8_t raw[6];
    if (!reg_read(REG_OUTX_L_A, raw, 6)) return false;

    int16_t rx = (int16_t)((raw[1] << 8) | raw[0]);
    int16_t ry = (int16_t)((raw[3] << 8) | raw[2]);
    int16_t rz = (int16_t)((raw[5] << 8) | raw[4]);

    out.x = rx * SENS_XL_2G / 1000.0f;   // convert mg → g
    out.y = ry * SENS_XL_2G / 1000.0f;
    out.z = rz * SENS_XL_2G / 1000.0f;
    return true;
}

static bool read_gyro(Vec3f &out)
{
    uint8_t raw[6];
    if (!reg_read(REG_OUTX_L_G, raw, 6)) return false;

    int16_t rx = (int16_t)((raw[1] << 8) | raw[0]);
    int16_t ry = (int16_t)((raw[3] << 8) | raw[2]);
    int16_t rz = (int16_t)((raw[5] << 8) | raw[4]);

    out.x = rx * SENS_G_250DPS / 1000.0f;  // convert mdps → dps
    out.y = ry * SENS_G_250DPS / 1000.0f;
    out.z = rz * SENS_G_250DPS / 1000.0f;
    return true;
}

// ── Main ──────────────────────────────────────────────────────────────────────
int main(void)
{
    char msg[128];

    i2c.frequency(400000);   // 400 kHz fast-mode (matches R3 pull-up on schematic)

    print_line("\r\n=== LSM6DSOQTR Demo ===\r\n");

    if (!lsm6dso_init()) {
        print_line("ERROR: LSM6DSO not found. Check I2C wiring and pull-ups.\r\n");
        while (true) { ThisThread::sleep_for(1s); }
    }

    print_line("LSM6DSO initialised OK (WHO_AM_I = 0x6C)\r\n\r\n");

    // NOTE: The LSM6DSOQTR measures ACCELERATION (g) and ANGULAR RATE (dps).
    // It is NOT a distance sensor.  If you need distance, use the Time-of-Flight
    // sensor enabled by the EN_ToF line visible on connector J1.

    while (true) {
        Vec3f accel, gyro;

        if (read_accel(accel)) {
            snprintf(msg, sizeof(msg),
                     "Accel [g]   X=%+7.4f  Y=%+7.4f  Z=%+7.4f\r\n",
                     accel.x, accel.y, accel.z);
            print_line(msg);
        } else {
            print_line("Accel read failed\r\n");
        }

        if (read_gyro(gyro)) {
            snprintf(msg, sizeof(msg),
                     "Gyro  [dps] X=%+8.3f  Y=%+8.3f  Z=%+8.3f\r\n",
                     gyro.x, gyro.y, gyro.z);
            print_line(msg);
        } else {
            print_line("Gyro read failed\r\n");
        }

        print_line("──────────────────────────────────────\r\n");
        ThisThread::sleep_for(200ms);
    }
}