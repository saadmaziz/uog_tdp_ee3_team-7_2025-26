// traffic_lights.cpp
#ifndef TLIGHTS_CPP
#define TLIGHTS_CPP

#include "header.hpp"

Ticker sampleTLights;

// ---- VEML3328SL colour sensor over I2C ----------------------
// 7-bit device address: 0x10
// Mbed I2C API uses 8-bit addresses: write = 0x20, read = 0x21
#define VEML3328_ADDR_W  0x20
#define VEML3328_ADDR_R  0x21   // BUG FIX: old code passed 0x20 (write addr) to i2c.read()
                                //          which incorrectly addressed the sensor for reads.

#define REG_CONFIG  0x00
#define REG_R_DATA  0x05
#define REG_G_DATA  0x06

// Higher value = more smoothing, more lag (tune for lighting conditions)
int roundingFactor = 30;

bool updateTLights = false;

void initVEML3328() {
    updateTLights = false;
    red_light     = false;
    green_light   = false;

    // Wake sensor: write 0x0000 to CONFIG register (all defaults, active mode)
    char config_cmd[3] = { REG_CONFIG, 0x00, 0x00 };
    int  ack = i2c.write(VEML3328_ADDR_W, config_cmd, 3);
    if (ack != 0) {
        dbgPrint("[TLIGHT] VEML3328 init FAILED (no I2C ACK)\r\n");
    } else {
        dbgPrint("[TLIGHT] VEML3328 init OK\r\n");
    }

    sampleTLights.attach(&trigPollTLights, 100ms);
}

void trigPollTLights() {
    updateTLights = true;
}

// ---- Read one 16-bit little-endian register -----------------
uint16_t read_regVEML3328(uint8_t reg) {
    char cmd[1]  = { (char)reg };
    char data[2] = { 0, 0 };

    // Write register pointer (repeated-start: no STOP before the read)
    i2c.write(VEML3328_ADDR_W, cmd, 1, true);

    // BUG FIX: was i2c.read(VEML3328_ADDR, …) with the write address 0x20.
    //          Must use the read address 0x21 so the bus R/W bit is correct.
    i2c.read(VEML3328_ADDR_R, data, 2);

    return (uint16_t)((data[1] << 8) | data[0]);  // Little-endian
}

uint16_t r_raw = 0;
uint16_t g_raw = 0;

void readRG() {
    r_raw = read_regVEML3328(REG_R_DATA);
    g_raw = read_regVEML3328(REG_G_DATA);
}

float r_norm = 0.0f;
float g_norm = 0.0f;

void normRG() {
    // Quantise to 1/roundingFactor steps to suppress noise
    r_norm = roundf((r_raw / 65535.0f) * roundingFactor) / (float)roundingFactor;
    g_norm = roundf((g_raw / 65535.0f) * roundingFactor) / (float)roundingFactor;
}

bool red_light   = false;
bool green_light = false;

void TLightsMain() {
    if (!updateTLights) return;
    updateTLights = false;

    readRG();
    normRG();

    dbgPrint("[TLIGHT] r_raw=%u g_raw=%u r_norm=%.2f g_norm=%.2f\r\n",
             r_raw, g_raw, r_norm, g_norm);

    // BUG FIX: old code unconditionally overwrote roverState with FOLLOWING or
    //          TRAFFICSTOP even during OBSTACLEAVOID / MANUAL.  Guard added so
    //          traffic-light logic only acts when the rover is under line-follow
    //          or traffic-stop supervision.
    bool lineFollowingSupervised = (roverState == RoverState::FOLLOWING  ||
                                    roverState == RoverState::TRAFFICSTOP);

    if (r_norm > 0.5f && g_norm < 0.5f) {
        red_light   = true;
        green_light = false;
        if (lineFollowingSupervised && roverState != RoverState::TRAFFICSTOP) {
            roverState = RoverState::TRAFFICSTOP;
            dbgPrint("[TLIGHT] RED  – rover stopped\r\n");
        }
    } else if (g_norm > 0.5f && r_norm < 0.5f) {
        red_light   = false;
        green_light = true;
        if (roverState == RoverState::TRAFFICSTOP) {
            roverState = RoverState::FOLLOWING;
            dbgPrint("[TLIGHT] GREEN – rover resuming\r\n");
        }
    } else {
        // Ambiguous reading (yellow, sensor error, or neither)
        red_light   = false;
        green_light = false;
        if (roverState == RoverState::TRAFFICSTOP) {
            // Default: cautiously remain stopped until a clear green is seen.
            dbgPrint("[TLIGHT] AMBIGUOUS – staying stopped\r\n");
        }
    }
}

#endif // TLIGHTS_CPP