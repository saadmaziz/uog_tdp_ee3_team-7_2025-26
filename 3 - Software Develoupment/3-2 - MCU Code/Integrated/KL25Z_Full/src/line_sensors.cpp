// line_sensors.cpp
#ifndef LINESENSORS_CPP
#define LINESENSORS_CPP

#include <cmath>
#include "header.hpp"

Ticker sampleLine;
volatile bool updateLine = false;

void initializeLine() {
    updateLine  = false;
    initTurn90  = false;
    lineLost    = false;
    e_prev      = 0.0f;
    integralSum = 0.0f;
    PID_Out     = 0.0f;

    // Ticker fires every 2 ms → 500 Hz sample rate.
    sampleLine.attach(&trigPollLine, 500us);

    // 90-degree turn triggers on far sensors (rising edge = black line detected)
    L2L.fall(&isrL90);
    R2L.fall(&isrR90);

    dbgPrint("[LINE] Line sensor init complete\r\n");
}

// Called by Ticker ISR – sets flag for main-loop polling (ISR-safe)
void trigPollLine() {
    // BUG FIX: function was declared as trigPoll() in the old header.hpp.
    //          Name is now consistent: trigPollLine() everywhere.
    updateLine = true;
}

// ---- 90-degree turn interrupt flags -------------------------
volatile bool initTurn90 = false;
volatile int  dir90      = 0;

void isrL90() {
    dir90      = -1;   // Left 90-degree turn requested
    initTurn90 = true;
}

void isrR90() {
    dir90      = 1;    // Right 90-degree turn requested
    initTurn90 = true;
}

// ---- Sensor history ring-buffer (10 samples per sensor) -----
int sensorVals[3][10] = {
    {0,0,0,0,0,0,0,0,0,0},  // [0] = L1L (left-near)
    {0,0,0,0,0,0,0,0,0,0},  // [1] = C0L (centre)
    {0,0,0,0,0,0,0,0,0,0}   // [2] = R1L (right-near)
};

void pollLine() {
    // Shift history left (oldest sample drops off, newest added at [9])
    for (int i = 0; i < 9; i++) {
        sensorVals[0][i] = sensorVals[0][i + 1];
        sensorVals[1][i] = sensorVals[1][i + 1];
        sensorVals[2][i] = sensorVals[2][i + 1];
    }
    sensorVals[0][9] = !L1L.read();
    sensorVals[1][9] = !C0L.read();
    sensorVals[2][9] = !R1L.read();
}

// ---- PID controller variables --------------------------------
float k_p = 10.0f;
float k_i = 0.0f;
float k_d = 1.0f;
float e_prev      = 0.0f;
bool  lineLost    = false;
float integralSum = 0.0f;
float PID_Out     = 0.0f;

void PIDCalc() {
    pollLine();

    float L = 0.0f, C = 0.0f, R = 0.0f;

    // Accumulate and average the 10-sample history
    for (int i = 0; i < 10; i++) {
        L += sensorVals[0][i];
        C += sensorVals[1][i];
        R += sensorVals[2][i];
    }
    L /= 10.0f;
    C /= 10.0f;
    R /= 10.0f;

    // Prevent divide-by-zero when no sensor sees the line
    if (L + C + R < 0.001f) {
        if (!lineLost) {
            lineLost = true;
            dbgPrint("[LINE] Line lost! L=%.2f C=%.2f R=%.2f\r\n", L, C, R);
        }
        return;   // PID_Out keeps its last value until state transitions
    }
    lineLost = false;

    // Weighted centroid error: -1 (hard left) … 0 (centre) … +1 (hard right)
    float e      = ((-1.0f * L) + (0.0f * C) + (1.0f * R)) / (L + C + R);
    float delta_e = e - e_prev;

    // Integral with anti-windup:
    //  – skip accumulation if either motor is near current-limit (saturated)
    //  – skip if integral is already pushing the wrong way (clamped at ±1)
    bool motorSaturated = (SenseL.read() > 0.88f || SenseR.read() > 0.88f);
    bool intWindup      = (integralSum >=  1.0f && e > 0.0f) ||
                          (integralSum <= -1.0f && e < 0.0f);
    bool lowDynamics    = (std::abs(delta_e) < 0.0001f);

    if (!(motorSaturated && lowDynamics) && !intWindup) {
        integralSum += e * 0.002f;   // dt = 2 ms (must match Ticker period)
    }

    // Full PID output
    PID_Out = (k_p * e)
            + (k_i * integralSum)
            + (k_d * (delta_e / 0.002f));

    // Clamp to [-1, +1]
    if      (PID_Out >  1.0f) PID_Out =  1.0f;
    else if (PID_Out < -1.0f) PID_Out = -1.0f;

    e_prev = e;
}

#endif // LINESENSORS_CPP