// motors_control.cpp
#ifndef MOTOR_CONTROL_CPP
#define MOTOR_CONTROL_CPP

#include "header.hpp"

Ticker control;

// ---- Line-following base (longitudinal) speed ---------------
// BUG FIX: baseSpeed was declared extern in the old header but never defined.
//          Defined here as a named constant; extern-declared in header.hpp.
const float BASE_SPEED = 0.40f;  // 40 % of max torque for straight-ahead travel

// ---- PI current (torque) loop parameters --------------------
float kp_torque = 2.5f;
float ki_torque = 0.1f;
float integralL = 0.0f;
float integralR = 0.0f;

// ---- Motor / L298 specifications ----------------------------
const float MAX_CURRENT_A = 1.5f;  // L298 per-channel current limit
float motorF = 0.001f;             // PWM period (1 kHz)

volatile bool updateMotors = false;

// ---- Measured quantities ------------------------------------
float measured_iL   = 0.0f;
float measured_iR   = 0.0f;
float actualTorqueL = 0.0f;
float actualTorqueR = 0.0f;

// ---- Initialisation -----------------------------------------
void initializeMotors() {
    integralL = 0.0f;
    integralR = 0.0f;
    EnL.period(motorF);
    EnR.period(motorF);
    EnL.write(0.0f);
    EnR.write(0.0f);
    control.attach(&trigControl, 20ms);  // 50 Hz motor-update loop
    dbgPrint("[MTR] Motor init complete (PWM %.0f Hz, BASE_SPEED=%.2f)\r\n",
             1.0f / motorF, BASE_SPEED);
}

void trigControl() {
    updateMotors = true;
}

// ---- Current and torque measurement -------------------------
void calcCurrentAndTorque() {
    // Two motors wired in parallel on each L298 bridge channel → divide by 2
    // Sense-resistor values: 0.45 Ω (left), 0.44 Ω (right)
    // ADC full-scale = 3.3 V; sense gain 
    measured_iL = (SenseL.read() / (0.45f * 4.909f)) / 2.0f;
    measured_iR = (SenseR.read() / (0.44f * 4.909f))/ 2.0f;

    // Torque estimate: T [g·cm] = (I − I_no_load) × Kt
    // Kt ≈ 5166 g·cm/A from datasheet stall/no-load points
    actualTorqueL = (measured_iL > 0.113f) ? (measured_iL - 0.113f) * 5166.0f : 0.0f;
    actualTorqueR = (measured_iR > 0.113f) ? (measured_iR - 0.113f) * 5166.0f : 0.0f;
}

// ---- Main motor output function -----------------------------
// lateral     : +1.0 = full right,  −1.0 = full left
// longitudinal: +1.0 = full forward, −1.0 = full reverse
void motorsOutput(float lateral, float longitudinal) {

    // 1. Skid-steer mixer
    float reqL = longitudinal + lateral;
    float reqR = longitudinal - lateral;

    // 2. Normalise to [−1, +1] while preserving the steering ratio
    float maxReq = fabs(reqL) > fabs(reqR) ? fabs(reqL) : fabs(reqR);
    if (maxReq > 1.0f) {
        reqL /= maxReq;
        reqR /= maxReq;
    }

    // 3. Set H-Bridge direction pins
    OutL1 = (reqL >= 0.0f) ? 1 : 0;
    OutL2 = (reqL >= 0.0f) ? 0 : 1;
    OutR1 = (reqR >= 0.0f) ? 1 : 0;
    OutR2 = (reqR >= 0.0f) ? 0 : 1;

    // 4. Sample current sensors
    calcCurrentAndTorque();

    // 5. Target current proportional to requested magnitude
    float target_iL = fabs(reqL) * MAX_CURRENT_A;
    float target_iR = fabs(reqR) * MAX_CURRENT_A;

    // 6. PI error terms
    float errorL = target_iL - measured_iL;
    float errorR = target_iR - measured_iR;

    // 7. Conditional integral accumulation (anti-windup before limiting)
    //    Only accumulate when not already at a PWM rail – prevents integrator
    //    charging while the output is saturated (classic back-calculation lite).
    integralL += errorL;
    integralR += errorR;

    // 8. PI output
    float pwm_L = (kp_torque * errorL) + (ki_torque * integralL);
    float pwm_R = (kp_torque * errorR) + (ki_torque * integralR);

    // 9. Clamp and symmetric anti-windup
    if (pwm_L > 0.5f) {
        pwm_L = 0.5f;
        integralL -= errorL;   // do not accumulate while at upper rail
        dbgPrint("[MTR] Left  PWM upper-rail sat (iL=%.3fA)\r\n", measured_iL);
    } else if (pwm_L < 0.0f) {
        pwm_L = 0.0f;
        integralL -= errorL;   // do not accumulate while at lower rail
    }

    if (pwm_R > 0.5f) {
        pwm_R = 0.5f;
        integralR -= errorR;
        dbgPrint("[MTR] Right PWM upper-rail sat (iR=%.3fA)\r\n", measured_iR);
    } else if (pwm_R < 0.0f) {
        pwm_R = 0.0f;
        integralR -= errorR;
    }

    // 10. Write PWM
    EnL.write(pwm_L);
    EnR.write(pwm_R);
}

// ---- Utility: reset PI integrators (call on state transitions) ----
static void resetMotorIntegrals() {
    integralL = 0.0f;
    integralR = 0.0f;
}

// ---- 90-degree pivot turn -----------------------------------
// Returns true once any forward sensor sees the line again (turn complete).
bool turn90(int direction) {
    if (direction == 1) {
        motorsOutput( 0.5f, 0.0f);   // Pivot right (L fwd, R rev)
    } else {
        motorsOutput(-0.5f, 0.0f);   // Pivot left  (L rev, R fwd)
    }
    pollLine();
    return (bool)(L1L.read() || C0L.read() || R1L.read());
}

// ---- Lost-line search manoeuvre -----------------------------
// Pivots in the direction the line was last seen; reverses if centred.
// Returns true once a forward sensor detects the line.
bool findLine() {
    const float searchTorque = 0.4f;

    if (e_prev < -0.05f) {
        motorsOutput( searchTorque, 0.0f);   // Last seen left → pivot right
    } else if (e_prev > 0.05f) {
        motorsOutput(-searchTorque, 0.0f);   // Last seen right → pivot left
    } else {
        motorsOutput( 0.0f, -searchTorque);  // Centred: reverse to find line
    }

    pollLine();
    return (bool)(L1L.read() || C0L.read() || R1L.read());
}

#endif // MOTOR_CONTROL_CPP