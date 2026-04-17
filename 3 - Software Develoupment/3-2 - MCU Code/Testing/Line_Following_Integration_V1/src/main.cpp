#include "mbed.h"

// --- Pin Assignments ---
PwmOut LEDblue(LED_BLUE);
PwmOut LEDred(LED_RED);
PwmOut LEDgreen(LED_GREEN);

// Right motor I/O (L298N: ENA, IN1, IN2)
PwmOut rEn(PTE21);
DigitalOut rO1(PTE29);  // IN1
DigitalOut rO2(PTE30);  // IN2

// Left motor I/O (L298N: ENB, IN3, IN4)
PwmOut lEn(PTE20);
DigitalOut lO1(PTE22);  // IN3
DigitalOut lO2(PTE23);  // IN4

// Line sensors (High = On Line)
DigitalIn NlLine(PTB8);   // Left
DigitalIn CeLine(PTB9);   // Center
DigitalIn NrLine(PTB10);  // Right
InterruptIn Flline(PTD6); // Far Left
InterruptIn Frline(PTD7); // Far Right

// --- Manual Control Inputs ---
DigitalIn manualEnable(PTA2); // HIGH = Manual Mode
AnalogIn steerInput(PTB2);    // Steering:  0.0 (Left) → 0.5 (Centre) → 1.0 (Right)
AnalogIn throttleInput(PTB3); // Throttle:  0.0 (Reverse) → 0.5 (Centre) → 1.0 (Forward)

// --- Settings ---
float drivePower = 0.35f;
float turnPower  = 1.0f;
const float DEADBAND = 0.05f;

// --- Shared mode flag (volatile: written in ISR context indirectly) ---
volatile bool inManualMode = false;

// ============================================================
// Helpers
// ============================================================
void initialize() {
    lEn.period_ms(10);
    rEn.period_ms(10);
    LEDblue  = 1.0f;
    LEDred   = 1.0f;
    LEDgreen = 1.0f;
}

// mode: 0 = Fwd, 1 = Rev, 2 = Brake
void leftControl(int mode, float power) {
    if      (mode == 0) { lO1 = 1; lO2 = 0; }
    else if (mode == 1) { lO1 = 0; lO2 = 1; }
    else                { lO1 = 0; lO2 = 0; }
    lEn.write(power);
}

void rightControl(int mode, float power) {
    if      (mode == 0) { rO1 = 1; rO2 = 0; }
    else if (mode == 1) { rO1 = 0; rO2 = 1; }
    else                { rO1 = 0; rO2 = 0; }
    rEn.write(power);
}

// ============================================================
// Manual control — one update tick
// ============================================================
void runManual() {
    float nSteer    = (steerInput.read()    - 0.5f) * 2.0f;
    float nThrottle = (throttleInput.read() - 0.5f) * 2.0f;

    if (fabsf(nSteer)    < DEADBAND) nSteer    = 0.0f;
    if (fabsf(nThrottle) < DEADBAND) nThrottle = 0.0f;

    float leftSpeed  = nThrottle + nSteer;
    float rightSpeed = nThrottle - nSteer;

    // Clamp
    if (leftSpeed  >  1.0f) leftSpeed  =  1.0f;
    if (leftSpeed  < -1.0f) leftSpeed  = -1.0f;
    if (rightSpeed >  1.0f) rightSpeed =  1.0f;
    if (rightSpeed < -1.0f) rightSpeed = -1.0f;

    leftControl (leftSpeed  >= 0.0f ? 0 : 1, fabsf(leftSpeed));
    rightControl(rightSpeed >= 0.0f ? 0 : 1, fabsf(rightSpeed));

    // Cyan LED = manual mode
    LEDred = 1.0f; LEDgreen = 0.0f; LEDblue = 0.0f;
}

// ============================================================
// Main
// ============================================================
int main() {
    initialize();

    // --- Far-sensor ISRs (only act when in autonomous mode) ---
    Flline.rise([]() {
        if (inManualMode) return;   // <-- ignore in manual mode
        bool rejoined = false;
        while (!rejoined) {
            LEDred = 0.0f; LEDgreen = 0.0f; LEDblue = 0.0f; // White
            leftControl(1, 0.5f);
            rightControl(0, 0.5f);
            wait_us(1000);
            leftControl(0, 0);
            rightControl(0, 0);
            if (NlLine || CeLine || NrLine) rejoined = true;
        }
    });

    Frline.rise([]() {
        if (inManualMode) return;   // <-- ignore in manual mode
        bool rejoined = false;
        while (!rejoined) {
            LEDred = 0.0f; LEDgreen = 0.0f; LEDblue = 0.0f; // White
            leftControl(0, 0.5f);
            rightControl(1, 0.5f);
            wait_us(1000);
            leftControl(0, 0);
            rightControl(0, 0);
            if (NlLine || CeLine || NrLine) rejoined = true;
        }
    });

    while (true) {
        inManualMode = (manualEnable.read() == 1);

        // ── MANUAL MODE ──────────────────────────────────────
        if (inManualMode) {
            runManual();
            wait_us(10000); // 10 ms
            continue;
        }

        // ── AUTONOMOUS LINE-FOLLOWING MODE ───────────────────
        int left   = NlLine.read();
        int center = CeLine.read();
        int right  = NrLine.read();

        if (center == 1) {
            leftControl(0, drivePower);
            rightControl(0, drivePower);
            // Keep driving until a side sensor fires or mode changes
            while (!left && !right && !inManualMode) {
                wait_us(10000);
                left  = NlLine.read();
                right = NrLine.read();
                inManualMode = (manualEnable.read() == 1);
            }
            LEDgreen = 0.0f; LEDred = 1.0f; LEDblue = 1.0f; // Green
        }
        else if (right == 1) {
            // Drifted left → turn right
            leftControl(0, turnPower);
            rightControl(1, turnPower);
            LEDblue = 0.0f; LEDgreen = 1.0f; LEDred = 1.0f; // Blue
        }
        else if (left == 1) {
            // Drifted right → turn left
            leftControl(1, turnPower);
            rightControl(0, turnPower);
            LEDblue = 0.0f; LEDgreen = 1.0f; LEDred = 1.0f; // Blue
        }
        else {
            // Lost — slow search spin
            leftControl(0, 0.2f);
            rightControl(1, 0.2f);
            LEDred = 0.0f; LEDgreen = 1.0f; LEDblue = 1.0f; // Red
        }

        wait_us(10000); // 10 ms
    }
}