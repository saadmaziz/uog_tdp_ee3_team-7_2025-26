// pins.cpp
#ifndef PINS_CPP
#define PINS_CPP

#include "header.hpp"

// =============================================================
// -------------------- PIN DEFINITIONS ------------------------ 
// =============================================================

// ---- Left motor (H-Bridge A) --------------------------------
PwmOut     EnL(PTE20);           // ENA  – PWM enable / speed
DigitalOut OutL1(PTE22);         // IN1A – direction
DigitalOut OutL2(PTE23);         // IN2A – direction
AnalogIn   SenseL(PTB0);         // Current sense: IL = SenseL/(R_S_L * 1.0269)

// ---- Right motor (H-Bridge B) -------------------------------
PwmOut     EnR(PTE21);           // ENB  – PWM enable / speed
DigitalOut OutR1(PTE29);         // IN1B – direction
DigitalOut OutR2(PTE30);         // IN2B – direction
AnalogIn   SenseR(PTB1);         // Current sense: IR = SenseR/(R_S_R * 1.0269)

// ---- IR line sensors ----------------------------------------
DigitalIn  L1L(PTB8);            // Left-near
DigitalIn  C0L(PTB9);            // Centre
DigitalIn  R1L(PTB10);           // Right-near
InterruptIn L2L(PTD6);           // Left-far  (90-degree ISR)
InterruptIn R2L(PTD7);           // Right-far (90-degree ISR)

// ---- I2C (VEML3328 colour sensor) ---------------------------
I2C i2c(PTC9, PTC8);             // SDA, SCL

// ---- ESP32 / Joystick analogue inputs -----------------------
AnalogIn    JS_Turning(PTB2);    // Lateral  axis  (0.0=full-left, 0.5=centre, 1.0=full-right)
AnalogIn    JS_Speed(PTB3);      // Longitudinal   (0.0=full-rev,  0.5=stop,   1.0=full-fwd)

// BUG FIX: was DigitalIn JS_CycleMode – changed to InterruptIn so that
//          joystick.cpp can attach a rising-edge ISR with JS_CycleMode.rise().
InterruptIn JS_CycleMode(PTD2);  // Mode-cycle push-button from ESP32

// ---- UART to ESP32 (shared with OpenSDA on KL25Z) -----------
// PTA2 = UART0_TX, PTA1 = UART0_RX.  Same physical UART as OpenSDA USB-serial,
// so debug messages appear on the PC terminal when the USB cable is connected.
BufferedSerial espUART(PTA2, PTA1, 115200);

// ---- Obstacle-avoidance control lines -----------------------
/*  ObsAv_Active  Control1  Control2  | Action
 *       0           X         X      | Inactive (ignored)
 *       1           0         0      | Forward at base speed
 *       1           0         1      | Skid Right
 *       1           1         0      | Skid Left
 *       1           1         1      | Reverse at base speed
 */
InterruptIn ObsAv_Active(PTD1);      // Rise = obstacle detected, Fall = obstacle cleared
DigitalIn   ObsAvoidControl1(PTD3);
DigitalIn   ObsAvoidControl2(PTD0);

#endif // PINS_CPP