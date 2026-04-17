// header.hpp
#ifndef HEADER_HPP
#define HEADER_HPP

#include "mbed.h"
#include <cstdarg>
#include <cstdio>
#include <cstring>

// =============================================================
//  HARDWARE PIN DECLARATIONS  (defined in pins.cpp)
// =============================================================

// Left motor H-Bridge (Bridge A)
extern PwmOut      EnL;
extern DigitalOut  OutL1;
extern DigitalOut  OutL2;
extern AnalogIn    SenseL;

// Right motor H-Bridge (Bridge B)
extern PwmOut      EnR;
extern DigitalOut  OutR1;
extern DigitalOut  OutR2;
extern AnalogIn    SenseR;

// IR Line sensors
extern DigitalIn   L1L;   // Left-near
extern DigitalIn   C0L;   // Centre
extern DigitalIn   R1L;   // Right-near
extern InterruptIn L2L;   // Left-far  (90-deg trigger)
extern InterruptIn R2L;   // Right-far (90-deg trigger)

// I2C (VEML3328 colour sensor)
extern I2C i2c;

// ESP32 / Joystick interface
extern AnalogIn    JS_Turning;      // PTB2  – lateral  joystick axis
extern AnalogIn    JS_Speed;        // PTB3  – longitudinal joystick axis
extern InterruptIn JS_CycleMode;    // PTD2  – mode-cycle button (rising edge)

// Obstacle-avoidance digital inputs
extern InterruptIn ObsAv_Active;    // PTD1  – high = obstacle present
extern DigitalIn   ObsAvoidControl1;// PTD3
extern DigitalIn   ObsAvoidControl2;// PTD0

// Serial to ESP32 / PC OpenSDA  (PTA2=TX, PTA1=RX)
// NOTE: KL25Z OpenSDA shares UART0 (PTA1/PTA2) with this peripheral.
//       All debug messages route here, appearing on both the PC terminal
//       (via OpenSDA) and the ESP32 RX line when deployed.
extern BufferedSerial espUART;

// =============================================================
//  DEBUG HELPER  (defined in main.cpp)
// =============================================================
// Sends a printf-style message over espUART (≤127 chars per call).
void dbgPrint(const char* fmt, ...);

// =============================================================
//  ROVER STATE MACHINE
// =============================================================
enum class RoverState {
    FOLLOWING,      // Normal PID line-following
    TURNING90,      // Executing a 90-degree pivot turn
    SEARCHING,      // Line lost – scanning to recover
    TRAFFICSTOP,    // Stopped at red traffic light
    OBSTACLEAVOID,  // Slewing around an on-track obstacle
    MANUAL,         // Joystick control via ESP32 analogue inputs
};

extern volatile RoverState roverState;

// =============================================================
//  LINE SENSOR MODULE  (line_sensors.cpp)
// =============================================================
extern int           sensorVals[3][10];
extern volatile bool updateLine;
extern bool          lineLost;
extern float         k_p, k_i, k_d;
extern float         e_prev;
extern float         integralSum;
extern float         PID_Out;
extern volatile int  dir90;
extern volatile bool initTurn90;

void initializeLine();
void trigPollLine();  
void pollLine();
void PIDCalc();
void isrL90();
void isrR90();

// =============================================================
//  MOTOR CONTROL MODULE  (motors_control.cpp)
// =============================================================
extern float         motorF;
extern volatile bool updateMotors;
extern const float   BASE_SPEED;   

void initializeMotors();
void trigControl();
void motorsOutput(float lateral, float longitudinal);
bool turn90(int direction);
bool findLine();

// =============================================================
//  TRAFFIC LIGHT MODULE  (traffic_lights.cpp)
// =============================================================
extern bool updateTLights;
extern bool red_light;
extern bool green_light;

void initVEML3328();
void trigPollTLights();
void TLightsMain();

// =============================================================
//  OBSTACLE AVOIDANCE MODULE  (obstacle_avoid.cpp)
// =============================================================
void initObstacleAvoidance();
void trigObsAvoidance();
void trigObsAvoidanceEnd();   
void operateObstacleAvoidance();

// =============================================================
//  JOYSTICK / MANUAL MODULE  (joystick.cpp)
// =============================================================
void initJoystick();
void trigModeCycle();
void operateJoystick();

#endif // HEADER_HPP