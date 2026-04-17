// main.cpp
#include "header.hpp"
#include <cstdarg>
#include <cstdio>
#include <cstring>

// =============================================================
//  GLOBAL STATE
// =============================================================
volatile RoverState roverState = RoverState::FOLLOWING;

// =============================================================
//  DEBUG HELPER
// =============================================================
// Routes all debug output to espUART (PTA1/PTA2).
// When the KL25Z USB cable is connected the OpenSDA interface
// bridges this same UART to the host PC terminal.
// When deployed with the ESP32, the ESP32 receives the messages.
//
// Max message length: 127 characters (increase buf[] if needed).
void dbgPrint(const char* fmt, ...) {
    char buf[128];
    va_list args;
    va_start(args, fmt);
    int len = vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    if (len > 0) {
        espUART.write(buf, (size_t)len);
    }
}

// =============================================================
//  MAIN
// =============================================================
int main() {
    // ---- Hardware / module initialisation -------------------
    initializeLine();
    initializeMotors();
    initVEML3328();
    initObstacleAvoidance();
    initJoystick();

    roverState = RoverState::FOLLOWING;
    dbgPrint("[MAIN] Rover boot complete – entering FOLLOWING\r\n");

    // ---- Settle-tick counter for 90-degree turns ------------
    static int turnSettleTicks = 0;
    const  int TURN_SETTLE_TICKS = 5;  // 5 × 20 ms = 100 ms coast-past period

    // ==========================================================
    //  MAIN LOOP
    // ==========================================================
    while (true) {

        // ---- Run traffic-light sensor every loop iteration --
        // Must execute regardless of rover state so the sensor
        // keeps updating; TLightsMain() guards its own state transitions.
        TLightsMain();

        // ---- FOLLOWING: check for asynchronous state triggers ----
        if (roverState == RoverState::FOLLOWING) {
            if (initTurn90) {
                initTurn90      = false;
                lineLost        = false;
                turnSettleTicks = 0;
                // Reset motor integrals so the new turn starts cleanly
                // (integralL / integralR are file-scope in motors_control.cpp;
                //  reset here by commanding zero before entering turn state)
                motorsOutput(0.0f, 0.0f);
                roverState = RoverState::TURNING90;
                dbgPrint("[MAIN] FOLLOWING → TURNING90 (dir=%d)\r\n", dir90);

            } else if (lineLost) {
                lineLost   = false;
                roverState = RoverState::SEARCHING;
                dbgPrint("[MAIN] FOLLOWING → SEARCHING\r\n");
            }
        }

        // ---- STATE EXECUTION --------------------------------
        switch (roverState) {

            // --------------------------------------------------
            case RoverState::FOLLOWING:
                if (updateLine) {
                    updateLine = false;
                    PIDCalc();
                }
                if (updateMotors) {
                    updateMotors = false;
                    // BUG FIX: old code called motorsOutput(PID_Out) with a single
                    //          argument – the function requires two (lateral, longitudinal).
                    //          BASE_SPEED provides the forward drive; PID_Out steers.
                    motorsOutput(PID_Out, BASE_SPEED);
                    dbgPrint("[FOLLOW] PID=%.3f spd=%.2f L=%.3f R=%.3f\r\n",
                             PID_Out, BASE_SPEED);
                }
                break;

            // --------------------------------------------------
            case RoverState::TURNING90:
                if (updateMotors) {
                    updateMotors = false;
                    turnSettleTicks++;

                    if (turnSettleTicks > TURN_SETTLE_TICKS) {
                        // Settle period over – poll sensors for turn completion
                        bool done = turn90(dir90);
                        dbgPrint("[TURN90] tick=%d done=%d\r\n",
                                 turnSettleTicks, (int)done);
                        if (done) {
                            lineLost    = false;
                            e_prev      = 0.0f;
                            integralSum = 0.0f;
                            roverState  = RoverState::FOLLOWING;
                            dbgPrint("[MAIN] TURNING90 → FOLLOWING\r\n");
                        }
                    } else {
                        // Still in settle phase – keep turning without checking
                        turn90(dir90);
                        dbgPrint("[TURN90] settling tick=%d\r\n", turnSettleTicks);
                    }
                }
                break;

            // --------------------------------------------------
            case RoverState::SEARCHING:
                if (updateMotors) {
                    updateMotors = false;
                    bool found = findLine();
                    dbgPrint("[SEARCH] found=%d e_prev=%.3f\r\n",
                             (int)found, e_prev);
                    if (found) {
                        e_prev      = 0.0f;
                        integralSum = 0.0f;
                        roverState  = RoverState::FOLLOWING;
                        dbgPrint("[MAIN] SEARCHING → FOLLOWING\r\n");
                    }
                }
                break;

            case RoverState::TRAFFICSTOP:
                if (updateMotors) {
                    updateMotors = false;
                    // Hold the rover stationary while red light persists.
                    // TLightsMain() (called above the switch) will transition
                    // back to FOLLOWING when a green light is detected.
                    motorsOutput(0.0f, 0.0f);
                    dbgPrint("[STOP] Holding – red=%d green=%d\r\n",
                             (int)red_light, (int)green_light);
                }
                break;

            // --------------------------------------------------
            case RoverState::OBSTACLEAVOID:
                if (updateMotors) {
                    updateMotors = false;
                    // ESP32 drives Control1/Control2 to steer around obstacle.
                    // trigObsAvoidanceEnd() (fall ISR on ObsAv_Active) exits this state.
                    operateObstacleAvoidance();
                }
                break;

            // --------------------------------------------------
            case RoverState::MANUAL:
                if (updateMotors) {
                    updateMotors = false;
                    // Joystick values arrive as analogue inputs from the ESP32.
                    // trigModeCycle() (JS_CycleMode rising-edge ISR) toggles exit.
                    operateJoystick();
                }
                break;

        } // end switch
    } // end while
} // end main