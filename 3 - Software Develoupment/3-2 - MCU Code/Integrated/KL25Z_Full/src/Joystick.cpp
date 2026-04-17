// joystick.cpp
#ifndef JOYSTICK_CPP
#define JOYSTICK_CPP

#include "header.hpp"

void initJoystick() {
    // BUG FIX 1: old code referenced Mode_Cycle which does not exist in pins.cpp.
    //            The correct name (matching the InterruptIn defined in pins.cpp) is JS_CycleMode.
    // BUG FIX 2: old header declared JS_Enable (DigitalIn) which was never defined anywhere.
    //            Removed entirely; mode cycling is handled purely by JS_CycleMode interrupt.
    JS_CycleMode.rise(&trigModeCycle);
    dbgPrint("[JS] Joystick init complete\r\n");
}

void trigModeCycle() {
    // Rising-edge ISR: toggle between MANUAL and FOLLOWING.
    // Called from ISR context – keep minimal.
    if (roverState == RoverState::MANUAL) {
        roverState = RoverState::FOLLOWING;
        dbgPrint("[JS] Mode → FOLLOWING\r\n");
    } else {
        roverState = RoverState::MANUAL;
        dbgPrint("[JS] Mode → MANUAL\r\n");
    }
}

void operateJoystick() {
    // BUG FIX: AnalogIn returns 0.0–1.0.  A centred joystick reads ~0.5, not 0.
    //          Scale to −1.0 … +1.0 by subtracting the midpoint and doubling.
    //          Old code passed raw 0–1 values directly to motorsOutput, meaning
    //          "centre stick" commanded ~50 % forward + 50 % right at all times.
    float turning = (JS_Turning.read() - 0.5f) * 2.0f;  // −1=full left, +1=full right
    float speed   = (JS_Speed.read()   - 0.5f) * 2.0f;  // −1=full rev,  +1=full fwd

    // Optional: apply a small dead-band around centre to avoid joystick drift
    const float DEADBAND = 0.05f;
    if (fabs(turning) < DEADBAND) turning = 0.0f;
    if (fabs(speed)   < DEADBAND) speed   = 0.0f;

    motorsOutput(turning, speed);

    dbgPrint("[JS] turn=%.2f spd=%.2f\r\n", turning, speed);
}

#endif // JOYSTICK_CPP