// obstacle_avoid.cpp
#ifndef OBSTACLE_AVOID_CPP
#define OBSTACLE_AVOID_CPP

#include "header.hpp"

void initObstacleAvoidance() {
    // BUG FIX: old code only attached a rise ISR – there was no way to
    //          exit OBSTACLEAVOID when the obstacle signal went low.
    //          Fall ISR added to return the rover to FOLLOWING.
    ObsAv_Active.rise(&trigObsAvoidance);
    ObsAv_Active.fall(&trigObsAvoidanceEnd);
    dbgPrint("[OBS] Obstacle avoidance init complete\r\n");
}

void trigObsAvoidance() {
    // Called from ISR context – keep brief
    roverState = RoverState::OBSTACLEAVOID;
    // Note: dbgPrint calls espUART.write which buffers asynchronously – safe in ISR
    dbgPrint("[OBS] Obstacle detected – entering OBSTACLEAVOID\r\n");
}

void trigObsAvoidanceEnd() {
    // Called from ISR context when ObsAv_Active falls low
    roverState = RoverState::FOLLOWING;
    dbgPrint("[OBS] Obstacle cleared – returning to FOLLOWING\r\n");
}

void operateObstacleAvoidance() {
    const float baseSpeed = 0.35f;
    bool control1 = ObsAvoidControl1.read();
    bool control2 = ObsAvoidControl2.read();

    /*  Control1  Control2  |  Action
     *     0         0      |  Forward
     *     0         1      |  Skid right
     *     1         0      |  Skid left
     *     1         1      |  Reverse
     */
    if (!control1 && !control2) {
        motorsOutput(0.0f,    baseSpeed);   // Forward
    } else if (!control1 && control2) {
        motorsOutput(0.35f,   baseSpeed);   // Skid right
    } else if (control1 && !control2) {
        motorsOutput(-0.35f,  baseSpeed);   // Skid left
    } else {
        motorsOutput(0.0f,   -baseSpeed);   // Reverse
    }

    dbgPrint("[OBS] ctrl1=%d ctrl2=%d\r\n", (int)control1, (int)control2);
}

#endif // OBSTACLE_AVOID_CPP