#include "mbed.h"

//  Pin Asignments

//  LED
DigitalOut LEDblue(PTD1);
DigitalOut LEDred(PTB18);
DigitalOut LEDgreen(PTB19);


//  Right motor I/O
PwmOut rEn(A2);     //right motor enable
DigitalOut rDi(A3); //right motor direction
AnalogIn rSen(A0); //right motor amplified sense input 

//  Left motor I/O
PwmOut lEn(A4);     //left motor enable
DigitalOut lDi(A5); //left motor direction
AnalogIn lSen(A1); //left motor amplified sense input

//  Turn Rates
int lTR = 1; //time taken to turn 360 degrees at 50% power, make a transfer function of power vs trunrate for final version 
int rTR = 1; //time taken to turn 360 degrees at 50% power, make a transfer function of power vs trunrate for final version 


//motor frequency
int motorPeriod = 10; //10ms period = 100Hz switching frequency

void initialize() {
    lEn.period_ms(motorPeriod);
    rEn.period_ms(motorPeriod);
}

//  Raw Motor Control Functions
void leftControl(int reverse, float power) {
    //  direction 0 for fwd 1 for backwards
    //  power expressed between 0 and 1
    lDi = reverse;
    lEn.write(power);
}

void rightControl(int reverse, float power) {
    //  direction 0 for fwd 1 for backwards
    //  power expressed between 0 and 1
    rDi = reverse;
    rEn.write(power);
}

//Motor control output functions 
void stop (int time_us) {
    leftControl(0, 0);
    rightControl(0, 0);
    wait_us(time_us);
}
void forward(float power, int time_us) {
    rightControl(0, power);
    leftControl(0, power);
    wait_us(time_us);
}

void backward(float power, int time_us) {
    rightControl(1, power);
    leftControl(1, power);
    wait_us(time_us);
}

void turnRight(float power, int turnangle) {
    int turntime = rTR*turnangle/90;
    leftControl(0, power); 
    rightControl(1, power);
    wait_us(turntime);
}

void turnLeft(float power, int turnangle) {
    int turntime = lTR*turnangle/90;
    leftControl(1, power);
    rightControl(0, power);
    wait_us(turntime);
}

int main() {
    
    initialize();
    LEDred = 0;
    LEDblue = 0;
    LEDgreen = 0;


    while (true){
        LEDred = 1;
        forward(0.7, 5000000);
        stop(1000000);
        LEDred = 0;
        LEDblue = 1;
        backward(0.7, 5000000);
        stop(2500000);
        LEDblue = 0;
        LEDgreen = 1;
        turnLeft(0.7, 450000000);
        stop(2500000);
        LEDgreen = 0;
        LEDblue = 1;
        turnRight(0.7, 450000000);
        stop(2500000);
        LEDblue = 0;
    }
    
}