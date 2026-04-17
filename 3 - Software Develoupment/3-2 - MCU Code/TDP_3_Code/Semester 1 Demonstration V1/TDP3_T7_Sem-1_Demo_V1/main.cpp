#include "mbed.h"

//  Pin Asignments

//  Right motor I/O
PwmOut rEn(A2);     //right motor enable
DigitalOut rDi(A3); //right motor direction
AnalogIn rSen(A0); //right motor amplified sense input 

//  Left motor I/O
PwmOut lEn(A4);     //left motor enable
DigitalOut lDi(A5); //left motor direction
AnalogIn lSen(A1); //left motor amplified sense input

//motor frequency
int motorFreq = 200; 

void initialize(){
    lEn.period_ms(motorFreq);
    rEn.period_ms(motorFreq);
}

//  Motor Control Functions
void leftControl(int reverse, float power){
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

void stop () {
    lEn = 0;
    rEn = 0;
}
void forward(float power) {
    rightControl(0, power);
    leftControl(0, power);
}

void backward(float power) {
    rightControl(1, power);
    leftControl(1, power);
}

void turnRight(float power) {
    leftControl(0, power); 
    rightControl(1, power);
}

void turnLeft(float power) {
    leftControl(1, power);
    rightControl(0, power);
}

int main() {
    initialize();
    forward(0.5);
    wait_us(1000000);
    backward(0.5);
    wait_us(1000000);
    turnLeft(0.5);
    wait_us(1000000);
    turnRight(0.5);
    wait_us(1000000);
}