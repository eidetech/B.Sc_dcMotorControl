#include "motorControl.h"

MotorControl::MotorControl()
{
}

MotorControl::~MotorControl()
{
}

MotorControl::pid(float setpoint)
{
    // Time calculation
    unsigned long t = micros();
    float dt = (t - t_prev)/(1.0e6); // Convert to s
    t_prev = t;
    
    int setpoint = 4*330;//1000*sin(prevT/1e6);
    // Read the position
    int safePosL = 0;
    noInterrupts(); // disable interrupts temporarily while reading
    safePosL = posL;
    interrupts(); // turn interrupts back on

    // error
    int e = safePosL - setpoint;

    // derivative
    float dedt = (e-eprev)/(deltaT);

    // integral
    eintegral = eintegral + e*deltaT;

    // control signal
    float u = kp*e + kd*dedt + ki*eintegral;

    // motor power
    float pwr = fabs(u);
    if( pwr > 100 ){
    pwr = 100;
    }

    // motor direction
    int dir = 1;
    if(u<0){
    dir = -1;
    }

    // signal the motor
    setMotor(dir,pwr,PWM_L,IN1_L,IN2_L);

    // store previous error
    eprev = e;

    Serial.print(target);
    Serial.print(" ");
    Serial.print(safePosL);
    Serial.println();

}