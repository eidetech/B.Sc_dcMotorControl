#include "motorControl.h"
#include "Arduino.h"

static volatile int posL = 0;// volatile: https://www.arduino.cc/reference/en/language/variables/variable-scope-qualifiers/volatile/
static volatile int posR = 0;

MotorControl::MotorControl()
{
    // Left encoder setup
    pinMode(ENCA_L,INPUT);
    pinMode(ENCB_L,INPUT);
    attachInterrupt(digitalPinToInterrupt(ENCA_L),encoder_ISR_L,RISING);

    // Right encoder setup
    pinMode(ENCA_R,INPUT);
    pinMode(ENCB_R,INPUT);
    attachInterrupt(digitalPinToInterrupt(ENCA_R),encoder_ISR_R,RISING);

    // Left motor controller setup
    pinMode(PWM_L,OUTPUT);
    pinMode(IN1_L,OUTPUT);
    pinMode(IN2_L,OUTPUT);

    // Right motor controller setup
    pinMode(PWM_R,OUTPUT);
    pinMode(IN1_R,OUTPUT);
    pinMode(IN2_R,OUTPUT);
}

MotorControl::~MotorControl()
{
}

float MotorControl::pid(float setpointL, float setpointR, motor motorSide)
{
    // Time calculation
    unsigned long t = micros();
    float dt = (t - t_prev)/(1.0e6); // Convert to s
    t_prev = t;
    
    // Read the position
    int safePosL = 0;
    int safePosR = 0;
    noInterrupts(); // disable interrupts temporarily while reading
    safePosL = posL;
    safePosR = posR;
    interrupts(); // turn interrupts back on

    if (motorSide == LEFT)
    {
        // error
        int e = safePosL - setpointL;

        // derivative
        float dedt = (e-eprev)/(dt);

        // integral
        eintegral = eintegral + e*dt;

        // control signal
        float u = kp_L*e + ki_L*eintegral + kd_L*dedt;

        // motor power
        float pwr = fabs(u);
        if( pwr > 100 ){
        pwr = 80;
        }

        // motor direction
        int dir = 1;
        if(u<0){
        dir = -1;
        }

        // signal the motor
        runMotor(dir,pwr,PWM_L,IN1_L,IN2_L);

        // store previous error
        eprev = e;
        return e;



    }else if (motorSide == RIGHT)
    {
        // error
        int e = safePosR - setpointR;

        // derivative
        float dedt = (e-eprev)/(dt);

        // integral
        eintegral = eintegral + e*dt;

        // control signal
        float u = kp_R*e + ki_R*eintegral + kd_R*dedt;

        // motor power
        float pwr = fabs(u);
        if( pwr > 100 ){
        pwr = 120;
        }

        // motor direction
        int dir = 1;
        if(u<0){
        dir = -1;
        }

        // signal the motor
        runMotor(dir,pwr,PWM_R,IN1_R,IN2_R);

        // store previous error
        eprev = e;
        return e;

    }


}

void MotorControl::runMotor(int dir, int pwmVal, int pwmPin, int in1, int in2)
{
    // Set the direction of the motor
    if(dir == 1){
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    }
    else if(dir == -1){
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    }
    else{
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    }

    // Output PWM signal to motor driver
    analogWrite(pwmPin, pwmVal);  
}


void MotorControl::encoder_ISR_L(){
  int encL = digitalRead(ENCB_L);
  if(encL > 0){
    posL++;
  }
  else{
    posL--;
  }
}

void MotorControl::encoder_ISR_R(){
  int encR = digitalRead(ENCB_R);
  if(encR > 0){
    posR++;
  }
  else{
    posR--;
  }
}
