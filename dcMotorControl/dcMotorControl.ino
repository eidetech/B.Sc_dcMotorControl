#include "motorControl.h"

int safePosL = 0;
int safePosR = 0;
int posLLocal = 0;
int posRLocal = 0;

int leftDir = 1;
int rightDir = 1;

MotorControl leftM, rightM;

// Kinematics
#define PI 3.14159265359

double px = 400;
double py = 400;
double L1_start = 140; // Length of left string mm
double L2_start = 2960 - 140; // Length of right string mm
double L1 = 0;
double L2 = 0;
const double c = 2960; // Length of top aluminium bar mm
const double r = 20; // Radius of wire wheel in mm
const double circumference = 2*PI*r; // Circumference of string wheel mm (2*pi*r)
const double encoderCountsPerRev = 330;
const double resolution = circumference/encoderCountsPerRev;

bool jobDone = false;

void setup() {
  Serial.begin(9600);
}



void loop() {
  
  px++;

  L1 = sqrt(px*px+py*py) - L1_start;
  L2 = sqrt((c-px)*(c-px)+py*py) - L2_start;
  //teleport();
  down();
  //up();
  //getEncoderVals();   
}

void up()
{
  leftM.pid(-int(round(L1/resolution)), 0, LEFT);
  rightM.pid(0, -int(round(L2/resolution)), RIGHT);
}

void down()
{
  leftM.pid(int(round(L1/resolution)), 0, LEFT);
  rightM.pid(0, int(round(L2/resolution)), RIGHT);
}

void setMotor(int dir, int pwmVal, int pwm, int in1, int in2){
  analogWrite(pwm,pwmVal);
  if(dir == 1){
    digitalWrite(in1,HIGH);
    digitalWrite(in2,LOW);
  }
  else if(dir == -1){
    digitalWrite(in1,LOW);
    digitalWrite(in2,HIGH);
  }
  else{
    digitalWrite(in1,LOW);
    digitalWrite(in2,LOW);
  }  
}

void getEncoderVals()
{
  noInterrupts(); // disable interrupts temporarily while reading
  safePosL = posLLocal;
  safePosR = posRLocal;
  interrupts(); // turn interrupts back on
  Serial.print("safePosL = ");
  Serial.print(safePosL);
  Serial.print(" ");
  Serial.print("safePosR = ");
  Serial.println(safePosR);
}

void teleport()
{
  unsigned long startTime = millis();
  int runtime = 200;
  if (Serial.available() > 0) {
    int inData = Serial.read();
    if (inData == 'j')
    {
      while (startTime + runtime > millis() )
      {
        setMotor(1, 150, PWM_L, IN1_L, IN2_L);
      }
      setMotor(1, 0, PWM_L, IN1_L, IN2_L);
      
    }else if(inData == 'n')
    {
       while (startTime + runtime > millis() )
      {
        setMotor(-1, 150, PWM_L, IN1_L, IN2_L);
      }
      setMotor(-1, 0, PWM_L, IN1_L, IN2_L);
    }else if(inData == 'k')
    {
       while (startTime + runtime > millis() )
      {
        setMotor(1, 150, PWM_R, IN1_R, IN2_R);
      }
      setMotor(1, 0, PWM_R, IN1_R, IN2_R);
    }else if(inData == 'm')
    {
       while (startTime + runtime > millis() )
      {
        setMotor(-1, 150, PWM_R, IN1_R, IN2_R);
      }
      setMotor(-1, 0, PWM_R, IN1_R, IN2_R);
    }
  }
}