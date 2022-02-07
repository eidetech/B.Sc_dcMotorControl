#include "motorControl.h"

#include <SPI.h>
#include <mcp2515.h>

struct can_frame canMsg;
MCP2515 mcp2515(53);

int safePosL = 0;
int safePosR = 0;
int posLLocal = 0;
int posRLocal = 0;

int leftDir = 1;
int rightDir = 1;

MotorControl leftM, rightM;

// Kinematics
#define PI 4*atan(1)

double px = 2000;
double py = 500;
double L1_start = 50; // Length of left string mm
double L2_start = 2820 - 50; // Length of right string mm
double L1 = 0;
double L2 = 0;
const double c = 2820; // Length of top aluminium bar mm
const double r = 25.21; // Radius of wire wheel in mm
const double circumference = 2*PI*r; // Circumference of string wheel mm (2*pi*r)
const double encoderCountsPerRev = 330;
const double resolution = circumference/encoderCountsPerRev;

bool jobDone = false;

void setup() {
  Serial.begin(115200);
  mcp2515.reset();
  mcp2515.setBitrate(CAN_500KBPS);
  mcp2515.setNormalMode();
  // The line below causes the left motor to stop working
  //TCCR4B = TCCR4B & 0b11111000 | 0x01; // Setting the PWM frequency from 490Hz to 32kHz
}

void loop() {

 if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) {
   Serial.print(canMsg.can_id, HEX); // print ID
   Serial.print(" ");
   Serial.print(canMsg.can_dlc, HEX); // print DLC
   Serial.print(" ");
    
   for (int i = 0; i<canMsg.can_dlc; i++)  {  // print the data
     Serial.print(canMsg.data[i],HEX);
     Serial.print(" ");
   }

   Serial.println();      
 }

  

  //leftM.pid(round(L1/resolution), 0, LEFT);
  //rightM.pid(0, round(L2/resolution), RIGHT);
  //wayPoints();
  //teleport();
  //down();
  //up();
  //getEncoderVals();   
}

void wayPoints()
{
  L1 = sqrt(px*px+py*py) - L1_start;
  L2 = sqrt((c-px)*(c-px)+py*py) - L2_start;

  float e_L = leftM.pid(int(round(L1/resolution)), 0, LEFT);
  float e_R = rightM.pid(0, -int(round(L2/resolution)), RIGHT);

  Serial.print(e_L);
  Serial.print(" ");
  Serial.print(e_R);
  Serial.println();

  if(e_L > -10 && e_L < 10 && e_R > -10 && e_R < 10)
  {
  px = 1000;
  py = 250;
  
  L1 = sqrt(px*px+py*py) - L1;
  L2 = sqrt((c-px)*(c-px)+py*py) - L2;

  float e_L = leftM.pid(int(round(L1/resolution)), 0, LEFT);
  float e_R = rightM.pid(0, -int(round(L2/resolution)), RIGHT);
  }
}

void up()
{
  leftM.pid(-int(round(L1/resolution)), 0, LEFT);
  rightM.pid(0, int(round(L2/resolution)), RIGHT);
}

void down()
{
  leftM.pid(int(round(L1/resolution)), 0, LEFT);
  rightM.pid(0, -int(round(L2/resolution)), RIGHT);
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
