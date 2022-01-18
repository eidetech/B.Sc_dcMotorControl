#include <util/atomic.h> // For the ATOMIC_BLOCK macro

// Left encoder pins
#define ENCA_L 18
#define ENCB_L 19

// Right encoder pins
#define ENCA_R 3
#define ENCB_R 2

// Left motor pins
#define PWM_L 7
#define IN2_L 8
#define IN1_L 9

// Right motor pins
#define PWM_R 4
#define IN2_R 5
#define IN1_R 6

volatile int posL = 0; // volatile: https://www.arduino.cc/reference/en/language/variables/variable-scope-qualifiers/volatile/
volatile int posR = 0;//4982; // Encoder count at (0, 0)

int safePosL = 0;
int safePosR = 0;

int leftDir = 1;
int rightDir = 1;

// Kinematics
#define PI 3.14159265359

double px = 100;
double py = -100;
double L1 = 0; // Length of left string mm
double L2 = 2330; // Length of right string mm
const double c = 2310; // Length of top aluminium bar mm
const double r = 20; // Radius of wire wheel in mm
const double circumference = 2*PI*r; // Circumference of string wheel mm (2*pi*r)
const double encoderCountsPerRev = 310;
const double encoderCircumferenceRatio = circumference/encoderCountsPerRev;

bool jobDone = false;

void setup() {
  Serial.begin(9600);

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

void loop() {
  //teleport();
  //goSomeplace();
  //delay(1000);
  //jobDone = false;
  //goHome();
  //getEncoderVals();
  runEncoder();
}

void goSomeplace()
{
  if(!jobDone)
  {

  // Calculate lengths of strings
  L1 = sqrt(px*px+py*py) - L1;
  L2 = sqrt((c-px)*(c-px)+py*py) - L2;

  //L1 = sqrt(py*py + (px+c/2) * (px+c/2));
  //L2 = sqrt(py*py + (px-c/2) * (px-c/2));

  // Print calculated lengths
  Serial.print("L1 = ");
  Serial.println(L1);
  Serial.print("L2 = ");
  Serial.println(L2);

  if (L1 < 0)
  {
    leftDir = 1;
  }else if (L1 > 0)
  {
    leftDir = -1;
  }

  if (L2 > 0)
  {
    rightDir = -1;
  }else if (L2 < 0)
  {
    rightDir = 1;
  }

  while (safePosL*encoderCircumferenceRatio <= L1 || safePosR*encoderCircumferenceRatio >= L2)
  {
    // Interrupt handling
    noInterrupts(); // Disable interrupts
    safePosL = posL;
    safePosR = posR;
    interrupts(); // Enable interrupts

    
    Serial.print("L1 = ");
    Serial.print(safePosL*encoderCircumferenceRatio);
    Serial.print(" ");
    Serial.print(L1);
    Serial.print(" | ");
    Serial.print("L2 = ");
    Serial.print(safePosR*encoderCircumferenceRatio);
    Serial.print(" ");
    Serial.println(L2);

    setMotor(leftDir, 75, PWM_L, IN1_L, IN2_L);
    setMotor(rightDir, 75, PWM_R, IN1_R, IN2_R);
    
    

    if (safePosL*encoderCircumferenceRatio >= L1)
    {
      setMotor(-leftDir, 10, PWM_L, IN1_L, IN2_L);
    }
    if (safePosR*encoderCircumferenceRatio <= L2)
    {
      setMotor(-rightDir, 10, PWM_R, IN1_R, IN2_R);
    }
  }
  
  Serial.println("Outside while loop");

  jobDone = true;
  }
}

void goHome()
{
  if(!jobDone)
  {
  while (safePosL > 0 || safePosR > 4215)
  {
    // Interrupt handling
    safePosL = 0;
    safePosR = 0;
    noInterrupts(); // Disable interrupts
    safePosL = posL;
    safePosR = posR;
    interrupts(); // Enable interrupts


    setMotor(-leftDir, 150, PWM_L, IN1_L, IN2_L);
    Serial.println(safePosL);
    Serial.println(safePosR);
    setMotor(-rightDir, 150, PWM_R, IN1_R, IN2_R);

    if (safePosL > 0)
    {
      setMotor(leftDir, 0, PWM_L, IN1_L, IN2_L);
    }
    if (safePosR > 4215)
    {
      setMotor(rightDir, 0, PWM_R, IN1_R, IN2_R);
    }
  }
  jobDone = true;
  }
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
// Interrupt handling
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    safePosL = posL;
    safePosR = posR;
  }
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

void runEncoder()
{
  unsigned long startTime = millis();
  int runtime = 200;
  int counts = 140;
  if (Serial.available() > 0) {
    int inData = Serial.read();
    if (inData == 'j')
    {
      ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      safePosL = posL;
      safePosR = posR;
      }
      while (safePosL < counts)
      {
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        safePosL = posL;
        safePosR = posR;
        }
        setMotor(-1, 100, PWM_L, IN1_L, IN2_L);

        Serial.print("safePosL = ");
        Serial.print(safePosL);
        Serial.print(" ");
        Serial.print("safePosR = ");
        Serial.println(safePosR);
        if(safePosL >= counts)
        {
        setMotor(1, 20, PWM_L, IN1_L, IN2_L);
        Serial.print("safePosL = ");
        Serial.print(safePosL);
        Serial.print(" ");
        Serial.print("safePosR = ");
        Serial.println(safePosR);
        }
      }
    }else if (inData == 'n')
    {
      while (safePosR > counts)
      {
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    safePosL = posL;
    safePosR = posR;
  }
        setMotor(1, 100, PWM_L, IN1_L, IN2_L);

        Serial.print("safePosL = ");
        Serial.print(safePosL);
        Serial.print(" ");
        Serial.print("safePosR = ");
        Serial.println(safePosR);
        if(safePosR == counts)
        {
          setMotor(-1, 0, PWM_R, IN1_R, IN2_R);
                Serial.print("safePosL = ");
        Serial.print(safePosL);
        Serial.print(" ");
        Serial.print("safePosR = ");
        Serial.println(safePosR);
        }
      }
    }else if(inData == 'k')
    {
       while (startTime + runtime > millis() )
      {
        setMotor(1, 150, PWM_R, IN1_R, IN2_R);
        safePosL = 0;
        safePosR = 0;
      }
      setMotor(1, 0, PWM_R, IN1_R, IN2_R);
    }else if(inData == 'm')
    {
       while (startTime + runtime > millis() )
      {
        setMotor(-1, 150, PWM_R, IN1_R, IN2_R);
        safePosL = 0;
        safePosR = 0;
      }
      setMotor(-1, 0, PWM_R, IN1_R, IN2_R);
    }
    
    
  }
}

void encoder_ISR_L(){
  int encL = digitalRead(ENCB_L);
  if(encL > 0){
    posL++;
  }
  else{
    posL--;
  }
}

void encoder_ISR_R(){
  int encR = digitalRead(ENCB_R);
  if(encR > 0){
    posR++;
  }
  else{
    posR--;
  }
}
