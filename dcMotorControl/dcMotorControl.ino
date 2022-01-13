// Left encoder pins
#define ENCA_L 13
#define ENCB_L 2

// Right encoder pins
#define ENCA_R 13
#define ENCB_R 3

// Left motor pins
#define PWM_L 9
#define IN2_L 5
#define IN1_L 4

// Right motor pins
#define PWM_R 10
#define IN2_R 11
#define IN1_R 12

volatile int posL = 0; // volatile: https://www.arduino.cc/reference/en/language/variables/variable-scope-qualifiers/volatile/
volatile int posR = 0;

int safePosL = 0;
int safePosR = 0;

// Kinematics
double px = 2640/2;
double py = 500;
double L1 = 0; // Length of left string mm
double L2 = 2940; // Length of right string mm
const double c = 2940; // Length of top aluminium bar mm
const double circumference = 138.23; // Circumference of string wheel mm
const double encoderCircumferenceRatio = circumference/342;

bool jobDone = false;

void setup() {
  Serial.begin(9600);

  // Left encoder setup
  pinMode(ENCA_L,INPUT);
  pinMode(ENCB_L,INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCB_L),encoder_ISR_L,RISING);

  // Right encoder setup
  pinMode(ENCA_R,INPUT);
  pinMode(ENCB_R,INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCB_R),encoder_ISR_R,RISING);
  
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
  goSomeplace();
}

void goSomeplace()
{
  if(!jobDone)
  {

  L1 = sqrt(px*px+py*py) - L1;
  L2 = sqrt((c-px)*(c-px)+py*py) - L2;

  Serial.print("L1 = ");
  Serial.println(L1);
  Serial.print("L2 = ");
  Serial.println(L2);

  

  while (safePosL*encoderCircumferenceRatio <= L1)
  {
  Serial.println("Inside while loop 1");
  // Interrupt handling
  safePosL = 0;
  noInterrupts(); // Disable interrupts
  safePosL = posL;
  interrupts(); // Enable interrupts
  setMotor(1, 150, PWM_L, IN1_L, IN2_L);
  Serial.println(safePosL*encoderCircumferenceRatio);
  Serial.println(L1);

  if (safePosL*encoderCircumferenceRatio >= L1)
  {
    setMotor(2, 0, PWM_L, IN1_L, IN2_L);
    while (safePosR*encoderCircumferenceRatio >= L2)
    {
      Serial.println("Inside while loop 2");
      // Interrupt handling
      safePosR = 0;
      noInterrupts(); // Disable interrupts
      safePosR = posR;
      interrupts(); // Enable interrupts
      setMotor(1, 150, PWM_R, IN1_R, IN2_R);
      Serial.println(safePosR*encoderCircumferenceRatio);
      Serial.println(L2);
        if (safePosR*encoderCircumferenceRatio <= L2)
        {
          setMotor(2, 0, PWM_R, IN1_R, IN2_R);
        }
    }
  
  }
  
  Serial.println("Outside while loop");


  jobDone = true;
  }
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

void teleport()
{
  unsigned long startTime = millis();
  int runtime = 500;
  if (Serial.available() > 0) {
    int inData = Serial.read();
    if (inData == 'j')
    {
      while (startTime + runtime > millis() )
      {
        setMotor(1, 150, PWM_L, IN1_L, IN2_L);
      }
      setMotor(2, 150, PWM_L, IN1_L, IN2_L);
      
    }else if(inData == 'n')
    {
       while (startTime + runtime > millis() )
      {
        setMotor(-1, 150, PWM_L, IN1_L, IN2_L);
      }
      setMotor(2, 150, PWM_L, IN1_L, IN2_L);
    }else if(inData == 'k')
    {
       while (startTime + runtime > millis() )
      {
        setMotor(1, 150, PWM_R, IN1_R, IN2_R);
      }
      setMotor(2, 150, PWM_R, IN1_R, IN2_R);
    }else if(inData == 'm')
    {
       while (startTime + runtime > millis() )
      {
        setMotor(-1, 150, PWM_R, IN1_R, IN2_R);
      }
      setMotor(2, 150, PWM_R, IN1_R, IN2_R);
    }
  }
}

void encoder_ISR_L(){
  int b = digitalRead(ENCB_L);
  if(b > 0){
    posL++;
  }
  else{
    posL--;
  }
}

void encoder_ISR_R(){
  int c = digitalRead(ENCB_R);
  if(c > 0){
    posR--;
  }
  else{
    posR++;
  }
}
