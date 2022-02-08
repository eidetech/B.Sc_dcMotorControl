#include "motorControl.h"
#include "Arduino.h"
#include <SPI.h>
#include <mcp2515.h>

struct can_frame rx_cartCoord;
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

double px = 0;//1000;
double py = 0;//500;
double L1_start = 50; // Length of left string mm
double L2_start = 2820 - 50; // Length of right string mm
double L1 = 0;
double L2 = 0;
const double c = 2820; // Length of top aluminium bar mm
const double r = 45; // Radius of wire wheel in mm
const double circumference = 2*PI*r; // Circumference of string wheel mm (2*pi*r)
const double encoderCountsPerRev = 330;
const double resolution = circumference/encoderCountsPerRev;

bool jobDone = false;

void setup() {
  Serial.begin(115200);
  mcp2515.reset();
  mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ);
  mcp2515.setNormalMode();
  // The line below causes the left motor to stop working
  //TCCR4B = TCCR4B & 0b11111000 | 0x01; // Setting the PWM frequency from 490Hz to 32kHz
}

void loop() {
	L1 = sqrt(px*px+py*py) - L1_start;
	L2 = sqrt((c-px)*(c-px)+py*py) - L2_start;

	Serial.print("L1: ");
	Serial.print(L1);
	Serial.print(", ");
	Serial.print("L2: ");
	Serial.println(L2);

	leftM.pid(int(round(L1/resolution)), 0, LEFT);
	rightM.pid(0, int(round(L2/resolution)), RIGHT);

	receiveCAN();
	// Serial.print("px: ");
	// Serial.print(px);
	// Serial.print(", ");
	// Serial.print("py: ");
	// Serial.println(py);

  //wayPoints();
  //teleport();
  //down();
  //up();
  //getEncoderVals();   
}

void receiveCAN()
{
	if (mcp2515.readMessage(&rx_cartCoord) == MCP2515::ERROR_OK) {
		// Serial.print("Received CAN message with ID ");
    	// Serial.println(rx_cartCoord.can_id, HEX);
		// Serial.print("Payload size:");
		// Serial.println(rx_cartCoord.can_dlc, HEX); // print DLC
		// Serial.println("CAN message:");
		// for (int i = 0; i<rx_cartCoord.can_dlc; i++){  // print the data
		// Serial.print(rx_cartCoord.data[i], HEX);
		// Serial.print(" ");
		// }
		// Serial.println();

		uint32_t joyX = (long)rx_cartCoord.data[0] | (long)rx_cartCoord.data[1]<<8 | (long)rx_cartCoord.data[2]<<16;
		uint32_t joyY = (long)rx_cartCoord.data[3] | (long)rx_cartCoord.data[4]<<8 | (long)rx_cartCoord.data[5]<<16;

		uint8_t joyDir = rx_cartCoord.data[6];
		float joyXY[2] = {0., 0.};

		joyXY[0] = (float)joyX / 1000000.;
		joyXY[1] = (float)joyY / 1000000.;
		if (!((joyDir >> 0) & 1U))
		{
			joyXY[0] *= -1.;
		}
		if (((joyDir >> 1) & 1U))
		{
			joyXY[1] *= -1.;
		}

		if (joyXY[0] > 0.5)
		{
			px++;
		}else if (joyXY[0] < -0.5)
		{
			px--;
		}else if (joyXY[1] > 0.5)
		{
			py++;
		}else if (joyXY[1] < -0.5)
		{
			py--;
		}
		
		// Serial.println(joyXY[0]);
		// Serial.println(joyXY[1]);
			
	}
}

// void wayPoints()
// {
//   L1 = sqrt(px*px+py*py) - L1_start;
//   L2 = sqrt((c-px)*(c-px)+py*py) - L2_start;

//   float e_L = leftM.pid(int(round(L1/resolution)), 0, LEFT);
//   float e_R = rightM.pid(0, -int(round(L2/resolution)), RIGHT);

//   Serial.print(e_L);
//   Serial.print(" ");
//   Serial.print(e_R);
//   Serial.println();

//   if(e_L > -10 && e_L < 10 && e_R > -10 && e_R < 10)
//   {
//   px = 1000;
//   py = 250;
  
//   L1 = sqrt(px*px+py*py) - L1;
//   L2 = sqrt((c-px)*(c-px)+py*py) - L2;

//   float e_L = leftM.pid(int(round(L1/resolution)), 0, LEFT);
//   float e_R = rightM.pid(0, -int(round(L2/resolution)), RIGHT);
//   }
// }

// void up()
// {
//   leftM.pid(-int(round(L1/resolution)), 0, LEFT);
//   rightM.pid(0, int(round(L2/resolution)), RIGHT);
// }

// void down()
// {
//   leftM.pid(int(round(L1/resolution)), 0, LEFT);
//   rightM.pid(0, -int(round(L2/resolution)), RIGHT);
// }

// void setMotor(int dir, int pwmVal, int pwm, int in1, int in2){
//   analogWrite(pwm,pwmVal);
//   if(dir == 1){
//     digitalWrite(in1,HIGH);
//     digitalWrite(in2,LOW);
//   }
//   else if(dir == -1){
//     digitalWrite(in1,LOW);
//     digitalWrite(in2,HIGH);
//   }
//   else{
//     digitalWrite(in1,LOW);
//     digitalWrite(in2,LOW);
//   }  
// }

// void teleport()
// {
//   unsigned long startTime = millis();
//   int runtime = 200;
//   if (Serial.available() > 0) {
//     int inData = Serial.read();
//     if (inData == 'j')
//     {
//       while (startTime + runtime > millis() )
//       {
//         setMotor(1, 150, PWM_L, IN1_L, IN2_L);
//       }
//       setMotor(1, 0, PWM_L, IN1_L, IN2_L);
      
//     }else if(inData == 'n')
//     {
//        while (startTime + runtime > millis() )
//       {
//         setMotor(-1, 150, PWM_L, IN1_L, IN2_L);
//       }
//       setMotor(-1, 0, PWM_L, IN1_L, IN2_L);
//     }else if(inData == 'k')
//     {
//        while (startTime + runtime > millis() )
//       {
//         setMotor(1, 150, PWM_R, IN1_R, IN2_R);
//       }
//       setMotor(1, 0, PWM_R, IN1_R, IN2_R);
//     }else if(inData == 'm')
//     {
//        while (startTime + runtime > millis() )
//       {
//         setMotor(-1, 150, PWM_R, IN1_R, IN2_R);
//       }
//       setMotor(-1, 0, PWM_R, IN1_R, IN2_R);
//     }
//   }
// }
