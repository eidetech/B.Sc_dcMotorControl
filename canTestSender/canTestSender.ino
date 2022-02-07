#include <SPI.h>
#include <mcp2515.h>

struct can_frame canMsg1;
MCP2515 mcp2515(10);


void setup() {
  canMsg1.can_id  = 0x01;
  canMsg1.can_dlc = 8;
  canMsg1.data[0] = 0x01;
  canMsg1.data[1] = 0x02;
  canMsg1.data[2] = 0x03;
  canMsg1.data[3] = 0x04;
  canMsg1.data[4] = 0x05;
  canMsg1.data[5] = 0x06;
  canMsg1.data[6] = 0x07;
  canMsg1.data[7] = 0x08;
  
  while (!Serial);
  Serial.begin(115200);
  
  mcp2515.reset();
  mcp2515.setBitrate(CAN_500KBPS);
  mcp2515.setNormalMode();
  
  Serial.println("Starting CAN interface...");
}

void loop() {
  mcp2515.sendMessage(&canMsg1);

  Serial.println("Messages sent");
  
  delay(100);
}
