#include "SerialChecker.h"

SerialChecker scpc(Serial);
HardwareSerial s1(1);

void setup()
{
  scpc.init();
  scmotor.init();
  scpc.setMsgMaxLen(64);
  scmotor.setMsgMaxLen(64);
  pinMode(13, OUTPUT); // Initialising the pins here, logic in lab book
  pinMode(12, OUTPUT);
  pinMode(34, INPUT);
  pinMode(35, INPUT);
}

void loop()
{
 checkSerialPC();
 checkEndStop();
 printToLCD();
}

void checkSerialPC()
{

}
