#include "SerialChecker.h"
//#include "MilliTimer.h"
//
SerialChecker scpc(Serial);
//HardwareSerial s1(1);

const int step_pin = 22;
const int direc_pin = 21;

void setup()
{
  scpc.init();
//  scmotor.init();
  scpc.setMsgMaxLen(64);
//  scmotor.setMsgMaxLen(64);
  pinMode(direc_pin, OUTPUT); // Initialising the pins here, logic in lab book
  pinMode(step_pin, OUTPUT);
  pinMode(34, INPUT);
  pinMode(35, INPUT);
  digitalWrite(direc_pin, HIGH);
  scpc.println("connected successfully");
}

void loop()
{
    checkSerialPC();
//  checkEndStop();
//  printToLCD();
    runContinuous();
}

void runContinuous()
{
  digitalWrite(step_pin, HIGH);
  delay(5);
  digitalWrite(step_pin, LOW);
  delay(5);
}

void checkSerialPC()
{
  if (scpc.contains("H")){
    scpc.println("Going up");
    digitalWrite(direc_pin, HIGH);
  }
  else if (scpc.contains("L")){
    scpc.println("Going down");
    digitalWrite(direc_pin, LOW);
  }
}
