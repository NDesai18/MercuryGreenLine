#include "SerialChecker.h"
#include "MilliTimer.h"

SerialChecker sc(Serial);

//PIN ASSIGNMENTS FOR REFERENCE
const int step_pin = 22;
const int direc_pin = 21;
const int upstop_pin = 34;
const int downstop_pin = 35;
const int top_LED_pin = 32;
const int bot_LED_pin = 33;

int delaytime = 400;    //in microseconds
const int mindelaytime = 400;
int steprate = 1/delaytime;
const int maxsteprate = 1250;
int steps = 0;
bool direc = HIGH;    //stores direction

bool upstop = HIGH;   //LOW corresponds to the switch being pressed
bool downstop = HIGH;

MilliTimer DebounceTimer(25);   //in ms

enum class states{
  STATIONARY,
  ZEROING,
  MOVING,    //run continuously
  MOVE_DISCRETE   //move a given number of steps
};

states motorstate = states::STATIONARY;

void setup() {
  attachInterrupt(digitalPinToInterrupt(upstop_pin), stopMotorTop, CHANGE);
  attachInterrupt(digitalPinToInterrupt(downstop_pin), stopMotorTop, CHANGE);
  pinMode(step_pin, OUTPUT);
  pinMode(direc_pin, OUTPUT);
  pinMode(upstop_pin, INPUT);
  pinMode(downstop_pin, INPUT);
  pinMode(top_LED_pin, OUTPUT);
  pinMode(bot_LED_pin, OUTPUT);
  digitalWrite(direc_pin, HIGH);    //start with direction set to UP
  sc.init();
  sc.setMsgMinLen(1);
  sc.println("Connected to Bouncymotor.ino");
}

void loop() {
//  upstop = digitalRead(34);
//  downstop = digitalRead(35);
  checkSerial();
  
  switch(motorstate){
    case states::STATIONARY:    //do nothing
    break;
    case states::ZEROING:
//      zeroing();
    break;
    case states::MOVING:
      digitalWrite(step_pin, HIGH);
      delayMicroseconds(delaytime);
      digitalWrite(step_pin, LOW);
      delayMicroseconds(delaytime); 
      if(direc == HIGH){
        steps += -1;
      }
      else if(direc == LOW){
        steps += 1;
      }
    break;
    case states::MOVE_DISCRETE:
    break;
  }
}

void checkSerial(){
  if(sc.check()){
    if(sc.contains("id")){    //get id
      sc.println("BM");
    }
    else if(sc.contains("0")){    //stop
      motorstate = states::STATIONARY;
      digitalWrite(step_pin, LOW);
    }
    else if(sc.contains("1")){    //run continuous
      if(upstop == LOW && direc == HIGH){
        sc.println("Cannot move further up! Please change directions!");
      }
      else if(downstop == LOW && direc == LOW){
        sc.println("Cannot move further down! Please change directions!");
      }
      else{
        motorstate = states::MOVING;
      }
    }
    else if(sc.contains("ms")){   //move given number of steps
      int ms = sc.toInt16();
      if(upstop == LOW && direc == HIGH){
        sc.println("Cannot move further up! Please change directions!");
      }
      else if(downstop == LOW && direc == LOW){
        sc.println("Cannot move further down! Please change directions!");
      }
      else{
        motorstate = states::MOVE_DISCRETE;
        for(int i = 0; i < ms; i++){    
          digitalWrite(step_pin, HIGH);
          delayMicroseconds(delaytime);
          digitalWrite(step_pin, LOW);
          delayMicroseconds(delaytime); 
          if(direc == HIGH){
            steps += -1;
          }
          else if(direc == LOW){
            steps += 1;
          }
        }
        motorstate = states::STATIONARY;
      }
    }
    else if(sc.contains("su")){   //set direc to up
      if(direc == LOW){
        digitalWrite(direc_pin, HIGH);
        direc = HIGH;
        sc.println("Direction set to: up");
      }
      else{
        sc.println("Already moving upward!");
      }
    }
    else if(sc.contains("sd")){   //set direc to down
      if(direc == HIGH){
        digitalWrite(direc_pin, LOW);
        direc = LOW;
        sc.println("Direction set to: down");
      }
      else{
        sc.println("Already moving downward!");
      }
    }
  }
}

void stopMotorTop(){    //top endstop registers a change
  if(DebounceTimer.timedOut(true)){
    if(direc == HIGH){    //if moving upward... ie. moving onto the endstop
//      sc.println("upstop triggered!");              //!!FOR TESTING!!
      motorstate = states::STATIONARY;
      digitalWrite(step_pin, LOW);    //stop sending pulses
      upstop = LOW;   //update upstop
      digitalWrite(top_LED_pin, HIGH);    //turn on the top LED
    }
    else if(direc == LOW){    //if moving downward... ie. moving off of the endstop
//      sc.println("upstop released!");         //!!FOR TESTING!!
      upstop = HIGH;    //update upstop
      digitalWrite(top_LED_pin, LOW);    //turn off the top LED
    }
    DebounceTimer.reset();
  }
}

void stopMotorBot(){    //bot endstop registers a change
  if(DebounceTimer.timedOut(true)){
    if(direc == LOW){    //if moving downward... ie. moving onto the endstop
//      sc.println("downstop triggered!");              //!!FOR TESTING!!
      motorstate = states::STATIONARY;
      digitalWrite(step_pin, LOW);    //stop sending pulses
      downstop = LOW;   //update downstop
      digitalWrite(bot_LED_pin, HIGH);    //turn on the bot LED
    }
    else if(direc == HIGH){   //if moving upward... ie. moving off of the endstop
//      sc.println("downstop released!");              //!!FOR TESTING!!
      downstop = HIGH;    //update downstop
      digitalWrite(bot_LED_pin, LOW);    //turn off the bot LED
    }
    DebounceTimer.reset();
  }
}
