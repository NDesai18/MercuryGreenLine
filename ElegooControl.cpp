#include "SerialChecker.h"
#include "MilliTimer.h"
#include "MCP3208.h"


SerialChecker sc(Serial);

//PIN ASSIGNMENTS FOR REFERENCE
const int step_pin = 22;
const int direc_pin = 21;
const int upstop_pin = 34;
const int downstop_pin = 35;
const int top_LED_pin = 32;
const int bot_LED_pin = 33;
const int reset_pin = 25; //enables motor functions when high

//Initialise ADC?
MCP3208 adc(5);
const int scan_pot_chan = 0;

//Useful constants
int delaytime = 900;    //in microseconds
const int pulsetime = 250;    //in microseconds
const int mindelaytime = 600;
int steprate = (pow(10,6))/(delaytime + pulsetime);   //in steps/s
const int maxsteprate = (pow(10,6))/850;    // in steps/s
int steps = 0;
float distance = 0.0;
bool direc = HIGH;    //stores direction
bool upstop = HIGH;   //LOW corresponds to the switch being pressed
bool downstop = HIGH;
float scanPotV = 0.0; // Initialises measured potentiometer voltage to 0.0
const int timeStep = 100;
float floatPotV = 0.0;
MilliTimer dataTimer(timeStep);
MilliTimer DebounceTimer(25);   //in ms

enum class states{
  STATIONARY,
  ZEROING,    //reset 0 steps at top endstop
  MOVING,    //run continuously
  MOVE_DISCRETE   //move a given number of steps
};

states motorstate = states::STATIONARY;

void setup() {
  pinMode(step_pin, OUTPUT);
  pinMode(direc_pin, OUTPUT);
  pinMode(upstop_pin, INPUT);
  pinMode(downstop_pin, INPUT);
  pinMode(top_LED_pin, OUTPUT);
  pinMode(bot_LED_pin, OUTPUT);
  pinMode(reset_pin, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(upstop_pin), stopMotorTop, CHANGE);
  attachInterrupt(digitalPinToInterrupt(downstop_pin), stopMotorBot, CHANGE);
  digitalWrite(direc_pin, HIGH);    //start with direction set to UP
  digitalWrite(reset_pin, LOW);    //start with motor disabled
  adc.begin();
  sc.init();
  sc.setMsgMinLen(1);
  sc.println("Connected to Bouncymotor.ino");
}

void loop() {
//  upstop = digitalRead(upstop_pin);
//  downstop = digitalRead(downstop_pin);
//  sc.println(digitalRead(upstop_pin));
  checkSerial();
  readAnalogInputs();

  
  switch(motorstate){
    case states::STATIONARY:    //do nothing
      digitalWrite(reset_pin, LOW);
    break;
    case states::ZEROING:
      digitalWrite(reset_pin, HIGH);
      zeroing();
    break;
    case states::MOVING:    //loops, sending pulses continuously
      digitalWrite(reset_pin, HIGH);
      pulse();
    break;
    case states::MOVE_DISCRETE:
      digitalWrite(reset_pin, HIGH);
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
      digitalWrite(reset_pin, LOW);   //disable motor
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
    else if(sc.contains("ze")){   //commence zeroing
      sc.println("Commencing Zeroing...");
      motorstate = states::ZEROING;
    }
    else if(sc.contains("ms")){   //move given number of steps
      int ms = sc.toInt32();
      if(upstop == LOW && direc == HIGH){
        sc.println("Cannot move further up! Please change directions!");
      }
      else if(downstop == LOW && direc == LOW){
        sc.println("Cannot move further down! Please change directions!");
      }
      else{
        sc.println(ms);
        motorstate = states::MOVE_DISCRETE;
        digitalWrite(reset_pin, HIGH);
        for(int i = 0; i < ms; i++){
          digitalWrite(step_pin, HIGH);
          delayMicroseconds(pulsetime);
          digitalWrite(step_pin, LOW);
          delayMicroseconds(delaytime); 
          if(direc == HIGH){
            steps += -1;
          }
          else if(direc == LOW){
            steps += 1;
          }
          if(motorstate == states::STATIONARY){
            break;
          }
          if(sc.check()){
            sc.println("Cancelled move steps.");
            break;
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
    else if(sc.contains("gp")){   //get position
      distance = steps*7.98/20000;   //in mm
      sc.print("Current stepcount is: ");
      sc.println(steps);
      sc.print("Current position is: ");
      sc.print(distance);
      sc.println("mm");
    }
  }
}

void stopMotorTop(){    //top endstop registers a change
  if(DebounceTimer.timedOut(true)){
    if(direc == HIGH){    //if moving upward... ie. moving onto the endstop
//      sc.println("upstop triggered!");              //!!FOR TESTING!!
      motorstate = states::STATIONARY;
      digitalWrite(reset_pin, LOW);   //disable motor
      steps = 0; //steps count from the top endstop downward
      sc.println("Stepcount set to 0");
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
      digitalWrite(reset_pin, LOW);   //disable motor
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

void pulse(){
  digitalWrite(step_pin, HIGH);
  delayMicroseconds(pulsetime);
  digitalWrite(step_pin, LOW);
  delayMicroseconds(delaytime); 
  if(direc == HIGH){
    steps += -1;
  }
  else if(direc == LOW){
    steps += 1;
  }
}

void zeroing(){   //runs the motor upward into the endstop, where it calibrates steps to 0
  if(upstop == LOW){
    steps = 0;
    sc.println("Stepcount set to 0");
    motorstate = states::STATIONARY;
  }
  else{
    digitalWrite(direc_pin, HIGH);
    direc = HIGH;
    pulse();
  }
}

void readAnalogInputs(){
    static int32_t potV = 0;
    static int32_t nsamples = 0;
    if(!dataTimer.timedOut(true) || nsamples == 0){
      potV += adc.read(scan_pot_chan);
      nsamples++;
    }
    else{
      if(nsamples){
        potV /= nsamples;
      }
      floatPotV = 5 * float(potV) / 4095.0;
      potV = 0;
      nsamples = 0;
//      sc.println(floatPotV);
    }
}
