#include "SerialChecker.h"
#include "MilliTimer.h"
#include "MCP3208.h"


SerialChecker sc(Serial);

//PIN ASSIGNMENTS FOR REFERENCE
const int step_pin = 22;
const int direction_pin = 21;
const int upstop_pin = 34;
const int downstop_pin = 35;
const int top_LED_pin = 32;
const int bot_LED_pin = 33;
const int reset_pin = 25; //enables motor functions when high

//Initialise ADC
MCP3208 adc(5); //Clock of the adc is tied to DO5 on the ESP32
const int scan_pot_chan = 0;

//Useful constants
int delay_time = 900;    //in microseconds
const int pulse_time = 250;    //in microseconds
const int min_delay_time = 600;
int step_rate = (pow(10, 6)) / (min_delay_time + pulse_time);   //in steps/s
const int max_step_rate = (pow(10, 6)) / 850;    // in steps/s
int steps = 0;
int step_old = 0;
int discrete_steps = 0;
float distance = 0.0;
bool direction = HIGH;    //stores direction
bool upstop = HIGH;   //LOW corresponds to the switch being pressed
bool downstop = HIGH;
float scan_pot_v = 0.0; // Initialises measured potentiometer voltage to 0.0
const int time_step = 100;
float float_pot_v = 0.0;
MilliTimer data_timer(timeStep);
MilliTimer debounce_timer(25);   //in ms
bool move_discrete = false;

enum class states {
	STATIONARY,
	ZEROING,    //reset 0 steps at top endstop
	MOVING,    //run continuously
};

states motorstate = states::STATIONARY;

void setup() {
	pinMode(step_pin, OUTPUT);
	pinMode(direction_pin, OUTPUT);
	pinMode(upstop_pin, INPUT_PULLUP);
	pinMode(downstop_pin, INPUT_PULLUP);
	pinMode(top_LED_pin, OUTPUT);
	pinMode(bot_LED_pin, OUTPUT);
	pinMode(reset_pin, OUTPUT);
	attachInterrupt(digitalPinToInterrupt(upstop_pin), stopMotorTop, CHANGE);
	attachInterrupt(digitalPinToInterrupt(downstop_pin), stopMotorBot, CHANGE);
	digitalWrite(direction_pin, HIGH);    //start with direction set to UP
	digitalWrite(reset_pin, LOW);    //start with motor disabled
	adc.begin();
	sc.init();
	sc.setMsgMinLen(1);
	sc.println("Connected to Bouncymotor.ino");
}

void loop()
{
	checkSerial();
	readAnalogInputs();

	switch (motorstate) {
	case states::STATIONARY:    //do nothing
		digitalWrite(reset_pin, LOW);
		break;
	case states::ZEROING:
		digitalWrite(reset_pin, HIGH);
		zeroing();
		break;
	case states::MOVING:    //loops, sending pulses continuously
		if (!move_discrete) {
			digitalWrite(reset_pin, HIGH);
			pulse();
		}
		else if (move_discrete) {
			digitalWrite(reset_pin, HIGH);
			pulse();
			if (abs(steps - stepold) >= ms) {
				move_discrete = false;
				motorstate = states::STATIONARY;
			}
		}
		break;
	}
}


void checkSerial()
{
	if (sc.check()) {
		if (sc.contains("id")) {    //get id
			sc.println("BM");
		}
		else if (sc.contains("0")) {    //stop
			motorstate = states::STATIONARY;
			digitalWrite(reset_pin, LOW);   //disable motor
		}
		else if (sc.contains("1")) {    //run continuous
			if (upstop == LOW && direction == HIGH) {
				sc.println("Cannot move further up! Please change directions!");
			}
			else if (downstop == LOW && direction == LOW) {
				sc.println("Cannot move further down! Please change directions!");
			}
			else {
				motorstate = states::MOVING;
			}
		}
		else if (sc.contains("ze")) {   //commence zeroing
			sc.println("Commencing Zeroing...");
			motorstate = states::ZEROING;
		}
		else if (sc.contains("ms")) {   //move given number of steps
			discrete_steps = sc.toInt32();
			if (upstop == LOW && direction == HIGH) {
				sc.println("Cannot move further up! Please change directions!");
			}
			else if (downstop == LOW && direction == LOW) {
				sc.println("Cannot move further down! Please change directions!");
			}
			else {
				stepold = steps;
				move_discrete = true;
				motorstate = states::MOVING;
			}
		}
		else if (sc.contains("su")) {   //set direction to up
			if (direction == LOW) {
				digitalWrite(direction_pin, HIGH);
				direction = HIGH;
				sc.println("Direction set to: up");
			}
			else {
				sc.println("Already moving upward!");
			}
		}
		else if (sc.contains("sd")) {   //set direction to down
			if (direction == HIGH) {
				digitalWrite(direction_pin, LOW);
				direction = LOW;
				sc.println("Direction set to: down");
			}
			else {
				sc.println("Already moving downward!");
			}
		}
		else if (sc.contains("gp")) {   //get position
			distance = steps * 7.98 / 20000;   //in mm
			sc.print("Current stepcount is: ");
			sc.println(steps);
			sc.print("Current position is: ");
			sc.print(distance);
			sc.println("mm");
		}
	}
}

void stopMotorTop() //top endstop registers a change
{   
	if (DebounceTimer.timedOut(true)) {  
		if (direction == HIGH) {    //if moving upward... ie. moving onto the endstop
			motorstate = states::STATIONARY;
			digitalWrite(reset_pin, LOW);   //disable motor
			steps = 0; //steps count from the top endstop downward
			sc.println("Stepcount set to 0");
			upstop = LOW;   //update upstop
			digitalWrite(top_LED_pin, HIGH);    //turn on the top LED
		}
		else if (direction == LOW) {    //if moving downward... ie. moving off of the endstop
			upstop = HIGH;    //update upstop
			digitalWrite(top_LED_pin, LOW);    //turn off the top LED
		}
		DebounceTimer.reset();
	}
}

void stopMotorBot()  //bot endstop registers a change
{   
	if (DebounceTimer.timedOut(true)) {
		if (direction == LOW) {    //if moving downward... ie. moving onto the endstop
			motorstate = states::STATIONARY;
			digitalWrite(reset_pin, LOW);   //disable motor
			downstop = LOW;   //update downstop
			digitalWrite(bot_LED_pin, HIGH);    //turn on the bot LED
		}
		else if (direction == HIGH) {   //if moving upward... ie. moving off of the endstop
			downstop = HIGH;    //update downstop
			digitalWrite(bot_LED_pin, LOW);    //turn off the bot LED
		}
		DebounceTimer.reset();
	}
}

void pulse()
{
	digitalWrite(step_pin, HIGH);
	delayMicroseconds(pulse_time);
	digitalWrite(step_pin, LOW);
	delayMicroseconds(min_delay_time);
	if (direction == HIGH) {
		steps += -1;
	}
	else if (direction == LOW) {
		steps += 1;
	}
}

void zeroing() //runs the motor upward into the endstop, where it calibrates steps to 0
{   
	if (upstop == LOW) {
		steps = 0;
		sc.println("Stepcount set to 0");
		motorstate = states::STATIONARY;
	}
	else {
		digitalWrite(direction_pin, HIGH);
		direction = HIGH;
		pulse();
	}
}

void readAnalogInputs() //Currently only tested with reading a potentiometer (pot), proof of concept for the APD
{
	static int32_t potV = 0;
	static int32_t nsamples = 0;
	if (!dataTimer.timedOut(true) || nsamples == 0) {
		potV += adc.read(scan_pot_chan);
		nsamples++;
	}
	else {
		if (nsamples) {
			potV /= nsamples;
		}
		floatPotV = 5 * float(potV) / 4095.0;
		potV = 0;
		nsamples = 0;
		sc.println(floatPotV);
	}
}
