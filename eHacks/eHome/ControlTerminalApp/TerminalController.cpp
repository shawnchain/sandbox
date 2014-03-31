/*
 * TerminalController.cpp
 *
 *  Created on: 2014-1-26
 *      Author: shawn
 */

#include "TerminalController.h"
#include <stdio.h>
#include <string.h>

#define OFF false
#define ON true

//namespace ehome {

TerminalController::TerminalController() {
	outputState = 0;
}

///////////////////////////////////////////////////////////
// STATUS LED
///////////////////////////////////////////////////////////
#ifdef BOARD_PROMINI
#define STATUS_LED_PIN A1 // Use p11 for PWM on 328 ProMini
#else
#define STATUS_LED_PIN 13 // Use p13 on 2560Mega
#endif

void TerminalController::statusTaskSetup() {
	// the LED pin
	pinMode(STATUS_LED_PIN, OUTPUT);
}
void TerminalController::statusTask() {
	// breath effect on status LED
	float val = (exp(sin(millis() / 2000.0 * PI)) - 0.36787944) * 108.0;
	analogWrite(STATUS_LED_PIN, val);

	//TODO more status logic goes here
}

///////////////////////////////////////////////////////////
// Main Task
///////////////////////////////////////////////////////////
const int OUT_NUMS = 4;
const int OUT_PIN[4] = {2,3,4,5};
const int IN_PIN[4] = {6,7,8,9};

bool TerminalController::getOutputState(int index) {
	return bitRead(outputState,index);
}

bool TerminalController::setOutputState(int pinIndex, bool on) {
	if(bitRead(outputState,pinIndex) != on){
		digitalWrite(OUT_PIN[pinIndex],on?HIGH:LOW);
		bitWrite(outputState,pinIndex,on);
	}
	return on;
}

void TerminalController::mainTaskSetup() {
	// enable the Output Pins
	for(int i = 0;i < OUT_NUMS;i++){
		pinMode(OUT_PIN[i],OUTPUT);
	}
	delay(100);

	// TODO - restore state
	setOutputState(0,ON);
}

static int mainCounter = 0;
static int currentOut = 0;
void TerminalController::mainTask() {
	// delay
	if(mainCounter++ < 10240){
		 return;
	}
	mainCounter = 0;

	// Loop on each pins and enable it
	setOutputState(currentOut,OFF);
	if(++currentOut == OUT_NUMS){
		currentOut = 0;
	}
	setOutputState(currentOut,ON);
}


///////////////////////////////////////////////////////////
// Debug Task
///////////////////////////////////////////////////////////
#if SERIAL_DEBUG
static unsigned int debugTaskCounter = 0;
void TerminalController::debugTaskSetup() {
	Serial.begin(9600);
}
void TerminalController::debugTask() {
	if (debugTaskCounter++ < 10240) {
		return;
	}
	debugTaskCounter = 0;

	// dump out
	char buf[128];
	sprintf(buf, "Out1:%d, Out2:%d, Out3:%d, Out4:%d\n",
			getOutputState(0), getOutputState(1),
			getOutputState(2), getOutputState(3));
	Serial.write(buf);
}
#endif

///////////////////////////////////////////////////////////
// Entry
///////////////////////////////////////////////////////////
void TerminalController::setup() {
#if SERIAL_DEBUG
	debugTaskSetup();
#endif
	statusTaskSetup();
	mainTaskSetup();
}

void TerminalController::loop() {
	mainTask();
	statusTask();
	debugTask();
}

//} /* namespace ehome */
