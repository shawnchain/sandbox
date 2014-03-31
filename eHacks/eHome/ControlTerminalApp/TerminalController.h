/*
 * TerminalController.h
 *
 *  Created on: 2014-1-26
 *      Author: shawn
 */

#ifndef TERMINALCONTROLLER_H_
#define TERMINALCONTROLLER_H_

#include <arduino.h>

//namespace ehome {

#define SERIAL_DEBUG 1

/**
 * Terminal Controller
 * 终端控制器，负责强电的通断
 */
class TerminalController {
public:
	TerminalController();
	void setup();
	void loop();

private:
	// Tasks
	void statusTask();
	void statusTaskSetup();

	void mainTask();
	void mainTaskSetup();
	uint8_t outputState;

#if SERIAL_DEBUG
	void debugTask();
	void debugTaskSetup();
#endif

	bool getOutputState(int pin);
	bool setOutputState(int pin, bool on);
};


//} /* namespace ehome */
#endif /* TERMINALCONTROLLER_H_ */
