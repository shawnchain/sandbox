// Do not remove the include below
#include "ControlTerminalApp.h"
#include "TerminalController.h"

//using namespace ehome;

TerminalController tc;

// constants won't change. Used here to
// set pin numbers:
const int pin1 =  4;      // the number of the LED pin
const int pin2 =  5;      // the number of the LED pin

//The setup function is called once at startup of the sketch
void setup() {
	tc.setup();
}

// The loop function is called in an endless loop
void loop() {
	tc.loop();
}
