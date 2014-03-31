// Only modify this file to include
// - function definitions (prototypes)
// - include files
// - extern variable definitions
// In the appropriate section

#ifndef ControlTerminalApp_H_
#define ControlTerminalApp_H_

#include <Arduino.h>
//add your includes for the project ControlTerminalApp here

// Eliminate the GCC warning
#ifdef PROGMEM
#undef PROGMEM
#define PROGMEM __attribute__((section(".progmem.data")))
#endif

//end of add your includes here
#ifdef __cplusplus
extern "C" {
#endif
void loop();
void setup();
#ifdef __cplusplus
} // extern "C"
#endif

//add your function definitions for the project ControlTerminalApp here




//Do not add code below this line
#endif /* ControlTerminalApp_H_ */
