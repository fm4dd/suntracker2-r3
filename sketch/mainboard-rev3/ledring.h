/* ---------------------------------------------------- *
 * Suntracker2  mainboard-rev3 ledring.h 2019-07 @FM4DD *
 *                                                      *
 * This file has the functions to control the dualcolor *
 * 32 LED ring on the Displayboard v2.0, which connects *
 * to the Suntracker2 Base-Board v1.7.                  *
 * ---------------------------------------------------- */
#include <Arduino.h>           // Arduino default library
#include "Adafruit_MCP23017.h" // https://github.com/adafruit/Adafruit-MCP23017-Arduino-Library

class ledRing {
  public:
    void enable();
    void all_ledoff();
    void lightcheck();
    //void stepled_red(int millisec);
    //void stepled_green(int millisec);
    void lightshow(int millisec);
};
