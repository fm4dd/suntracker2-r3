/* ---------------------------------------------------- *
 * Suntracker2  mainboard-rev3 ledring.c 2019-07 @FM4DD *
 *                                                      *
 * This file has the functions to control the dualcolor *
 * 32 LED ring on the Displayboard v2.0, which connects *
 * to the Suntracker2 Base-Board v1.7.                  *
 * ---------------------------------------------------- */
#include "ledring.h" // local module for the 32 LED ring

/* ---------------------------------------------------- */
/* 4x IO Expanders for a total of 64 I/O ports          */
/* ---------------------------------------------------- */
Adafruit_MCP23017 mcp1;
Adafruit_MCP23017 mcp2;
Adafruit_MCP23017 mcp3;
Adafruit_MCP23017 mcp4;

/* ---------------------------------------------------- */
/* led_enable() initializes the 4 GPIO port expanders   */
/* ---------------------------------------------------- */
void ledRing::enable() {
  mcp1.begin(0);     /* Enable the MCP23017 module 1 */
  mcp2.begin(1);     /* Enable the MCP23017 module 2 */
  mcp3.begin(2);     /* Enable the MCP23017 module 3 */
  mcp4.begin(3);     /* Enable the MCP23017 module 4 */
  /* Configure all MCP23017 ports in LED output mode */
  for(int i=0; i<16; i++) {
    mcp1.pinMode(i, OUTPUT);
    mcp2.pinMode(i, OUTPUT);
    mcp3.pinMode(i, OUTPUT);
    mcp4.pinMode(i, OUTPUT);
    mcp1.digitalWrite(i, LOW);
    mcp2.digitalWrite(i, LOW);
    mcp3.digitalWrite(i, LOW);
    mcp4.digitalWrite(i, LOW);
  }
}

/* ---------------------------------------------------- */
/* all_ledoff() turns off 32 LED, regardless of state   */
/* ---------------------------------------------------- */
void ledRing::all_ledoff() {
  mcp1.writeGPIOAB(0x00); /* expander1 all green LED off */
  mcp2.writeGPIOAB(0x00); /* expander2 all green LED off */
  mcp3.writeGPIOAB(0x00); /* expander3 all red LED off   */
  mcp4.writeGPIOAB(0x00); /* expander3 all red LED off   */
}

/* ---------------------------------------------------- */
/* led_lightcheck() LED on/off tests for all 4 units:   */
/* Blink all ports. 0xFFFF lights both GPIOA and GPIOB  */
/* Setting 0xFF only lights A, 0x00FF lights up only B  */
/* ---------------------------------------------------- */
void ledRing::lightcheck() {
  mcp1.writeGPIOAB(0xFFFF); /* expander1 all green LED on  */
  delay(500);
  mcp1.writeGPIOAB(0x00);   /* expander1 all green LED off */
  mcp2.writeGPIOAB(0xFFFF); /* expander2 all green LED on  */
  mcp3.writeGPIOAB(0xFFFF); /* expander3 all red LED on    */
  delay(500);
  mcp2.writeGPIOAB(0x00);   /* expander2 all green LED off */
  mcp3.writeGPIOAB(0x00);   /* expander3 all red LED off   */
  mcp4.writeGPIOAB(0xFFFF); /* expander4 all red LED on    */
  delay(500);
  mcp4.writeGPIOAB(0x00);   /* expander4 all red LED off   */
}

// Below two functions are commented out because I have
// the same functions in the main file with additional
// code for OLED status display.
/* ----------------------------------------------------- */
/* Single LED walkthrough - Red: mcp1,2, port A,B        */
/* Runs a single red LED light around the circle.        */
/* ----------------------------------------------------- */
//void stepled_red(int millisec) {
//  all_ledoff();                          /* reset LEDs   */
//  for(int i=0; i<16; i++) {              /* IO expander1 */
//    mcp1.digitalWrite(i, HIGH);          /* next LED on  */
//    if(i>0) mcp1.digitalWrite(i-1, LOW); /* prev LED off */
//    delay(millisec);                     /* speed in ms  */
//  }
//  mcp1.digitalWrite(15, LOW);             /* last LED off */
//  for(int i=0; i<16; i++) {               /* IO expander2 */
//    mcp2.digitalWrite(i, HIGH);           /* next LED on  */
//    if(i>0) mcp2.digitalWrite(i-1, LOW);  /* prev LED off */
//    delay(millisec);                      /* speed in ms  */
//  }
//  mcp2.digitalWrite(15, LOW);             /* last LED off */
//}

/* ------------------------------------------------------ */
/* Single LED walkthrough - Green: mcp3,4, port C,D       */
/* Runs a single green LED light around the circle.       */
/* ------------------------------------------------------ */
//void stepled_green(int millisec) {
//  all_ledoff();                           /* reset LEDs   */
// for(int i=0; i<16; i++) {               /* IO expander3 */
//    mcp3.digitalWrite(i, HIGH);           /* next LED on  */
//    if(i>0) mcp3.digitalWrite(i-1, LOW);  /* prev LED off */
//    delay(millisec);                      /* speed in ms  */
//  }
//  mcp3.digitalWrite(15, LOW);
//  for(int i=0; i<16; i++) {               /* IO expander4 */
//    mcp4.digitalWrite(i, HIGH);           /* next LED on  */
//    if(i>0) mcp4.digitalWrite(i-1, LOW);  /* prev LED off */
//    delay(millisec);                      /* speed in ms  */
//  }

/* ------------------------------------------------------- */
/* Lightshow demonstration on a 32x 2-color LED ring       */
/* ------------------------------------------------------- */
void ledRing::lightshow(int millisec) {
  /* ------------------------------------------------------- */
  /* Pattern-1: light up one expander (16 LED), then remove  */
  /* one LED on either side until all LED are turned off.    */
  /* ------------------------------------------------------- */
  int decline[9] = { 0xFFFF, 0x7FFE, 0x3FFC, 0x1FF8, 0xFF0, 0x7E0, 0x3C0, 0x180, 0x0 };
  /* ------------------------------------------------------- */
  /* Pattern-2: Switch on every second LED on a expander and */
  /* and alternate the LED with it's neighbor LED.           */
  /* ------------------------------------------------------- */
  int alternate[2] = { 0xAAAA, 0x5555 };
  /* ------------------------------------------------------- */
  /* Pattern-3: In four steps light up 1, add 2, then add 3, */
  /* finally 4 LEDs: LED1-12-123-1234 on the 16 LED expander */
  /* ------------------------------------------------------- */
  int null2four[5] = { 0x0, 0x8888, 0xCCCC, 0xEEEE, 0xFFFF };
  /* ------------------------------------------------------- */
  /* Pattern-4: In eight steps light up 1, add 2, then add 3 */
  /* finally 8: 1-12-123-1234-12345-123456-1234567-12345678  */
  /* ------------------------------------------------------- */
  //int null2eight[9] = { 0x0, 0x8888, 0xCCCC, 0xEEEE, 0xFFFF };
  /* ------------------------------------------------------- */
  /* Pattern-5: Light up every 4th LED and move it forward.  */
  /* LED1-5-9-13, LED2-6-10-14, LED3-7-11-15, LED4-8-12-16   */
  /* ------------------------------------------------------- */
  int move4[4] = { 0x8888, 0x4444, 0x2222, 0x1111 };
  /* ------------------------------------------------------- */
  /* Pattern-6: Light up all 16 LED,turn one off, and move   */
  /* the one dark led from right to left side. We start with */
  /* 0xFFFF, loop (subtract), and stop at 0x7FFF.            */
  /* ------------------------------------------------------- */
 int move1off[17] = { 0xFFFF, 0xFFFE, 0xFFFD, 0xFFFB, 0xFFF7, 0xFFEF, 0xFFDF, 0xFFBF,
                      0xFF7F, 0xFEFF, 0xFDFF, 0xFBFF, 0xF7FF, 0xEFFF, 0xDFFF, 0xBFFF, 0x7FFF };
  /* ------------------------------------------------------- */
  /* Pattern7: In combination with Pattern-6, this moves an */
  /* alternate color in the dark spot, right to left side.   */
  /* ------------------------------------------------------- */
  int move1on[17] = { 0x0, 0x1, 0x2, 0x4, 0x8, 0x10, 0x20, 0x40, 0x80, 0x100,
                    0x200, 0x400, 0x800, 0x1000, 0x2000, 0x4000, 0x8000 };

  all_ledoff();                             /* reset LEDs   */
  /* ------------------------------------------------------- */
  /* Pattern-1 on green LEDs                                 */
  /* ------------------------------------------------------- */
  for(uint8_t i = 0; i < 9; i++) {
    mcp1.writeGPIOAB(decline[i]);
    mcp2.writeGPIOAB(decline[i]);
    delay(millisec);
  }
  /* ------------------------------------------------------- */
  /* Reverse Pattern-1 on red LEDs, turn decline to increase */
  /* ------------------------------------------------------- */
  for(uint8_t i = 8; i > 0; i--) {
    mcp1.writeGPIOAB(decline[i]);
    mcp2.writeGPIOAB(decline[i]);
    delay(millisec);
  }
  /* ------------------------------------------------------- */
  /*  Pattern-1 on red LEDs                                  */
  /* ------------------------------------------------------- */
  for(uint8_t i = 0; i < 9; i++) {
    mcp1.writeGPIOAB(decline[i]);
    mcp2.writeGPIOAB(decline[i]);
    delay(millisec);
  }
  delay(millisec * 4);
  /* ------------------------------------------------------- */
  /*  Pattern-1 on green LEDs                                */
  /* ------------------------------------------------------- */
  for(uint8_t i = 0; i < 9; i++) {
    mcp3.writeGPIOAB(decline[i]);
    mcp4.writeGPIOAB(decline[i]);
    delay(millisec);
  }
  /* ------------------------------------------------------- */
  /* Reverse Pattern-1 on greenLEDs, increase                */
  /* ------------------------------------------------------- */
  for(uint8_t i = 8; i > 0; i--) {
    mcp3.writeGPIOAB(decline[i]);
    mcp4.writeGPIOAB(decline[i]);
    delay(millisec);
  }
  /* ------------------------------------------------------- */
  /*  Pattern-1 on green LEDs                                */
  /* ------------------------------------------------------- */
  for(uint8_t i = 0; i < 9; i++) {
    mcp3.writeGPIOAB(decline[i]);
    mcp4.writeGPIOAB(decline[i]);
    delay(millisec);
  }
  delay(millisec * 4);
  /* ------------------------------------------------------- */
  /* Pattern-3: LED1-12-123-1234 for green LEDs              */
  /* ------------------------------------------------------- */
  for(uint8_t j = 0; j < 8; j++) {
    for(uint8_t i = 0; i < 5; i++) {
      mcp3.writeGPIOAB(null2four[i]);
      mcp4.writeGPIOAB(null2four[i]);
      delay(millisec);
    }
  }
  all_ledoff();                           /* reset LEDs   */
  /* ------------------------------------------------------- */
  /* Pattern 5: move 1 in 4 LED for red LEDs               */
  /* ------------------------------------------------------- */
  for(uint8_t j = 0; j < 8; j++) {
      for(uint8_t i = 0; i < 5; i++) {
      mcp1.writeGPIOAB(move4[i]);
      mcp2.writeGPIOAB(move4[i]);
      delay(millisec);
    }
  }
  all_ledoff();
  /* ------------------------------------------------------- */
  /* Pattern 6 and 7: move 1 green LED with all red LEDs on  */
  /* ------------------------------------------------------- */
    for(uint8_t i = 0; i < 17; i++) {
      mcp1.writeGPIOAB(move1off[i]);
      mcp2.writeGPIOAB(move1off[i]);
      mcp3.writeGPIOAB(move1on[i]);
      mcp4.writeGPIOAB(move1on[i]);
      delay(millisec);
    }
    for(uint8_t i = 0; i < 17; i++) {
      mcp1.writeGPIOAB(move1off[i]);
      mcp2.writeGPIOAB(move1off[i]);
      mcp3.writeGPIOAB(move1on[i]);
      mcp4.writeGPIOAB(move1on[i]);
      delay(millisec);
    }
    for(uint8_t i = 16; i > 0; i--) {
      mcp1.writeGPIOAB(move1off[i]);
      mcp2.writeGPIOAB(move1off[i]);
      mcp3.writeGPIOAB(move1on[i]);
      mcp4.writeGPIOAB(move1on[i]);
      delay(millisec);
    }
    for(uint8_t i = 16; i > 0; i--) {
      mcp1.writeGPIOAB(move1off[i]);
      mcp2.writeGPIOAB(move1off[i]);
      mcp3.writeGPIOAB(move1on[i]);
      mcp4.writeGPIOAB(move1on[i]);
      delay(millisec);
    }

    for(uint8_t i = 16; i > 0; i--) {
      mcp4.writeGPIOAB(move1off[i]);
      mcp3.writeGPIOAB(move1off[i]);
      mcp2.writeGPIOAB(move1on[i]);
      mcp1.writeGPIOAB(move1on[i]);
      delay(millisec);
    }
    for(uint8_t i = 16; i > 0; i--) {
      mcp4.writeGPIOAB(move1off[i]);
      mcp3.writeGPIOAB(move1off[i]);
      mcp2.writeGPIOAB(move1on[i]);
      mcp1.writeGPIOAB(move1on[i]);
      delay(millisec);
    }
    for(uint8_t i = 0; i < 17; i++) {
      mcp4.writeGPIOAB(move1off[i]);
      mcp3.writeGPIOAB(move1off[i]);
      mcp2.writeGPIOAB(move1on[i]);
      mcp1.writeGPIOAB(move1on[i]);
      delay(millisec);
    }
    for(uint8_t i = 0; i < 17; i++) {
      mcp4.writeGPIOAB(move1off[i]);
      mcp3.writeGPIOAB(move1off[i]);
      mcp2.writeGPIOAB(move1on[i]);
      mcp1.writeGPIOAB(move1on[i]);
      delay(millisec);
    }
  all_ledoff();
}
