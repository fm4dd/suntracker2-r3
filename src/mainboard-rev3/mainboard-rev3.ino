/* ------------------------------------------------- *
 * Suntracker2  mainboard-rev3  June 2019 @ FM4DD    *
 *                                                   * 
 * This code controls Suntracker2 Base-Board v1.7 w. *
 * Displayboard v2.0 for solar azimuth path tracking *
 * Requires suncalc v1.2 generated solar positioning *
 * dataset stored in the root of the MKRZERO MicroSD *
 * ------------------------------------------------- */
#include <Wire.h>              // Arduino default 12C libary
#include <Arduino.h>           // Arduino default library
#include <SPI.h>               // Arduino default SPI library
#include <SD.h>                // Arduino default SD library
#include "LSM303.h"            // https://github.com/pololu/lsm303-arduino
#include "U8g2lib.h"           // https://github.com/olikraus/u8g2
#include "uRTCLib.h"           // https://github.com/Naguissa/uRTCLib
#include "ledring.h"           // local code module for the 32 LED ring
#include "oledimg.h"           // bitmap files for the two OLED displays

/* ------------------------------------------------- */
/* DEBUG enables debug output to the serial monitor  */
/* ------------------------------------------------- */
//#define DEBUG

/* ------------------------------------------------- */
/* SLOW and FAST set motor speed if stepper is used  */
/* ------------------------------------------------- */
#define SLOW 5000
#define FAST 500

/* ------------------------------------------------- */
/* SD card - Arduino MKRZero onboard card reader     */
/* ------------------------------------------------- */
Sd2Card card;
SdVolume volume;
SdFile root;
const int chipSelect = SDCARD_SS_PIN;
/* ------------------------------------------------- */
/* Magnetic field Sensor                             */
/* ------------------------------------------------- */
LSM303 compass;
/* ------------------------------------------------- */
/* 2x Oled Display - U8G2_R0 = orientation normal    */
/* ------------------------------------------------- */
U8G2_SSD1306_128X64_NONAME_F_HW_I2C oled1(U8G2_R0);
U8G2_SSD1306_128X64_NONAME_F_HW_I2C_2 oled2(U8G2_R0);

/* ------------------------------------------------- */
/* DS3231 Precision RTC clock                        */
/* ------------------------------------------------- */
uRTCLib rtc;

/* ------------------------------------------------- */
/* DisplayBoard LedRing initializer                  */
/* ------------------------------------------------- */
ledRing ring;

/* ---------------------------------------------------- */
/* 4x IO Expanders for a total of 64 I/O ports          */
/* ---------------------------------------------------- */
extern Adafruit_MCP23017 mcp1;
extern Adafruit_MCP23017 mcp2;
extern Adafruit_MCP23017 mcp3;
extern Adafruit_MCP23017 mcp4;

/* ------------------------------------------------- */
/* Global variable list                              */
/* ------------------------------------------------- */
int8_t hled = 0;             // heading LED (red)
int8_t oldhled = 33;         // 33 > max value 32, indicates "init"
int8_t aled = 0;             // azimuth LED (green)
int8_t oldaled = 33;         // 33 > max value 32, indicates "init"
int8_t rled = 0;             // sunrise LED (red)
int8_t oldrled = 33;         // 33 > max value 32, indicates "init"
int8_t sled = 0;             // sunset LED (red)
int8_t oldsled = 33;         // 33 > max value 32, indicates "init"
uint8_t risehour = 0;        // hour of sunrise
uint8_t risemin = 0;         // minute of sunrise
uint8_t transithour = 0;     // peak altitude hour
uint8_t transitmin = 0;      // peak altitude minute
uint8_t sethour = 0;         // sunset hour
uint8_t setmin = 0;          // sunset minute
float heading;               // North heading
double azimuth;              // sun azimuth angle, north->eastward
double zenith;               // zenith angle, substract from 90 to get altitude
uint16_t riseaz;             // sunrise azimuth, rounded to full degrees
int16_t transalt;            // transit altitude - solar noon peak altitude
uint16_t setaz;              // sunset azimuth, rounded to full degrees
boolean daylight;            // daylight flag, false = night
char lineStr[15];            // display output buffer, 14-chars + \0
char hledStr[5];             // display LED control "-B01"
char dirStr[6];              // display north angle "H263"
char aziStr[6];              // display azimuth angle "A134"
char timeStr[9];             // display time string "HH:MM:SS"
char dateStr[16];            // display date string "YYYY/MM/DD"
char riseStr[16];            // display string "sunrise: 06:45"
char setStr[16];             // display string "sunset:  18:10"
char binfile[13];            // daily sun data file yyyymmdd.bin
char srsfile[13];            // sunrise/sunset file srs-yyyy.bin
const uint8_t dip1  = 0;     // GPIO pin dipswitch-1 on GPIO-0
const uint8_t dip2  = 1;     // GPIO pin dipswitch-2 on GPIO-1
const uint8_t push1 = 2;     // GPIO pin pushbutton-1 on GPIO-2
const uint8_t push2 = 3;     // GPIO pin pushbutton-2 on GPIO-3
const uint8_t step1 = 4;     // GPIO pin stepper motor-1 step
const uint8_t dir1  = 5;     // GPIO pin stepper motor-1 direction
const uint8_t ms11  = 6;     // GPIO pin stepper motor-1 step size 1
const uint8_t ms21  = 7;     // GPIO pin stepper motor-1 step size 2
const uint8_t ena1  = 8;     // GPIO pin stepper motor-1 enable
const double mdecl = -7.583; // local magnetic declination
uint8_t dippos1 = 0;         // dipswitch 1 position (extra tests)
uint8_t dippos2 = 0;         // dipswitch 2 position (motor enable)
uint16_t m1cpos = 0;         // stepper motor-1 current step position (0..1599)
uint16_t m1tpos = 0;         // stepper motor-1 target step position (0..1599)
boolean m1move = false;      // stepper motor-1 "move in progress" flag
uint8_t opmode = 0;          // operations mode: 0 = normal, 1 = demo
uint16_t day_azi[1440];      // daily azimuth angle 0..360, 1min interval, 2.88KB
int16_t day_ele[1440];       // daily elevation angle -90 ..90, 1min interval 2.88KB

void setup() {
#ifdef DEBUG
  /* ------------------------------------------------- */
  /* Enable Serial Debug output                        */
  /* ------------------------------------------------- */
  Serial.begin(9600);
  while (!Serial);   // wait for serial port to connect.
  Serial.println("Serial Debug Start");
#endif
  /* ------------------------------------------------- */
  /* Set dipswitch GPIO ports as input                 */
  /* ------------------------------------------------- */
  pinMode(dip1, INPUT);
  pinMode(dip2, INPUT);
  dippos1 = digitalRead(dip1);
  dippos2 = digitalRead(dip2);    
  /* ------------------------------------------------- */
  /* Set pushbutton GPIO ports as input                */
  /* ------------------------------------------------- */
  pinMode(push1, INPUT);
  pinMode(push2, INPUT);
  /* ------------------------------------------------- */
  /* Pushbutton 1 (-) at startup triggers demo LOW=ON  */
  /* demo mode switches main loop from min to sec exec */
  /* ------------------------------------------------- */
  if(digitalRead(push1) == LOW)
  if(digitalRead(push1) == LOW) opmode = 1; // demo mode
  /* ------------------------------------------------- */
  /* Set motor-1 control pins as output                */
  /* ------------------------------------------------- */
  pinMode(step1,OUTPUT); 
  pinMode(dir1,OUTPUT);
  pinMode(ena1,OUTPUT);
  pinMode(ms11,OUTPUT);
  pinMode(ms21,OUTPUT);
  digitalWrite(ena1,HIGH); // Disable stepstick1
  /* ------------------------------------------------- */
  /* Enable I2C bus                                    */
  /* ------------------------------------------------- */
  Wire.begin();
  /* ------------------------------------------------- */
  /* Enable the OLED graphics display 14char line font */
  /* ------------------------------------------------- */
  oled1.setI2CAddress(0x3C * 2);
  oled2.setI2CAddress(0x3D * 2);
  oled1.begin();
  oled2.begin();
  oled1.setFont(u8g2_font_t0_17_mr);
  oled2.setFont(u8g2_font_t0_17_mr);
  oled1.setFontMode(0);
  oled2.setFontMode(0);
  
  oled1.drawXBMP(0,0, 64, 32, arduLogo);
  oled1.drawStr(67, 14, "MKRZERO");
  oled1.drawStr(67, 29, "Init OK");
  oled1.sendBuffer();

  oled2.drawXBMP(0,0, 64, 32, arduLogo);
  oled2.drawStr(67, 14, "OLED-#2");
  oled2.drawStr(67, 29, "Init OK");
  snprintf(lineStr, sizeof(lineStr), "2xOLED display");
  oled2.drawStr(0, 47, lineStr);
  snprintf(lineStr, sizeof(lineStr), "enabled");
  oled2.drawStr(0, 63, lineStr);
  oled2.sendBuffer();
  delay(2000);
  /* ------------------------------------------------- */
  /* 1st dip switch enables extended selftests, LOW=ON */
  /* ------------------------------------------------- */
  if(dippos1 == LOW) {
    oled1.drawStr(67, 14, "Modules");
    oled1.drawStr(67, 29, "Testrun");
    /* ------------------------------------------------- */
    /* Identify I2C devices                              */
    /* 0x1D = MM-TXS05 (LSM303D)                         */
    /* 0x20, 0x21, 0x22, 0x23 = MCP23017 1-4             */
    /* 0x3C, 0x3D = SD1306 OLED                          */
    /* 0x68 = DS3231 RTC                                 */
    /* ------------------------------------------------- */
    const int size = 8;
    byte addr[8] = { 0x1D, 0x20, 0x21, 0x22, 0x23, 0x3C, 0x3D, 0x68 };
    int i;
    byte error;
    for(i = 0; i<size; i++ ) {
      /* ------------------------------------------------- */
      /* The i2c_scanner uses the Write.endTransmisstion   */
      /* return value to see if device exists at the addr  */
      /* ------------------------------------------------- */
      snprintf(lineStr, sizeof(lineStr), "I2C check %02x  ", addr[i]);
      oled1.drawStr(0, 47, lineStr);
      oled1.sendBuffer();
      Wire.beginTransmission(addr[i]);
      error = Wire.endTransmission();

      if (error == 0){
        snprintf(lineStr, sizeof(lineStr), "%02x Response OK", addr[i]);
        oled1.drawStr(0, 63, lineStr);
        oled1.sendBuffer();
      }
      else if (error==4) {
        snprintf(lineStr, sizeof(lineStr), "%02x Error      ", addr[i]);
        oled1.drawStr(0, 63, lineStr);
        oled1.sendBuffer();
      }
      else {
        snprintf(lineStr, sizeof(lineStr), "%02x Not Found  ", addr[i]);
        oled1.drawStr(0, 63, lineStr);
        oled1.sendBuffer();
      }
      delay(1000);
    }
    delay(2000);
  } // end dippos != 1, skip extended selftest
  /* ------------------------------------------------- */
  /* Enable the SD card module                         */
  /* ------------------------------------------------- */
  oled1.drawStr(0, 47, "Init SD card: ");
  oled1.sendBuffer();
  if (!card.init(SPI_HALF_SPEED, chipSelect)) {
    oled1.drawStr(0, 63, "SD card FAIL  ");
    while(1);
  } else {
    switch (card.type()) {
      case SD_CARD_TYPE_SD1:
        Serial.println("SD1");
        oled1.drawStr(0, 63, "SD1 card OK   ");
        break;
      case SD_CARD_TYPE_SD2:
        oled1.drawStr(0, 63, "SD2 card OK   ");
        break;
      case SD_CARD_TYPE_SDHC:
        oled1.drawStr(0, 63, "SDHC card OK  ");
        break;
      default:
        oled1.drawStr(0, 63, "Unknown card  ");
    }
  }
  SD.begin(chipSelect); 
  oled1.sendBuffer();
  delay(1500);
  oled1.clearBuffer();
  
  if(dippos1 == LOW) {
    /* ------------------------------------------------- */
    /* Check File read from SD: "dset.txt"               */
    /* ------------------------------------------------- */
    File dset = SD.open("DSET.TXT");
    if(dset) {
      oled1.drawStr(0, 14, "Read file dset.txt");
      unsigned long fsize = dset.size();
      snprintf(lineStr, sizeof(lineStr), "Size %ld bytes", fsize);
      oled1.drawStr(0, 30, lineStr);
      oled1.sendBuffer();
      delay(2000);
      while (dset.available()) {
        oled1.drawStr(0, 47, "              ");
        oled1.drawStr(0, 63, "              ");
        oled1.sendBuffer();
        dset.readStringUntil(':').toCharArray(lineStr,15);
        oled1.drawStr(0, 47, lineStr);
        dset.read(lineStr, 1); // skip over one space
        dset.readStringUntil('\n').toCharArray(lineStr,15);
        oled1.drawStr(0, 63, lineStr);
        oled1.sendBuffer();
        delay(1000);
      }
    } else {
      oled1.drawStr(0, 14, "Fail open dset.txt");
      oled1.sendBuffer();
    }
    dset.close();
    delay(2000);
  } // end dippos != 1, skip extended selftest
  oled1.clearBuffer();
  /* ------------------------------------------------- */
  /* Enable the DS3231 RTC clock module                */
  /* ------------------------------------------------- */
  oled1.drawStr(0, 14, "Init DS3231:");
  oled1.sendBuffer();
  rtc.set_rtc_address(0x68);
  rtc.set_model(URTCLIB_MODEL_DS3231);
  rtc.refresh();
  /* ------------------------------------------------- */
  /* To set the initial time, enable the line below    */
  /* ------------------------------------------------- */
  //rtc.set sec,min,hour,dayOfWeek,dayOfMonth,month,year
  //rtc.set(0, 51, 0, 0, 16, 6, 19);
  snprintf(lineStr, sizeof(lineStr), "%d/%02d/%02d %02d:%02d",
  rtc.year(),rtc.month(),rtc.day(),rtc.hour(),rtc.minute());
  oled1.drawStr(0, 30, lineStr);
  oled1.sendBuffer();
  delay(1500);
  oled1.clearBuffer();
  /* ------------------------------------------------- */
  /* Enable the LSM303 Compass sensor module           */
  /* 1. Sunhayato MM-TXS05 SA0 default I2C addr (0x1D) */
  /* compass.init(LSM303::device_D,LSM303::sa0_high);  */
  /* 2. Adafruit LSM303DLHC module (PROD-ID 1120) use: */
  /* compass.init(LSM303::device_DLHC,LSM303::sa0_high);
  /* init() without args tries to determine chip type  */
  /* ------------------------------------------------- */
  oled1.drawStr(0, 14, "Init Compass:");
  oled1.sendBuffer();
  compass.init(LSM303::device_D,LSM303::sa0_high); /* Sunhayato MM-TXS05 */
  //compass.init(); /* Enable the LSM303DLC module autoconfig mode  */
  compass.enableDefault();
  /* LSM303D Calibration values, see Calibrate example */
  compass.m_min = (LSM303::vector<int16_t>){-2461, -2517, -2943};
  compass.m_max = (LSM303::vector<int16_t>){+1762, +2254, +1969};
  oled1.drawStr(0, 30, "LSM303D OK");
  oled1.sendBuffer();
  delay(1500);
  oled1.clearBuffer();
  /* ------------------------------------------------- */
  /* Enable the IO port expansion modules. I2C address */
  /* is given to begin() as parameter per A-pin values */
  /* 0 = 0x20, 1 = 0x21, 2 = 0x22, 3 = 0x23, ... max 7 */
  /* ------------------------------------------------- */
  oled1.drawStr(0, 14, "Init MCP23017:");
  oled1.sendBuffer();
  ring.enable();
  oled1.drawStr(0, 30, "4xMCP23017 OK");
  oled1.sendBuffer();
  delay(500);

  /* ------------------------------------------------- */
  /* LED ring testing                                  */
  /* ------------------------------------------------- */
  oled1.drawStr(0, 47, "Test 32x LED");
  oled1.sendBuffer();
  if(dippos1 == LOW) {
    ring.lightcheck();
    /* ----------------------------------------------- */
    /* Single LED walkthrough                          */
    /* ----------------------------------------------- */
    stepled_red(oled2, 20);
    stepled_green(oled1, 20);
    stepled_red(oled2, 20);
    stepled_green(oled1, 20);
  }
  oled1.drawStr(0, 63, "Lightshow ON");
  oled1.sendBuffer();  
  ring.lightshow(60);
  /* ------------------------------------------------- */
  /* If demo mode was requested, display note on OLED2 */
  /* ------------------------------------------------- */
  if(opmode == 1) {
    snprintf(lineStr, sizeof(lineStr), "              ");
    oled2.drawStr(0, 47, lineStr);
    oled2.setFont(u8g2_font_inr19_mr);
    snprintf(lineStr, sizeof(lineStr), "DEMO ON");
    oled2.drawStr(0, 63, lineStr);
    oled2.sendBuffer();
  }
  /* ------------------------------------------------- */
  /* 2nd dip switch enables the motor function, LOW=ON */
  /* ------------------------------------------------- */
  if(dippos2 == LOW) {
    /* ----------------------------------------------- */
    /* Motor1 testing                                  */
    /* ----------------------------------------------- */
    oled1.drawStr(0, 47, "Test Stepper-1");
    oled1.drawStr(0, 63, "              ");
    oled1.sendBuffer();
    motor1turn(2); /* test Azimuth motor1 for 2x times */
    oled1.drawStr(0, 63, "Complete");
    oled1.sendBuffer();
    delay(1000);
    m1cpos = 0;    /* Set motor1 current position = D1 */
    m1cpos = 0;    /* Set motor1 target position = D1  */
  }
  /* ------------------------------------------------- */
  /* Read sunrise sunset time from file                */
  /* ------------------------------------------------- */
  snprintf(srsfile, sizeof(srsfile), "SRS-20%02d.BIN", rtc.year());
  get_suntime(srsfile);
  snprintf(riseStr, sizeof(riseStr), "rise %02d:%02d %03d",
           risehour,risemin, riseaz);
  snprintf(setStr, sizeof(setStr), "set  %02d:%02d %03d",
           sethour, setmin, setaz);
  /* ------------------------------------------------- */
  /* Set daily data file name per RTC current date     */
  /* ------------------------------------------------- */
  snprintf(binfile, sizeof(binfile), "20%02d%02d%02d.BIN", 
             rtc.year(),rtc.month(),rtc.day());
  /* ------------------------------------------------- */
  /* Check for demo mode, output demo to OLED1 display */
  /* ------------------------------------------------- */
  if(opmode == 1) {
    uint16_t i = 0;
    uint8_t hr = 0;
    uint8_t mi = 0;
    fill_dayarray(binfile);
    oled1.clearBuffer();
    while(i < 1440) {       
      /* ------------------------------------------------- */
      /* 1st line show date and 'D'/'N' daytime/night flag */
      /* ------------------------------------------------- */
      snprintf(lineStr, sizeof(lineStr), "20%02d/%02d/%02d: ", 
               rtc.year(),rtc.month(),rtc.day());
      oled1.drawStr(0, 16, lineStr);
      if(day_ele[i] >= 0) { daylight = true; oled1.drawStr(117, 16, "D"); }
      else { daylight = false; oled1.drawStr(117, 16, "N"); }
      /* ------------------------------------------------- */
      /* 2nd line calculate and show time                  */
      /* ------------------------------------------------- */
      mi++;
      if(mi == 60) { hr++; mi = 0; }
      if(hr == 24) break;
      snprintf(lineStr, sizeof(lineStr), "Time:    %02d:%02d", hr, mi);
      oled1.drawStr(0, 30, lineStr);
      /* ------------------------------------------------- */
      /* 3rd line show azimuth angle                       */
      /* ------------------------------------------------- */
      snprintf(lineStr, sizeof(lineStr), "Azimuth:   %03d", day_azi[i]);
      oled1.drawStr(0, 47, lineStr);
      /* ------------------------------------------------- */
      /* 4th line show elevation angle                     */
      /* ------------------------------------------------- */
      snprintf(lineStr, sizeof(lineStr), "Elevation: %03d", day_ele[i]);
      oled1.drawStr(0, 63, lineStr);
      oled1.sendBuffer();
      /* ------------------------------------------------- */
      /* Aquire compass sensor data                        */
      /* ------------------------------------------------- */
      compass.read();
      heading = compass.heading() + mdecl;
      /* ------------------------------------------------- */
      /* print North heading to OLED2                      */
      /* ------------------------------------------------- */
      //snprintf(dirStr, sizeof(dirStr), "H%03d", (int)round((heading) * 10 / 10.0));
      //oled2.setFont(u8g2_font_inr19_mr);
      //oled2.drawStr(0,26, dirStr);
      /* ------------------------------------------------- */
      /* Convert heading angle to LED range value (0..32)  */   
      /* ------------------------------------------------- */
      hled = (uint8_t)round((heading / 11.25) * 10 / 10.0);
      aled = (uint8_t)round((day_azi[i] / 11.25) * 10 / 10.0);
      rled = (uint8_t)round((riseaz / 11.25) * 10 / 10.0);
      sled = (uint8_t)round((setaz / 11.25) * 10 / 10.0);
      /* ------------------------------------------------- */
      /* If 32 is returned, it should roll over to 0       */   
      /* ------------------------------------------------- */
      if(hled > 31) hled = 0;
      if(aled > 31) aled = 0;
      if(rled > 31) rled = 0;
      if(sled > 31) sled = 0;
      /* ------------------------------------------------- */
      /* Because LED-1 is 90 degrees offset, we compensate */
      /* ------------------------------------------------- */
      hled = hled + 24; if(hled > 31) hled = hled - 32;
      /* ------------------------------------------------- */
      /* Reverse counting to keep the LED aligned to North */
      /* ------------------------------------------------- */
      hled = 32 - hled; if(hled > 31) hled = 0;
      /* ------------------------------------------------- */
      /* Remaining LED follow heading                      */
      /* ------------------------------------------------- */
      aled = hled + aled; if(aled > 31) aled = aled - 32;
      rled = hled + rled; if(rled > 31) rled = rled - 32;
      sled = hled + sled; if(sled > 31) sled = sled - 32;
      /* ------------------------------------------------- */
      /* Set heading LED                                   */
      /* ------------------------------------------------- */
      if (hled != oldhled) {
        if(oldhled < 16) mcp1.digitalWrite(oldhled, LOW);
        else if(oldhled < 32) mcp2.digitalWrite((oldhled-16), LOW);
        if(hled < 16) mcp1.digitalWrite(hled, HIGH);
        if(hled > 15) mcp2.digitalWrite((hled-16), HIGH);
        oldhled = hled;
      }
      /* ------------------------------------------------- */
      /* Set azimuth LED                                   */
      /* ------------------------------------------------- */
      if (aled != oldaled) {
        if(oldaled < 16) mcp3.digitalWrite(oldaled, LOW);
        else if(oldaled < 32) mcp4.digitalWrite((oldaled-16), LOW);
        if(aled < 16) mcp3.digitalWrite(aled, HIGH);
        if(aled > 15) mcp4.digitalWrite((aled-16), HIGH);
        oldaled = aled;
      }
      /* ------------------------------------------------- */
      /* Set sunrise LED                                   */
      /* ------------------------------------------------- */
      if (rled != oldrled) {
        if(oldrled < 16) mcp1.digitalWrite(oldrled, LOW);
        else if(oldrled < 32) mcp2.digitalWrite((oldrled-16), LOW);
        if(rled < 16) mcp1.digitalWrite(rled, HIGH);
        if(rled > 15) mcp2.digitalWrite((rled-16), HIGH);
        oldrled = rled;
      }
      /* ------------------------------------------------- */
      /* Set sunset LED                                    */
      /* ------------------------------------------------- */
      if (sled != oldsled) {
        if(oldsled < 16) mcp1.digitalWrite(oldsled, LOW);
        else if(oldsled < 32) mcp2.digitalWrite((oldsled-16), LOW);
        if(sled < 16) mcp1.digitalWrite(sled, HIGH);
        if(sled > 15) mcp2.digitalWrite((sled-16), HIGH);
        oldsled = sled;
      }
      /* ------------------------------------------------- */
      /* motor1 target position follows azimuth LED value  */
      /* if 2nd dip switch enabled the motor. DIP2: LOW=ON */
      /* ------------------------------------------------- */
      if(dippos2 == LOW) {
        if(daylight) {
          if(m1move == false) {
            m1tpos = aled * 50;
            if(m1tpos != m1cpos) {
              motor1adjust(m1tpos, SLOW);
            }
          }
        }
        else {                /* nightfall, move motor-1 home */
          if(m1move == false) {
            if(m1tpos != 0) { /* are we already parked at D1? */
              m1tpos = 0;     /* set D1 home position until sunrise */
              motor1adjust(m1tpos, SLOW);
            }
          }
        }
      }
      /* ------------------------------------------------- */
      /* loop activities complete, increment loop counter  */     
      /* Set the loop speed. 60000 = 60s = normal time     */
      /* 100 = 1/10s intervals, 1440*0.1sec/60 = 2:24mins  */
      /* ------------------------------------------------- */
      delay(50);
      i++;
    }
    /* ------------------------------------------------- */
    /* After the DEMO turn off all LEDs and reset values */
    /* ------------------------------------------------- */
    mcp1.writeGPIOAB(0x00);
    mcp2.writeGPIOAB(0x00);
    mcp3.writeGPIOAB(0x00);
    mcp4.writeGPIOAB(0x00);
    hled = 0;             // heading LED (red)
    oldhled = 33;         // 33 > max value 32, indicates "init"
    aled = 0;             // azimuth LED (green)
    oldaled = 33;         // 33 > max value 32, indicates "init"
    rled = 0;             // sunrise LED (red)
    oldrled = 33;         // 33 > max value 32, indicates "init"
    sled = 0;             // sunset LED (red)
    oldsled = 33;         // 33 > max value 32, indicates "init"
  }
  delay(4000);
  /* ------------------------------------------------- */
  /* Intitialize azimuth/elevation position from file  */
  /* ------------------------------------------------- */
  solar_position(binfile);
  oled1.clearBuffer();
  oled2.clearBuffer();
} // end setup

void loop() {
  /* ------------------------------------------------- */
  /* Aquire time                                       */
  /* Every new minute read solar position data         */
  /* ------------------------------------------------- */
  rtc.refresh();
  if(rtc.second() == 0) {
    if(rtc.minute() == 0) {
      snprintf(binfile, sizeof(binfile), "20%d%02d%02d.BIN", 
                rtc.year(),rtc.month(),rtc.day());
      if(rtc.day() == 1) {
        snprintf(srsfile, sizeof(srsfile), "SRS-20%d.BIN", 
                rtc.year());
      }
      get_suntime(srsfile);
      snprintf(riseStr, sizeof(riseStr), "rise %02d:%02d %03d",
               risehour,risemin, riseaz);
      snprintf(setStr, sizeof(setStr), "set  %02d:%02d %03d",
               sethour, setmin, setaz);
    }
    solar_position(binfile);
    display_daysymbol(oled2);
  }
  /* ------------------------------------------------- */
  /* Show Azimuth and sunrise/sunset to OLED1          */
  /* ------------------------------------------------- */
  snprintf(aziStr, sizeof(aziStr), "A%03d", (int)round((azimuth) * 10 / 10.0));
  oled1.setFont(u8g2_font_inr19_mr);
  oled1.drawStr(0,26, aziStr);
  oled1.setFont(u8g2_font_t0_17_mr);
  oled1.setCursor(0, 47);
  oled1.print(riseStr);
  oled1.setCursor(0, 63);
  oled1.print(setStr);
  /* ------------------------------------------------- */
  /* Show time on OLED2                                */
  /* ------------------------------------------------- */
  snprintf(dateStr, sizeof(dateStr), "20%d/%02d/%02d",
  rtc.year(),rtc.month(),rtc.day());
  snprintf(timeStr, sizeof(timeStr), "%02d:%02d:%02d",
  rtc.hour(),rtc.minute(),rtc.second());
  oled2.setFont(u8g2_font_t0_17_mr);
  oled2.setCursor(0, 47);
  oled2.print(dateStr);
  oled2.setCursor(0, 63);
  oled2.print(timeStr);

  /* ------------------------------------------------- */
  /* Aquire compass sensor data                        */
  /* ------------------------------------------------- */
  compass.read();
  
  /* ------------------------------------------------- */
  /* If the sensor board orientation is unaligned, set */
  /* compass.heading((LSM303::vector<int>){0,1,0}) for */
  /* pointing along the Y-axis for example. Remove if  */
  /* the X-axis is the point of reference on a LSM303D */
  /* ------------------------------------------------- */
  /* Because magnetic north != real north, we adjust:  */
  /* Calculate magnetic declination and convergence    */
  /* to determine true North from magnetic North       */ 
  /* https://www.ngdc.noaa.gov/geomag/WMM/soft.shtml   */
  /* ------------------------------------------------- */
  //heading = compass.heading((LSM303::vector<int>){0,-1,0}) + mdecl;
  heading = compass.heading() + mdecl;
  
  /* ------------------------------------------------- */
  /* print North heading to OLED2                      */
  /* ------------------------------------------------- */
  snprintf(dirStr, sizeof(dirStr), "H%03d", (int)round((heading) * 10 / 10.0));
  oled2.setFont(u8g2_font_inr19_mr);
  oled2.drawStr(0,26, dirStr);

  /* ------------------------------------------------- */
  /* Convert heading angle to LED range value (0..32)  */   
  /* ------------------------------------------------- */
  hled = (uint8_t)round((heading / 11.25) * 10 / 10.0);
  aled = (uint8_t)round((azimuth / 11.25) * 10 / 10.0);
  rled = (uint8_t)round((riseaz / 11.25) * 10 / 10.0);
  sled = (uint8_t)round((setaz / 11.25) * 10 / 10.0);
  /* ------------------------------------------------- */
  /* If 32 is returned, it should roll over to 0       */   
  /* ------------------------------------------------- */
  if(hled > 31) hled = 0;
  if(aled > 31) aled = 0;
  if(rled > 31) rled = 0;
  if(sled > 31) sled = 0;
  /* ------------------------------------------------- */
  /* Because LED-1 is 90 degrees offset, we compensate */
  /* ------------------------------------------------- */
  hled = hled + 24; if(hled > 31) hled = hled - 32;
  /* ------------------------------------------------- */
  /* Reverse counting to keep the LED aligned to North */
  /* ------------------------------------------------- */
  hled = 32 - hled; if(hled > 31) hled = 0;
  /* ------------------------------------------------- */
  /* Remaining LED follow heading                      */
  /* ------------------------------------------------- */
  aled = hled + aled; if(aled > 31) aled = aled - 32;
  rled = hled + rled; if(rled > 31) rled = rled - 32;
  sled = hled + sled; if(sled > 31) sled = sled - 32;

#ifdef DEBUG
   var2serial();
#endif 

  /* ------------------------------------------------- */
  /* motor1 target position follows azimuth LED value  */
  /* if 2nd dip switch enabled the motor. DIP2: LOW=ON */
  /* ------------------------------------------------- */
  if(dippos2 == LOW) {
    if(daylight) {
      if(m1move == false) {
        m1tpos = aled * 50;
        if(m1tpos != m1cpos) {
          motor1adjust(m1tpos, SLOW);
        }
      }
    }
    else {                /* nightfall, move motor-1 home */
      if(m1move == false) {
        if(m1tpos != 0) { /* are we already parked at D1? */
          m1tpos = 0;     /* set D1 home position until sunrise */
          motor1adjust(m1tpos, SLOW);
        }
      }
    }
  }

  /* ------------------------------------------------- */
  /* motor1 home when push button 1 is pressed  LOW=ON */
  /* ------------------------------------------------- */
  if(dippos2 == LOW) {
    if(m1move == false) {
      if(digitalRead(push1) == LOW) { motor1home(FAST); }
      if(digitalRead(push2) == LOW) { motor1led(FAST); }
    }
  }
  /* ------------------------------------------------- */
  /* Set heading LED                                   */
  /* ------------------------------------------------- */
  if (hled != oldhled) {
    if(oldhled < 16) mcp1.digitalWrite(oldhled, LOW);
    else if(oldhled < 32) mcp2.digitalWrite((oldhled-16), LOW);
    if(hled < 16) mcp1.digitalWrite(hled, HIGH);
    if(hled > 15) mcp2.digitalWrite((hled-16), HIGH);
    display_hdgled(oled2, oldhled, hled);
    oldhled = hled;
  }
  
  /* ------------------------------------------------- */
  /* Set azimuth LED                                   */
  /* ------------------------------------------------- */
  if (aled != oldaled) {
    if(oldaled < 16) mcp3.digitalWrite(oldaled, LOW);
    else if(oldaled < 32) mcp4.digitalWrite((oldaled-16), LOW);
    if(aled < 16) mcp3.digitalWrite(aled, HIGH);
    if(aled > 15) mcp4.digitalWrite((aled-16), HIGH);
    display_aziled(oled1, oldaled, aled);
    oldaled = aled;
  }

  /* ------------------------------------------------- */
  /* Set sunrise LED                                   */
  /* ------------------------------------------------- */
  if (rled != oldrled) {
    if(oldrled < 16) mcp1.digitalWrite(oldrled, LOW);
    else if(oldrled < 32) mcp2.digitalWrite((oldrled-16), LOW);
    if(rled < 16) mcp1.digitalWrite(rled, HIGH);
    if(rled > 15) mcp2.digitalWrite((rled-16), HIGH);
    oldrled = rled;
  }
  
  /* ------------------------------------------------- */
  /* Set sunset LED                                   */
  /* ------------------------------------------------- */
  if (sled != oldsled) {
    if(oldsled < 16) mcp1.digitalWrite(oldsled, LOW);
    else if(oldsled < 32) mcp2.digitalWrite((oldsled-16), LOW);
    if(sled < 16) mcp1.digitalWrite(sled, HIGH);
    if(sled > 15) mcp2.digitalWrite((sled-16), HIGH);
    oldsled = sled;
  }

  oled1.updateDisplay();
  oled2.updateDisplay();
  delay(50);
}

/* ------------------------------------------------- */
/* get solar postion record for current time.        */
/* ------------------------------------------------- */
void solar_position(char *file) {
  uint8_t linebuf[19];
  File bin = SD.open(file);
  while (bin.available()) {
    bin.read(linebuf, sizeof(linebuf));
    if(linebuf[0] == rtc.hour() && linebuf[1] == rtc.minute()) {
      memcpy(&azimuth, &linebuf[3], sizeof(double));
      memcpy(&zenith, &linebuf[11], sizeof(double));
      if(linebuf[2] == 1) daylight = 1;
      else daylight = 0;
      break;
    }
  }
  bin.close();
}

/* ------------------------------------------------- */
/* get sunrise/transit/sunset record for current day */
/* ------------------------------------------------- */
void get_suntime(char *file) {
  uint8_t linebuf[14];
  File bin = SD.open(file);
  while (bin.available()) {
    bin.read(linebuf, sizeof(linebuf));
    if(linebuf[0] == rtc.month() && linebuf[1] == rtc.day()) {
      risehour=linebuf[2];
      risemin=linebuf[3];
      memcpy(&riseaz, &linebuf[4], sizeof(uint16_t));
      transithour=linebuf[6];
      transitmin=linebuf[7];
      memcpy(&transalt, &linebuf[8], sizeof(int16_t));
      sethour=linebuf[10];
      setmin=linebuf[11];
      memcpy(&setaz, &linebuf[12], sizeof(uint16_t));
      break;
    }
  }
  bin.close();
}

/* ------------------------------------------------- */
/* Show azimuth LED control data on OLED             */
/* ------------------------------------------------- */
void display_aziled(U8G2_SSD1306_128X64_NONAME_F_HW_I2C oled, int old, int now) {
  char aledStr[5];
  if(old < 16)
    snprintf(aledStr, sizeof(aledStr), "-C%02d", old);
  else if(old < 32)
    snprintf(aledStr, sizeof(aledStr), "-D%02d", old-16);
  else snprintf(aledStr, sizeof(aledStr), "init");
  oled.setFont(u8g2_font_t0_17_mr);
  oled.drawStr(82,14, aledStr);
  if(now < 16)
    snprintf(aledStr, sizeof(aledStr), "+C%02d", now);
  if(now > 15)
    snprintf(aledStr, sizeof(aledStr), "+D%02d", (now-16));
  oled.drawStr(82,30, aledStr);
  oled.sendBuffer();
}
  /* ------------------------------------------------- */
  /* Show north heading LED control data on OLED       */
  /* ------------------------------------------------- */
void display_hdgled(U8G2_SSD1306_128X64_NONAME_F_HW_I2C_2 oled, int old, int now) {
  char hledStr[5];
  if(old < 16)
    snprintf(hledStr, sizeof(hledStr), "-A%02d", old);
  else if(old < 32)
    snprintf(hledStr, sizeof(hledStr), "-B%02d", old-16);
  else snprintf(hledStr, sizeof(hledStr), "init");
  oled.setFont(u8g2_font_t0_17_mr);
  oled.drawStr(82,14, hledStr);
  if(now < 16)
    snprintf(hledStr, sizeof(hledStr), "+A%02d", now);
  if(now > 15)
    snprintf(hledStr, sizeof(hledStr), "+B%02d", (now-16));
  oled.drawStr(82,30, hledStr);
  oled.sendBuffer();
}

/* ------------------------------------------------- */
/* Single LED walkthrough - Red: mcp1,2, port A,B    */
/* ------------------------------------------------- */
void stepled_red(U8G2_SSD1306_128X64_NONAME_F_HW_I2C_2 oled, int millisec) {
  oled.setFont(u8g2_font_t0_17_mr);
  oled.setFontMode(0);
  char aledStr[5];
  mcp1.writeGPIOAB(0x00);
  mcp2.writeGPIOAB(0x00);
  oled.clearBuffer();
  oled.drawStr(0, 14, "MCP23017-A:");
  oled.drawStr(0, 30, "Port:");
  oled.sendBuffer();
  for(int i=0; i<16; i++) {
    mcp1.digitalWrite(i, HIGH);
    snprintf(aledStr, sizeof(aledStr), "+A%02d", i);
    oled.drawStr(48, 30, aledStr);
    if(i>0){
      mcp1.digitalWrite(i-1, LOW);
      snprintf(aledStr, sizeof(aledStr), "-A%02d", i-1);
      oled.drawStr(88, 30, aledStr);
    }
    oled.updateDisplay();
    delay(millisec);
  }
  mcp1.digitalWrite(15, LOW);
  oled.drawStr(0, 30, "Port A done   ");
  oled.sendBuffer();
  oled.drawStr(0, 47, "MCP23017-B:");
  oled.drawStr(0, 63, "Port:");
  oled.sendBuffer();
  for(int i=0; i<16; i++) {
    mcp2.digitalWrite(i, HIGH);
    snprintf(aledStr, sizeof(aledStr), "+B%02d", i);
    oled.drawStr(48, 63, aledStr);
    if(i>0){
      mcp2.digitalWrite(i-1, LOW);
      snprintf(aledStr, sizeof(aledStr), "-B%02d", i-1);
      oled.drawStr(88, 63, aledStr);
    }
    oled.sendBuffer();
    delay(millisec);
  }
  mcp2.digitalWrite(15, LOW);
  oled.drawStr(0, 63, "Port B done   ");
  oled.sendBuffer();
}

/* ------------------------------------------------- */
/* Single LED walkthrough - Green: mcp3,4, port C,D  */
/* ------------------------------------------------- */
void stepled_green(U8G2_SSD1306_128X64_NONAME_F_HW_I2C oled, int millisec) {
  oled.setFont(u8g2_font_t0_17_mr);
  oled.setFontMode(0);
  char aledStr[5];
  mcp3.writeGPIOAB(0x00);
  mcp4.writeGPIOAB(0x00);
  oled.clearBuffer();
  oled.drawStr(0, 14, "MCP23017-C:");
  oled.drawStr(0, 30, "Port:");
  oled.updateDisplay();
  for(int i=0; i<16; i++) {
    mcp3.digitalWrite(i, HIGH);
    snprintf(aledStr, sizeof(aledStr), "+C%02d", i);
    oled.drawStr(48, 30, aledStr);
    if(i>0){
      mcp3.digitalWrite(i-1, LOW);
      snprintf(aledStr, sizeof(aledStr), "-C%02d", i-1);
      oled.drawStr(88, 30, aledStr);
    }
    oled.updateDisplay();
    delay(millisec);
  }
  mcp3.digitalWrite(15, LOW);
  oled.drawStr(0, 30, "Port C done   ");
  oled.sendBuffer();
  oled.drawStr(0, 47, "MCP23017-D:");
  oled.drawStr(0, 63, "Port:");
  oled.updateDisplay();
  for(int i=0; i<16; i++) {
    mcp4.digitalWrite(i, HIGH);
    snprintf(aledStr, sizeof(aledStr), "+D%02d", i);
    oled.drawStr(48, 63, aledStr);
    if(i>0){
      mcp4.digitalWrite(i-1, LOW);
      snprintf(aledStr, sizeof(aledStr), "-D%02d", i-1);
      oled.drawStr(88, 63, aledStr);
    }
    oled.sendBuffer();
    delay(millisec);
  }
  mcp4.digitalWrite(15, LOW);
  oled.drawStr(0, 63, "Port D done   ");
  oled.updateDisplay();
}

void display_daysymbol(U8G2_SSD1306_128X64_NONAME_F_HW_I2C_2 oled) {
  oled.setFont(u8g2_font_t0_17_mr);
  if(daylight) oled.drawXBMP(96,32, 32, 32, dayImg);
  else if (rtc.hour() == 23) oled.drawXBMP(96,32, 32, 32, ghostImg);
  else oled.drawXBMP(96,32, 32, 32, nightImg);
  oled.sendBuffer();
}

/* ------------------------------------------------- */
/* motor1turn demonstrates 1x360 degree stepper turn */
/* clockwise, and then returns back counterclockwise */
/* Red LED will follow the indicator aligned with D1 */
/* ------------------------------------------------- */
void motor1turn(uint8_t count) {
  int mled;               /* To move one red LED clockwise  */
  int oldmled;            /* To turn off the previous LED   */
  mcp1.writeGPIOAB(0x00);  /* Turn off all red LED 1..16     */
  mcp2.writeGPIOAB(0x00);  /* Turn off all red LED 17..32    */
  digitalWrite(ena1,LOW); /* Enable stepstick1 motor driver */
  for(uint8_t turn = 0; turn < count; turn++) {
    digitalWrite(dir1,HIGH); /* clockwise motor direction   */
    for(int x = 0; x < 1600; x++) { /* 1600 steps = 1 turn  */
      digitalWrite(step1,HIGH); 
      delayMicroseconds(500); 
      digitalWrite(step1,LOW); 
      delayMicroseconds(500);
      if (x % 50 == 0) { /* convert step value to 1..32 LED */
        mled = x/50;     /* 1600/50=32, update LED only for */ 
        if (mled != oldmled) { /* steps with no remainder   */
          if(oldmled < 16) mcp1.digitalWrite(oldmled, LOW);
          else if(oldmled < 32) mcp2.digitalWrite((oldmled-16), LOW);
          if(mled < 16) mcp1.digitalWrite(mled, HIGH);
          if(mled > 15) mcp2.digitalWrite((mled-16), HIGH);
          oldmled = mled;
        }
      }
    }
    mcp1.digitalWrite(0, HIGH); /* Last value is 32, turns 1 */
    mcp2.digitalWrite(15, LOW); /* and delete the old LED 31 */
    
    oldmled = 0;                /* Reset the old LED value   */
    delay(1000);                /* Wait for one second delay */
    digitalWrite(dir1,LOW);     /* Set dir counterclockwise  */
    for(int x = 1599; x >= 0; x--) { /* 1599..0 steps 1 turn */
      digitalWrite(step1,HIGH);
      delayMicroseconds(500);
      digitalWrite(step1,LOW);
      delayMicroseconds(500);
      if (x % 50 == 0) {
        mled = x/50;
        if (mled != oldmled) {
          if(oldmled < 16) mcp1.digitalWrite(oldmled, LOW);
          else if(oldmled < 32) mcp2.digitalWrite((oldmled-16), LOW);
          if(mled > 15) mcp2.digitalWrite((mled-16), HIGH);
          else if(mled < 16) mcp1.digitalWrite(mled, HIGH);
          oldmled = mled;
        }
      }
    }
    mcp1.digitalWrite(0, HIGH); /* Turn the last LED 2 to 1  */
    mcp1.digitalWrite(1, LOW);  /* and delete the old LED 2  */
    delay(1000);                /* Wait for one second delay */
  }
  digitalWrite(ena1,HIGH); /* Disable stepstick1 motordriver */
  mcp1.writeGPIOAB(0x00);  /* Turn off all red LED 1..16     */
  mcp2.writeGPIOAB(0x00);  /* Turn off all red LED 17..32    */
}

/* ------------------------------------------------- */
/* motor1adjust2 moves from the current position     */
/* to the target position. position range 0..16000   */
/* ------------------------------------------------- */
void motor1adjust(uint16_t target, int speed) {
  uint16_t steps = 0;
  m1move = true;
  digitalWrite(ena1,LOW); /* Enable stepstick1 motor driver */
  if(m1cpos < m1tpos) {
    if(abs(m1cpos - m1tpos) < 800) {
      digitalWrite(dir1,HIGH); /* run clockwise motor direction */
      steps = m1tpos - m1cpos; /* get required steps to target  */
    }
    else {                    /* the other way around is shorter */
      digitalWrite(dir1,LOW); /* move counterclockwise direction */
      steps = 1600 - m1tpos + m1cpos;
    }
  }
  else {
    if((m1cpos - m1tpos) < 800) {
      digitalWrite(dir1,LOW); /* counterclockwise  direction  */
      steps = m1cpos - m1tpos;
    }
    else {                     /* the other way around is shorter */
      digitalWrite(dir1,HIGH); /* run clockwise motor direction   */
      steps = 1600 - m1cpos + m1tpos;      
    }
  }
  for(int x = 0; x < steps; x++) {
      digitalWrite(step1,HIGH);
      delayMicroseconds(speed);
      digitalWrite(step1,LOW); 
      delayMicroseconds(speed);
  }
  digitalWrite(ena1,HIGH); /* Disable stepstick1 motordriver */
  m1move = false;
  m1cpos = m1tpos;
}

/* ------------------------------------------------- */
/* motor1home moves from the current motor position  */
/* back to starting point aligned with LED D1.       */
/* ------------------------------------------------- */
void motor1home(int speed) {
  uint16_t steps = 0;
  m1move = true;
  digitalWrite(ena1,LOW); /* Enable stepstick1 motor driver */
  if(m1cpos < 800) {
    digitalWrite(dir1,LOW); /* counterclockwise  direction  */
    steps = m1cpos;
  }
  else {
    digitalWrite(dir1,HIGH); /* clockwise motor direction   */
    steps = 1600 - m1cpos;
  }
  for(int x = 0; x < steps; x++) {
      digitalWrite(step1,HIGH);
      delayMicroseconds(speed);
      digitalWrite(step1,LOW); 
      delayMicroseconds(speed);
  }
  digitalWrite(ena1,HIGH); /* Disable stepstick1 motordriver */
  m1move = false;
  m1cpos = 0;
}

/* ------------------------------------------------- */
/* motor1led moves from the current motor position   */
/* one single LED back. The speed defines step time, */
/* 500 = fast, 5000 = slow                           */
/* ------------------------------------------------- */
void motor1led(int speed) {
  uint16_t steps = 50;
  m1move = true;
  digitalWrite(ena1,LOW); /* Enable stepstick1 motordriver */
  digitalWrite(dir1,HIGH); /* clockwise motor direction    */
  for(int x = 0; x < steps; x++) {
      digitalWrite(step1,HIGH);
      delayMicroseconds(speed);
      digitalWrite(step1,LOW); 
      delayMicroseconds(speed);
  }
  digitalWrite(ena1,HIGH); /* Disable stepstick1 motordriver */
  m1move = false;
  m1cpos = m1cpos + 50;
}

#ifdef DEBUG
/* ------------------------------------------------- */
/* Debug LED and motor values to serial              */
/* ------------------------------------------------- */
void var2serial() {
  Serial.print("Heading LED: ");
  Serial.print(hled);
  Serial.print(" old: ");
  Serial.print(oldhled);
  Serial.print(" Azimuth LED: ");
  Serial.print(aled);
  Serial.print(" old: ");
  Serial.print(oldaled);
  Serial.print(" M1 target: ");
  Serial.print(m1tpos);
  Serial.print(" M1 current: ");
  Serial.print(m1cpos);
  Serial.print(" Push-1: ");
  Serial.print(digitalRead(push1));
  Serial.print(" Push-2: ");
  Serial.println(digitalRead(push2));
}
/* ------------------------------------------------- */
/* Debug DIP switch and button values to serial      */
/* ------------------------------------------------- */
void swi2serial() {
  Serial.print("DIP-1: ");
  Serial.print(dippos1);
  Serial.print(" DIP-2: ");
  Serial.print(dippos2);
  Serial.print(" Push-1: ");
  Serial.print(digitalRead(push1));
  Serial.print(" Push-2: ");
  Serial.println(digitalRead(push2));
}
#endif

/* ------------------------------------------------- */
/* Read daily sun angles, store them in day arrays   */
/* ------------------------------------------------- */
void fill_dayarray(char *file) {
  uint8_t linebuf[19];
  uint16_t i = 0;
  File bin = SD.open(file);
  while (bin.available()) {
    /* safety check. dayfile should have max 1440 data points */
    if(i >= 1440) break;
    bin.read(linebuf, sizeof(linebuf));
    memcpy(&azimuth, &linebuf[3], sizeof(double));
    memcpy(&zenith, &linebuf[11], sizeof(double));
    // convert float to uint16_t (2 bytes) 0..360
    day_azi[i] = (uint16_t) round(azimuth);
    // convert float to int16_t (2 bytes) -90..90
    day_ele[i] = 90 - (int16_t) round(zenith);
    i++;
  }
  bin.close();
}
