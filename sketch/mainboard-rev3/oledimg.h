/* ---------------------------------------------------- *
 * Suntracker2  mainboard-rev3 oledimg.h 2019-07 @FM4DD *
 *                                                      *
 * This file has the bitmaps for the two OLED displays. *
 * ---------------------------------------------------- */

/* ---------------------------------------------------- */
/* 'arduino logo', 64x32px, XBM format                  */
/* ---------------------------------------------------- */
static const unsigned char arduLogo[] U8X8_PROGMEM = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0xFC, 0x0F, 0x00, 0x00, 0xF0, 0x3F, 0x00, 
  0x80, 0xFF, 0x3F, 0x00, 0x00, 0xFE, 0xFF, 0x00, 0xC0, 0xFF, 0xFF, 0x00, 
  0x80, 0xFF, 0xFF, 0x03, 0xE0, 0xFF, 0xFF, 0x01, 0xC0, 0xFF, 0xFF, 0x07, 
  0xF0, 0x07, 0xF8, 0x07, 0xE0, 0x1F, 0xE0, 0x0F, 0xF8, 0x01, 0xE0, 0x0F, 
  0xF0, 0x07, 0x80, 0x0F, 0xFC, 0x00, 0xC0, 0x1F, 0xF8, 0x01, 0x00, 0x1F, 
  0x7C, 0x00, 0x00, 0x3F, 0xFC, 0x00, 0x00, 0x3E, 0x3E, 0x00, 0x00, 0x7E, 
  0x7E, 0xC0, 0x03, 0x3E, 0x3E, 0x00, 0x00, 0x7C, 0x3F, 0xC0, 0x03, 0x3C, 
  0x1E, 0x00, 0x00, 0xF8, 0x1F, 0xC0, 0x03, 0x7C, 0x1E, 0x00, 0x00, 0xF0, 
  0x0F, 0xC0, 0x03, 0x78, 0x1E, 0xFE, 0x1F, 0xF0, 0x07, 0xFC, 0x3F, 0x78, 
  0x1E, 0xFE, 0x1F, 0xE0, 0x03, 0xFC, 0x3F, 0x78, 0x1E, 0xFE, 0x1F, 0xE0, 
  0x07, 0xFC, 0x3F, 0x78, 0x1E, 0xFE, 0x1F, 0xF0, 0x0F, 0xFC, 0x3F, 0x78, 
  0x1E, 0x00, 0x00, 0xF8, 0x0F, 0xC0, 0x03, 0x78, 0x1E, 0x00, 0x00, 0xFC, 
  0x1F, 0xC0, 0x03, 0x3C, 0x3E, 0x00, 0x00, 0x7E, 0x3E, 0xC0, 0x03, 0x3C, 
  0x3C, 0x00, 0x00, 0x3F, 0x7E, 0xC0, 0x03, 0x3E, 0x7C, 0x00, 0x80, 0x1F, 
  0xFC, 0x01, 0x00, 0x1F, 0xF8, 0x00, 0xC0, 0x0F, 0xF8, 0x03, 0x80, 0x1F, 
  0xF8, 0x03, 0xF0, 0x07, 0xF0, 0x0F, 0xC0, 0x0F, 0xF0, 0x0F, 0xFC, 0x03, 
  0xE0, 0x3F, 0xF0, 0x07, 0xE0, 0xFF, 0xFF, 0x01, 0x80, 0xFF, 0xFF, 0x03, 
  0x80, 0xFF, 0x7F, 0x00, 0x00, 0xFF, 0xFF, 0x01, 0x00, 0xFF, 0x1F, 0x00, 
  0x00, 0xFC, 0x7F, 0x00, 0x00, 0xF8, 0x03, 0x00, 0x00, 0xE0, 0x0F, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, };

/* ---------------------------------------------------- */
/* 'day logo', 32x32px, XBM format                      */
/* ---------------------------------------------------- */
static const unsigned char dayImg[] U8X8_PROGMEM = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0xC0, 0x03, 0x00, 0x00, 0xE0, 0x07, 0x00, 
  0x00, 0xE0, 0x07, 0x00, 0xE0, 0xC0, 0x03, 0x07, 0xF0, 0x81, 0x81, 0x0F, 
  0xF0, 0x01, 0x80, 0x0F, 0xF0, 0x01, 0x80, 0x0F, 0xE0, 0xE0, 0x07, 0x07, 
  0x00, 0xF8, 0x1F, 0x00, 0x00, 0xFC, 0x3F, 0x00, 0x00, 0xFE, 0x7F, 0x00, 
  0x00, 0x3E, 0x7C, 0x00, 0x0C, 0x1F, 0xF8, 0x30, 0x1E, 0x0F, 0xF0, 0x78, 
  0x3E, 0x0F, 0xF0, 0x7C, 0x3E, 0x0F, 0xF0, 0x7C, 0x1E, 0x0F, 0xF0, 0x78, 
  0x0C, 0x1F, 0xF8, 0x30, 0x00, 0x7E, 0x7E, 0x00, 0x00, 0xFE, 0x7F, 0x00, 
  0x00, 0xFC, 0x3F, 0x00, 0x00, 0xF8, 0x1F, 0x00, 0xE0, 0xE0, 0x07, 0x07, 
  0xF0, 0x01, 0x80, 0x0F, 0xF0, 0x01, 0x80, 0x0F, 0xF0, 0x81, 0x81, 0x0F, 
  0xE0, 0xC0, 0x03, 0x07, 0x00, 0xE0, 0x07, 0x00, 0x00, 0xE0, 0x07, 0x00, 
  0x00, 0xC0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, };

/* ---------------------------------------------------- */
/* 'night logo', 32x32px, XBM format                    */
/* ---------------------------------------------------- */
static const unsigned char nightImg[] U8X8_PROGMEM = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 
  0x80, 0x03, 0x00, 0x00, 0xC0, 0x03, 0x00, 0x00, 0xE0, 0x03, 0x00, 0x00, 
  0xE0, 0x07, 0x00, 0x00, 0xF0, 0x07, 0x00, 0x00, 0xF0, 0x07, 0x00, 0x00, 
  0xF0, 0x0F, 0x00, 0x00, 0xF0, 0x1F, 0x00, 0x00, 0xF8, 0x1F, 0x00, 0x00, 
  0xF0, 0x3F, 0x00, 0x00, 0xF0, 0xFF, 0x00, 0x00, 0xF0, 0xFF, 0x01, 0x00, 
  0xF0, 0xFF, 0x0F, 0x0C, 0xF0, 0xFF, 0xFF, 0x0F, 0xE0, 0xFF, 0xFF, 0x0F, 
  0xE0, 0xFF, 0xFF, 0x07, 0xC0, 0xFF, 0xFF, 0x07, 0x80, 0xFF, 0xFF, 0x03, 
  0x00, 0xFF, 0xFF, 0x01, 0x00, 0xFE, 0x7F, 0x00, 0x00, 0xF8, 0x3F, 0x00, 
  0x00, 0xA0, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, };

/* ---------------------------------------------------- */
/* 'ghost logo', 32x32px, XBM format                    */
/* ---------------------------------------------------- */
static const unsigned char ghostImg[] U8X8_PROGMEM = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xE0, 0x07, 0x00, 
  0x00, 0xFC, 0x1F, 0x00, 0x00, 0xFE, 0x7F, 0x00, 0x00, 0xFF, 0xFF, 0x00, 
  0x80, 0xFF, 0xFF, 0x00, 0x80, 0xFF, 0xFF, 0x01, 0xC0, 0xFF, 0xFF, 0x01, 
  0xC0, 0xFF, 0xFF, 0x01, 0xC0, 0xC3, 0xC3, 0x03, 0xE0, 0xC3, 0xC3, 0x03, 
  0xE0, 0xC3, 0xC3, 0x03, 0xE0, 0xFF, 0xFF, 0x03, 0xE0, 0xFF, 0xFF, 0x03, 
  0xE0, 0x3F, 0xFE, 0x03, 0xE0, 0x0F, 0xF8, 0x03, 0xE0, 0x07, 0xF0, 0x03, 
  0xE0, 0x07, 0xF0, 0x03, 0xE0, 0xFF, 0xFF, 0x03, 0xE0, 0xFF, 0xFF, 0x03, 
  0xE0, 0xFF, 0xFF, 0x03, 0xE0, 0xFF, 0xFF, 0x03, 0xE0, 0xFF, 0xFF, 0x03, 
  0xE0, 0xFF, 0xFF, 0x03, 0xE0, 0xFD, 0xFE, 0x03, 0xE0, 0xFD, 0xBE, 0x03, 
  0xC0, 0x79, 0xBE, 0x03, 0xC0, 0x78, 0x1C, 0x03, 0x80, 0x30, 0x08, 0x02, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, };

/* ---------------------------------------------------- */
/* 'sunrise logo', 16x16px, XBM format                  */
/* ---------------------------------------------------- */
static const unsigned char riseImg[] U8X8_PROGMEM = {
  0x00, 0x00, 0x80, 0x00, 0x82, 0x40, 0x84, 0x20, 0x08, 0x10, 0xC0, 0x03, 
  0xF0, 0x0F, 0xFA, 0x5F, 0xF8, 0x1F, 0xFC, 0x3F, 0xFC, 0x3F, 0xFC, 0x3F, 
  0x00, 0x00, 0xFC, 0x3F, 0x00, 0x00, 0xE0, 0x07, };

/* ---------------------------------------------------- */
/* 'sunset logo', 16x16px, XBM format                   */
/* ---------------------------------------------------- */
static const unsigned char setImg[] U8X8_PROGMEM = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x0F, 0xC0, 0x03, 0xE0, 0x01, 0xF0, 0x00, 
  0x78, 0x00, 0x78, 0x00, 0x78, 0x00, 0xF0, 0x01, 0xE0, 0x0F, 0xFC, 0x3F, 
  0x00, 0x00, 0xFC, 0x3F, 0x00, 0x00, 0xE0, 0x07, };
