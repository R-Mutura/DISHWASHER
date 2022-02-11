#ifndef SCREEN_SSD1305_H
#define SCREEN_SSD1305_H

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1305.h>

// Used for software SPI
//#define OLED_CLK 13
//#define OLED_MOSI 11

// Used for software or hardware SPI
//#define OLED_CS 10
//#define OLED_DC 8

// Used for I2C or SPI
#define OLED_RESET 9

// software SPI
//Adafruit_SSD1305 display(128, 64, OLED_MOSI, OLED_CLK, OLED_DC, OLED_RESET, OLED_CS);
// hardware SPI - use 7Mhz (7000000UL) or lower because the screen is rated for 4MHz, or it will remain blank!
//Adafruit_SSD1305 display(128, 64, &SPI, OLED_DC, OLED_RESET, OLED_CS, 7000000UL);

// I2C
Adafruit_SSD1305 display(128, 64, &Wire, OLED_RESET);

//prototyes for funtions in the cpp folder
void testdrawchar(void);
void testdrawcircle(void);
void testfillrect(void);
void testdrawtriangle(void);
void testfilltriangle(void);
void testdrawroundrect(void);
void testfillroundrect(void);
void testdrawrect(void);
void testdrawline() ;



#endif
