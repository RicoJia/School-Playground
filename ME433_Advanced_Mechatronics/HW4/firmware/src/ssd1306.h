#ifndef SSD1306_H__
#define SSD1306_H__

#include "I2C.h"
#include <stdio.h>
#include "font.h"

// Based on the adafruit and sparkfun libraries
#define SSD1306_MEMORYMODE          0x20 
#define SSD1306_COLUMNADDR          0x21 
#define SSD1306_PAGEADDR            0x22 
#define SSD1306_SETCONTRAST         0x81 
#define SSD1306_CHARGEPUMP          0x8D 
#define SSD1306_SEGREMAP            0xA0 
#define SSD1306_DISPLAYALLON_RESUME 0xA4 
#define SSD1306_NORMALDISPLAY       0xA6 
#define SSD1306_INVERTDISPLAY       0xA7 
#define SSD1306_SETMULTIPLEX        0xA8 
#define SSD1306_DISPLAYOFF          0xAE 
#define SSD1306_DISPLAYON           0xAF 
#define SSD1306_COMSCANDEC          0xC8 
#define SSD1306_SETDISPLAYOFFSET    0xD3 
#define SSD1306_SETDISPLAYCLOCKDIV  0xD5 
#define SSD1306_SETPRECHARGE        0xD9 
#define SSD1306_SETCOMPINS          0xDA 
#define SSD1306_SETVCOMDETECT       0xDB 
#define SSD1306_SETSTARTLINE        0x40 
#define SSD1306_DEACTIVATE_SCROLL   0x2E ///< Stop scroll

void ssd1306_setup(void);       // use after i2c setup. 
void ssd1306_update(void);
void ssd1306_clear(void);       //use with update
void ssd1306_drawPixel(unsigned char x, unsigned char y, unsigned char color);      //use with update

/// this should be private
void ssd1306_command(unsigned char c);


// --------------------------upper level functions

struct position{
    short int x, y; 
    void (*reset_pos)(struct position*, double x, double y);    // reset function that using function pointer... c struct doesn't have member functions. sigh. 
    void (*move_to_next_pos)(struct position*);   // update the char position on the screen. 
};

//char get_ASCII_display_byte(char c);    //get one column of the char. 
void reset_pos(struct position*, double x, double y);
void move_to_next_pos(struct position*);   // update the char position on the screen. 
void draw_ASCII_bytes(short int x, short int y, const char ASCII_byte); 

// the mother function of all previous three functions. 
void write_char(char c); 

// mother function of write_char
void drawMessage(double x, double y, char* msg);
#endif