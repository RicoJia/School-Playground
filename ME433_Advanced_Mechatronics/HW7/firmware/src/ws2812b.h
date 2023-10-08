#ifndef WS2812B_H__
#define WS2812B_H__

#include<xc.h> // processor SFR definitions

// link three 8bit colors together
typedef struct {
    unsigned char r;
    unsigned char g;
    unsigned char b;
} wsColor; 

void ws2812b_setup();
void ws2812b_setColor(wsColor*,int);
wsColor HSBtoRGB(float hue, float sat, float brightness);


// --------------------------------------------High Level Functions -------------------------------------------
typedef struct {
    float h;
    float s;
    float b;
} wsHSB; 

// increase h value by 1. while s and b remains the same
void increment_hue(wsHSB* h);   //TODO

#endif