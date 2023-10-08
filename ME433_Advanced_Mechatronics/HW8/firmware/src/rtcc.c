#include "rtcc.h"

char DAYOFTHEWEEK[7][11] = { "Sunday.", "Monday.", "Tuesday.", "Wednesday.", "Thursday.", "Friday.", "Saturday."};

void dayOfTheWeek(unsigned char position, char* day){
    // given the number of the day of the week, return the word in a char array
    int i = 0;
    while(DAYOFTHEWEEK[position][i]!='.'){
        day[i] = DAYOFTHEWEEK[position][i];
        i++;
    }
    day[i] = '\0';
}

void rtcc_setup(unsigned long time, unsigned long date) {
    OSCCONbits.SOSCEN = 1; // turn on secondary clock
    while (!OSCCONbits.SOSCRDY) {
    } // wait for the clock to stabilize, touch the crystal if you get stuck here

    // unlock sequence to change the RTCC settings, then the CPU allows changes to be made 
    SYSKEY = 0x0; // force lock, try without first
    SYSKEY = 0xaa996655; // write first unlock key to SYSKEY
    SYSKEY = 0x556699aa; // write second unlock key to SYSKEY
    // RTCWREN is protected, unlock the processor to change it
    RTCCONbits.RTCWREN = 1; // RTCC bits are not locked, can be changed by the user

    RTCCONbits.ON = 0; // turn off the clock
    while (RTCCONbits.RTCCLKON) {
    } // wait for clock to be turned off

    RTCTIME = time; // safe to update the time
    RTCDATE = date; // update the date

    RTCCONbits.ON = 1; // turn on the RTCC

    while (!RTCCONbits.RTCCLKON); // wait for clock to start running, , touch the crystal if you get stuck here
    LATBbits.LATB5 = 0;
}


rtccTime readRTCC() {
    rtccTime time;
    time.hr10 = RTCTIMEbits.HR10; // copy over the BCD 
    time.hr01 = RTCTIMEbits.HR01; // copy over the BCD 
    time.min10 = RTCTIMEbits.MIN10; // copy over the BCD 
    time.min01 = RTCTIMEbits.MIN01; // copy over the BCD 
    time.sec10 = RTCTIMEbits.SEC10; // copy over the BCD 
    time.sec01 = RTCTIMEbits.SEC01;  // copy over the BCD 
                 
    time.yr10 = RTCDATEbits.YEAR10;
    time.yr01 = RTCDATEbits.YEAR01;
    time.mn10 = RTCDATEbits.MONTH10;
    time.mn01 = RTCDATEbits.MONTH01;
    time.dy01 = RTCDATEbits.DAY01;
    time.dy10 = RTCDATEbits.DAY10;
    time.wk = RTCDATEbits.WDAY01; 

    return time;
}