/*
 * File:   operation.c
 * Author: tiffa
 *
 * Created on February 18, 2019, 1:34 PM
 */


#include <xc.h>
#include <stdbool.h>
#include <math.h>
#include "configBits.h"
#include "lcd.h"
// for RTC
#include <stdio.h>
#include "I2C.h"

#include "operation.h"
#include "timer0.h"

// Global Variable Declarations
double total_time[] = {0,0,0}; // array storing the total time data

/* I2C to the arduino needs to go in here */
void startOperation () {
    // reset total_time
    total_time[0] = 0;
    total_time[1] = 0;
    total_time[2] = 0;
    initTimer0();
    unsigned int secs, mins, hrs;
    unsigned int stop = 0; // 1 when arduino has stopped all parts, for estop
    
    // print "Executing operation" message
    lcd_clear();
    lcd_home();
    printf("Executing");
    lcd_set_ddram_addr(LCD_LINE2_ADDR);
    printf("operation");
    
    
    while(1){
        if (keyD_was_pressed) {
            stopTimer0();
            keyD_was_pressed = false; // clear the flag if they press start
            break;
        }
        lcd_set_ddram_addr(LCD_LINE3_ADDR);
        printf("%.0fm%.0fs",round(total_time[1]),round(total_time[0]));
        __delay_ms(1000);

    }
    
    // reformat the time into minutes and seconds
    stopTimer0();
    secs = (unsigned int)round(total_time[0]); // round num of secs to nearest whole #
    mins = (unsigned int)floor(secs/60); // 60 seconds per min
    hrs = (unsigned int)floor(mins/60); // 60 mins per hr
    secs = secs % 60; // remove seconds that carried over to mins or hrs
    mins = mins % 60; // remove mins that carried over to hrs
    // assign values back to array
    total_time[0] = (double)secs;
    total_time[1] = (double)mins;
    total_time[2] = (double)hrs;
    
    printComplete();
    
    return;
}

void printComplete(void){
    lcd_clear();
    lcd_home();
    printf("Operation");
    lcd_set_ddram_addr(LCD_LINE2_ADDR);
    printf("complete!");
    __delay_ms(5000);
    return;
}
