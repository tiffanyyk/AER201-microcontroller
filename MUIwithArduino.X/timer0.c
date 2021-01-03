/*
 * File:   timer0.c
 * Author: tiffa
 *
 * Created on February 18, 2019, 1:35 PM
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

void initTimer0(void) {
    TMR0 = 0x0000; // clear the TMR0 register
    T0CONbits.TMR0ON = 1; // enable / turn on timer0
    T0CONbits.T08BIT = 0; // configure as 16-bit timer
    T0CONbits.PSA = 0; // prescaler assigned
    // assign a prescaler value of 32... (100))
    // so TMR0L increases once every 32 instruction cycles 
    // TMR0L then increases once every 32 * 400ns = 12800 nanoseconds
    // when TMR0L has reached 0xffff = 65535, 
    // 65535*12800 ns = 838,848,000 ns would have passed
    // this would translate to 0.838848 seconds between each interrupt
    // add this number to the total number of seconds each time interrupt occurs
    
    T0CONbits.T0PS0 = 0; // prescaler values
    T0CONbits.T0PS1 = 0; // prescaler values
    T0CONbits.T0PS2 = 1; // prescaler values
    
    T0CONbits.T0CS = 0; // internal instruction cycle clock selected (timer mode on)
    // excitation cycle of clock at 10MHz (set in configBits.h)
    // clock currently goes once once every 100 nanoseconds 
    // 400 nanoseconds per instruction
    
    INTCONbits.TMR0IE = 1; // enable timer0 interrupts
    INTCONbits.TMR0IF = 0; // initialize flag to be 0
    return;
}

void stopTimer0(void){
    T0CONbits.TMR0ON = 0; // stop timer
    return;
}