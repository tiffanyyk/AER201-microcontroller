/*
 * File:   main.c
 * Author: Tiffany Yau
 *
 * Created on January 19, 2019, 10:29 PM
 */

// for LCD
#include <xc.h>
#include <stdbool.h>
#include <math.h>
#include "configBits.h"
#include "lcd.h"
// for RTC
#include <stdio.h>
#include "I2C.h"
// for timing the operation

// function headers
void startOperation(void);
void initTimer0(void);
void printComplete(void);
void stopTimer0(void);

// Global volatile variables for keypad
const char keys[] = "123A456B789C*0#D";
volatile bool key1_was_pressed = false;
volatile bool key2_was_pressed = false;
volatile bool key3_was_pressed = false;
volatile bool key4_was_pressed = false;
volatile bool keyB_was_pressed = false; // always used as a "back" button
volatile bool keyD_was_pressed = false; // always used as "start" button
// operation global variables
//double distance = 0; // to keep track of current distance
//double crack_dists[] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};  // preset to be 20 0's
//double hole_dists[] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};   // preset to be 20 0's
//int num_cones = 0;                  // to track number of cones deployed so far
//int num_cracks = 0;                 // to track number of cracks detected so far
//int num_holes = 0;                  // to track number of holes detected so far
//double prev_dist = 999;             // to keep track of the distance of previous hole or crack, 999 for N/A or skipped
//int prev_CrHl = 2;                  // 2 for skipped or N/A, 1 for crack, 0 for hole
// global volatile for emergency stop
//volatile bool emergency_pressed = false;

// Global Variable Declarations
double total_time[] = {0,0,0}; // array storing the total time data
unsigned char variables[44]; // create a byte array to hold all needed vars
    /* Within this array:
     * index: 0 - distance
     *        1-20 - crack_dists[]
     *        21-40 - hole_dists[]
     *        41 - num_cones
     *        42 - num_cracks
     *        43 - num_holes
     */
char distance_byte[5];
char crack_dists_byte[85];
char hole_dists_byte[85];
char num_cones_byte[5];
char num_cracks_byte[5];
char num_holes_byte[5];

void main(void) {
//    INTCONbits.GIE = 1; // enable interrupts for all 
//    INTCON3bits.INT1IE = 1; // enable the specific interrupt that we want (INT1))
    
    // RB1, RB4, RB5, RB6, RB7 as inputs (for keypad)
    // RB2 as input (for emergency stop)
    LATB = 0x00;
    TRISB = 0b11110110;
    //INTCON3bits.INT2IE = 1; // enable specific interrupt that we want (estop) (INT2)
    
    // RD2 is the character LCD RS
    // RD3 is the character LCD enable (E)
    // RD4-RD7 are character LCD data lines
    LATD = 0x00; // set everything to 0 at first
    TRISD = 0x00; // set everything to output   
    
    // Initialize LCD
    initLCD();
    lcd_display_control(true, false, false); // turn off cursor and cursor flash
    
    // Clock setup
    // Initialize I2C Master with 100 kHz clock
    I2C_Master_Init(100000);
    unsigned char time[7]; // Create a byte array to hold time read from RTC
    // Set the time in the RTC. To see the RTC keep time, comment this line out
    // after programming the PIC directly before with this line included
    //rtc_set_time();

    
    // Keyboard setup
    // Set all A/D ports to digital (pg. 222)
    ADCON1 = 0b00001111;
    // Enable RB1 (keypad data available) interrupt
    INT1IE = 1;
    // Enable interrupts
    ei();
    
    // Main loop
    while(1){
                
        // loop to display time and menu options 
        __delay_ms(1000);
        // Reset RTC memory pointer
        I2C_Master_Start(); // Start condition
        I2C_Master_Write(0b11010000); // 7 bit RTC address + Write
        I2C_Master_Write(0x00); // Set memory pointer to seconds
        I2C_Master_Stop(); // Stop condition

        // Read current time
        I2C_Master_Start(); // Start condition
        I2C_Master_Write(0b11010001); // 7 bit RTC address + Read
        for(unsigned char i = 0; i < 6; i++){
            time[i] = I2C_Master_Read(ACK); // Read with ACK to continue reading
        }
        time[6] = I2C_Master_Read(NACK); // Final Read with NACK
        I2C_Master_Stop(); // Stop condition
        
        // Print received time data on LCD
        lcd_clear();
        lcd_home();
        printf("%02x/%02x/%02x", time[6],time[5],time[4]); // Print date in YY/MM/DD
        lcd_set_ddram_addr(LCD_LINE2_ADDR);
        printf("%02x:%02x:%02x", time[2],time[1],time[0]); // HH:MM:SS

        // print menu selections
        lcd_set_ddram_addr(LCD_LINE3_ADDR);
        printf("1-OpTime,2-#Depl");
        lcd_set_ddram_addr(LCD_LINE4_ADDR);
        printf("3-Hole&CrackData");  
        
        // while loop to wait for user's key press
        while (1) {
            // if they pick the first option (operation time))
            if (key1_was_pressed){
                key1_was_pressed = false; // Clear the flag
                // operation time display
                lcd_clear();
                lcd_home();
                printf("OPERATION TIME");
                lcd_set_ddram_addr(LCD_LINE2_ADDR);
                printf("%.0fm%.0fs",round(total_time[1]),round(total_time[0])); // from total time array, 0 before any runs
                // while loop to wait for user to press back button, "B"
                while (1) {
                    if (keyB_was_pressed) {
                        keyB_was_pressed = false; // Clear the flag
                        break; // exit waiting for back loop
                    }
                    // if user presses start within this sub screen
                    if (keyD_was_pressed) {
                        keyD_was_pressed = false; // clear the flag and do nothing
                    }
                }
            }
            // if they pick the second option (number of cones deployed)
            if (key2_was_pressed) {
                key2_was_pressed = false; // clear the flag
                // number of cones deployed display
                lcd_clear();
                lcd_home();
                printf("#CONES DEPLOYED");
                lcd_set_ddram_addr(LCD_LINE2_ADDR);
                printf("%s", num_cones_byte); // using variables array
                // while loop to wait for user to press back button, "B"
                while (1) {
                    if (keyB_was_pressed) {
                        keyB_was_pressed = false; // Clear the flag
                        break; // exit waiting for back loop
                    }
                    // if user presses start within this sub screen
                    if (keyD_was_pressed) {
                        keyD_was_pressed = false; // clear the flag and do nothing
                    }
                }
            }
            // if they pick the second option (hole and crack data)
            if (key3_was_pressed) {
                key3_was_pressed = false; // clear the flag
                // hole and crack distance menu
                lcd_clear();
                lcd_home();
                printf("Hole&CrackData");
                lcd_set_ddram_addr(LCD_LINE2_ADDR);
                printf("1-Holes");
                lcd_set_ddram_addr(LCD_LINE3_ADDR);
                printf("2-Cracks");
                // while loop to wait for user to choose either hole (1) or crack (2)
                while (1) {
                    // if they want to display the holes data
                    if (key1_was_pressed){ 
                        key1_was_pressed = false; // clear the flag
                        // number of holes and their distances
                        lcd_clear();
                        lcd_home();
                        printf("#Holes: %d", 3);
                        lcd_set_ddram_addr(LCD_LINE2_ADDR);
                        char arr_hole[] = "4,15,35";
                        printf("Distances (cm):");
                        lcd_set_ddram_addr(LCD_LINE3_ADDR);
                        printf("%s", arr_hole);
                        // while loop to wait for user to press button and go back 
                        // to hole (1) and crack (2) sub menu
                        while (1) {
                            if (keyB_was_pressed){ // go back if B was pressed
                                keyB_was_pressed = false; // clear the flag
                                // go back to display the hole and crack sub menu
                                lcd_clear();
                                lcd_home();
                                printf("Hole&CrackData");
                                lcd_set_ddram_addr(LCD_LINE2_ADDR);
                                printf("1-Holes");
                                lcd_set_ddram_addr(LCD_LINE3_ADDR);
                                printf("2-Cracks");
                                break; // exit waiting for back loop
                            }
                            // if user presses start in a sub screen
                            if (keyD_was_pressed) {
                                keyD_was_pressed = false; // clear the flag and do nothing
                            }
                        }
                    }
                    // if they want to display the cracks data
                    if (key2_was_pressed) { 
                        key2_was_pressed = false; // clear the flag
                        // number of cracks and their distances
                        lcd_clear();
                        lcd_home();
                        printf("#Cracks: %d", 5);
                        lcd_set_ddram_addr(LCD_LINE2_ADDR);
                        char arr_crack[] = "10,21,45,67,150";
                        printf("Distances (cm):");
                        lcd_set_ddram_addr(LCD_LINE3_ADDR);
                        printf("%s",arr_crack);
                        // while loop to wait for user to press button and go back 
                        // to hole (1) and crack (2) sub menu
                        while(1) {
                            if (keyB_was_pressed){ // go back if B was pressed
                                keyB_was_pressed = false;
                                // go back to display the hole and crack sub menu
                                lcd_clear();
                                lcd_home();
                                printf("Hole&CrackData");
                                lcd_set_ddram_addr(LCD_LINE2_ADDR);
                                printf("1-Holes");
                                lcd_set_ddram_addr(LCD_LINE3_ADDR);
                                printf("2-Cracks");
                                break; // exit waiting for back loop
                            }
                            // if user presses start in a sub screen
                            if (keyD_was_pressed) {
                                keyD_was_pressed = false; // clear the flag and do nothing
                            }
                        }
                    }
                    // if they want to go back to the main menu
                    if (keyB_was_pressed) {
                        keyB_was_pressed = false; // clear the flag
                        break; // exit waiting for back loop
                    }
                    // if user presses start inside a sub menu / screen
                    if (keyD_was_pressed) {
                        keyD_was_pressed = false; // clear the flag and do nothing
                    }
                }
            }
            // if they press the start button, start operation
            if (keyD_was_pressed) {
                keyD_was_pressed = false;
                // call the startOperation function
                startOperation();
            }
            if (keyB_was_pressed) {
                keyB_was_pressed = false; // clear the flag and do nothing
            }
            break; // exit loop that waits for user key press 
            // go back to display main menu
        }
    }
}

/* I2C to the arduino needs to go in here */
void startOperation () {
    // reset total_time
    total_time[0] = 0;
    total_time[1] = 0;
    total_time[2] = 0;
    initTimer0();
    unsigned int secs, mins, hrs;
    // for getting info from arduino
    unsigned char data;
    // reset global variables for operation
//    distance = 0; // to keep track of current distance
//    for (int i = 0; i <20; i++){
//        crack_dists[i] = 0;
//        hole_dists[i] = 0;
//    }
//    num_cones = 0;                  // to track number of cones deployed so far
//    num_cracks = 0;                 // to track number of cracks detected so far
//    num_holes = 0;                  // to track number of holes detected so far
//    prev_dist = 999;             // to keep track of the distance of previous hole or crack, 999 for N/A or skipped
//    prev_CrHl = 2;                  // 2 for skipped or N/A, 1 for crack, 0 for hole
    
    // initialize i2c with arduino address
    I2C_Master_Init(100000);
    I2C_Master_Start();
    I2C_Master_Write(0b00010000); // address of arduino
    I2C_Master_Stop();
    
    // print "Executing operation" message
    lcd_clear();
    lcd_home();
    printf("Executing");
    lcd_set_ddram_addr(LCD_LINE2_ADDR);
    printf("operation");
    
    // send data to the Arduino --> '1' signals it to start operation
    I2C_Master_Start();
    I2C_Master_Write(0b00010000); // address of arduino
    I2C_Master_Write((unsigned char)'1'); // send data to pic
    I2C_Master_Stop();
    
    // while loop that waits for data to be received back from the Arduino
    while(1){
        // week 5 testing stuff
//        if (keyD_was_pressed) {
//            stopTimer0();
//            keyD_was_pressed = false; // clear the flag if they press start
//            break;
//        }
        /* I2C with the arduino to execute operation */

        // asks for data once every two seconds
        __delay_ms(2000);
        
        // look at RTC reading stuff for how to read multiple bytes
        I2C_Master_Start();
        I2C_Master_Write(0b00010001); // 7-bit Arduino slave address + Read
        data = I2C_Master_Read(NACK); // Read one char only (data is unsigned char))
        I2C_Master_Stop();
        
        if (data == '9') { // 9 is the signal from arduino to show operation is done
            //printf("%c",data);
            I2C_Master_Start();
            I2C_Master_Write(0b00010001); // 7-bit Arduino slave address + Read
            
            // for interface, just print these unsigned chars one by one

            for (int varcnt = 0; varcnt < 5; varcnt++){
                distance_byte[varcnt] = I2C_Master_Read(ACK);
            }
            for (int varcnt = 0; varcnt < 85; varcnt++){
                crack_dists_byte[varcnt] = I2C_Master_Read(ACK);
            }
            for (int varcnt = 0; varcnt < 85; varcnt++){
                hole_dists_byte[varcnt] = I2C_Master_Read(ACK);
            }
            for (int varcnt = 0; varcnt < 5; varcnt++){
                num_cones_byte[varcnt] = I2C_Master_Read(ACK);
            }
            for (int varcnt = 0; varcnt < 5; varcnt++){
                num_cracks_byte[varcnt] = I2C_Master_Read(ACK);
            }
            for (int varcnt = 0; varcnt < 4; varcnt++){
                num_holes_byte[varcnt] = I2C_Master_Read(ACK);
            }
            num_holes_byte[4] = I2C_Master_Read(NACK); // final read
//            for (int varcnt = 0; varcnt < 43; varcnt++) {
//                variables[varcnt] = I2C_Master_Read(ACK); // read with ACK to continue reading
//                //printf("%x",variables[varcnt]);
//            }
//            variables[43] = I2C_Master_Read(NACK); // final read with NACK
            I2C_Master_Stop(); // stop condition
            break; // exit loop --> operation complete
        }
        

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

/**
 * @brief Any time an interrupt is generated, the microcontroller will execute
 *        this function (as long as interrupts are enabled). Any interrupts
 *        which are enabled need to be handled in this function, otherwise
 *        unexpected behavior will arise, perhaps even causing the PIC to reset
 *        (you AT LEAST need to clear the flag for each interrupt which is
 *        enabled!)
 */

void __interrupt() interruptHandler(void){
    
    // Interrupt on change handler for RB1 / keypad
    if(INT1IF){
        // Notice how we keep the interrupt processing very short by simply
        // setting a "flag" which the main program loop checks
        
        // rb1 takes input from the smaller pic which tells it which key was pressed
        // check to make sure the user let go of the button before making it continue
        while (PORTBbits.RB1 == 0) { continue; }
        unsigned char keypress = (PORTB & 0xF0) >> 4;
        while (PORTBbits.RB1 == 1) { continue; }
        //printf("%s",keypress);
        if (keys[keypress] == '1') {
            key1_was_pressed = true;
        }
        else if (keys[keypress] == '2') {
            key2_was_pressed = true;
        }
        else if (keys[keypress] == '3') {
            key3_was_pressed = true;
        }
        else if (keys[keypress] == '4') {
            key4_was_pressed = true;
        }
        else if (keys[keypress] == 'B') {
            keyB_was_pressed = true;
        }
        else if (keys[keypress] == 'D') {
            keyD_was_pressed = true;
        }
        INT1IF = 0; // Clear interrupt flag bit to signify it's been handled
    }
    
    // if timer overflow occurs
    // other than the textbook, this was a helpful source (although not the same pic):
    // https://www.exploreembedded.com/wiki/PIC18F4520_Timer 
    if (INTCONbits.TMR0IF) { // when the timer overflows
        total_time[0] = total_time[0] + 0.838848; // increase the time count
        INTCONbits.TMR0IF = 0; // Clear interrupt flag bit to show it's been handled
    }
    
}