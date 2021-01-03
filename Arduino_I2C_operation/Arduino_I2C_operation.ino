/************************ Arduino_I2C_operation.ino ************************/
/*** This file contains the code that is run by the Arduino during the operation
 *   of the cone deployment machine. Peripherals controlled by the Arduino include:
 *   3 servos (2 of which are controlled by the same Arduino output), 
 *   2 DC motors each with PWM signals for the wheels, 
 *   1 optical wheel encoder, 
 *   3 hole / crack IR sensors, 
 *   2 IR sensors responsible for guiding the machine along the lane, 
 *   2 LEDs (one to signal hole detection, one for crack detection).
 *   
 *   Author: Tiffany Yau
 *   Date Created: February 11, 2019
 */


/*** Include libraries used in the program ***/
#include <Wire.h>         // Required for I2C communication with the PIC
#include <Servo.h>        // Needed for manipulation of servo motors
#include "PinChangeInt.h" // Used to allow all pins to be interrupts

/*** Label each pin descriptively to help with making future debugging easier ***/
#define armServo 2
#define DCLeftPWM 3
#define lateralServo 4
#define DCLefta 5
#define DCLeftb 6
#define IRLeft 7
#define IRRight 8
#define DCRighta 9
#define DCRightb 10
#define DCRightPWM 11
#define LEDhole 12
#define LEDcrack 13
#define encoderRight 14 // (A0) use digitalRead(pin) to read digital value
#define IRa 15 // (A1)
#define IRb 16 // (A2)
#define IRc 17 // (A3)

/*** Function prototypes for all functions and ISRs in order of definition ***/
void setup(void);
void loop(void);
void start_operation(void);
void moveForward(void);
void holeDetected(void);
void crackDetected(int num);
void deployHole(void);
void deployCrack(int num);
void dropOne(void);
void returnToStart(void);
void receiveEvent(void);
void requestEvent(void);
void encoderRightPulseISR (void);
void HoleCrackIDISR (void);
void LRAdjustISR (void);

/*** Definition of global variables ***/
/*    including: 1) flags set by the ISRs,
 *               2) constant values used in the code
 *               3) servo objects
 *               4) variables to be updated during program execution
 */
/** (1) Flags set by the ISRs (checked by the main loop or other functions) **/
volatile bool send_to_pic = false;    
volatile bool operation = false;    // true when PIC communication signals operation start
volatile uint8_t i2cByte = '0';
volatile bool holeID = false;       // true when hole is detected
volatile bool crackID1 = false;     // true when crack detected in position 1
volatile bool crackID2 = false;     // true when crack detected in position 2
volatile bool crackID3 = false;     // true when crack detected in position 3
volatile bool endoflane = false;    // true when when distance travelled has reached 420
volatile bool endofturn = false;    // true when robot is done turning (upon return to start)
volatile bool backatstart = false;  // true when robot arrives back behind start line
volatile bool IRRightOff = false;   // true when right IR lane sensor detects black
volatile bool IRLeftOff = false;    // true when left IR lane sensor detects black
/** (2) Constant values used in the code **/
const float robot_length = 10.0;   // distance from front of robot to hole/crack IR sensors // CHANGE THIS LATER
const int ctsServoStop = 90;        // speed level at which lateral cts servo is unmoving // CHANGE THESE LATER, AFTER EXPERIMENTING WITH OUR SERVO
const int ctsServoRight = 0;      // cts servo moves dropping mechanism to the right (max speed)
const int ctsServoLeft = 180;         // cts servo moves dropping mechanism to the left (max speed)
/** (3) Servo objects **/
Servo LRServo;                      // servo object for lateral movement continuous servo
Servo ArmsServo;                    // servo object for deployment arms position servo
/** (4) Variables to be updated during program execution **/
volatile float distance = 0;                // Tracks the current distance from start line (CHANGE LATER TO ALIGN WITH HL/CR SENSORS)
float crack_dists[] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};  // preset to be 20 0's for crack distances
float hole_dists[] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};   // preset to be 20 0's for hole distances
int crack_dists_idx = 0;            // to track the array index we're at for the cracks
int hole_dists_idx = 0;             // to track the array index we're at for the holes
int num_cones = 0;                  // to track number of cones deployed so far
int num_cracks = 0;                 // to track number of cracks detected so far
int num_holes = 0;                  // to track number of holes detected so far
float prev_dist = 999;             // Distance at which the previous crack or hole was detected (999-skipped))
int prev_CrHl = 2;                  // Previous crack or hole was: 0-hole, 1-crack, 2-skipped
float turn_dist = 0;               // keep track of turning distance (when returning to start)
volatile int variables_idx = 0;     // for sending variables array back to PIC in sync
volatile bool back = false;         // to signal if in the middle of sending info back
int variables[44];        // create a byte array to hold all needed vars
    /* Within this array:
     * index: 0 - distance
     *        1-20 - crack_dists[]
     *        21-40 - hole_dists[]
     *        41 - num_cones
     *        42 - num_cracks
     *        43 - num_holes
     */
char distance_byte[5]; // size must be bigger than size of the float version of the number
char crack_dists_byte[85];
char hole_dists_byte[85];
char num_cones_byte[5];
char num_cracks_byte[5];
char num_holes_byte[5];
int send_idx = 0;
uint8_t variables_byte[300];
    /* Within this array:
     *  index: 0-2 - distance         // 3-digit number convert to 3 bytes
     *         3-82 - crack_dists[]   // 20 3-digit numbers convert to 3 bytes each + comma
     *         83-162 - hole_dists[]  
     *         163-164 - num_cones    // two digit number
     *         165-166 - num_cracks   // two digit number
     *         167-168 - num_holes    // two digit number
     */

/*** void setup(void);
 *   This sets up the pins and links interrupt pins with their respective ISRs 
 */
void setup(){
    Wire.begin(8); // Join I2C bus with address 8
  
    // Register callback functions (interrupt handlers for receiving and sending info to/from PIC)
    Wire.onReceive(receiveEvent); // Called when this slave device receives a data transmission from master
    Wire.onRequest(requestEvent); // Called when master requests data from this slave device
  
    // Open serial port to PC (hardware UART) (for debugging)
    Serial.begin(9600);

    // Associate servo objects with their respective pins
    LRServo.attach(lateralServo); 
    ArmsServo.attach(armServo); 
    // Pin assignments and initialization to input or output
    pinMode(DCLefta,OUTPUT);
    pinMode(DCLeftb,OUTPUT);
    pinMode(DCLeftPWM,OUTPUT);
    pinMode(DCRighta,OUTPUT);
    pinMode(DCRightb,OUTPUT);
    pinMode(DCRightPWM,OUTPUT);
    pinMode(LEDhole,OUTPUT);
    pinMode(LEDcrack,OUTPUT); 
    pinMode(IRLeft,INPUT);
    pinMode(IRRight,INPUT);
    // Configure the analog pins that will be used as digital pins
    pinMode(IRa,INPUT_PULLUP); 
    pinMode(IRb,INPUT_PULLUP);
    pinMode(IRc,INPUT_PULLUP);
    pinMode(encoderRight,INPUT_PULLUP);

    // Associate the interrupt pins with their respective ISRs using PinChangeInt library
    // interrupts for lane detection sensors
    attachPinChangeInterrupt(IRLeft, LRAdjustISR, CHANGE); 
    attachPinChangeInterrupt(IRRight, LRAdjustISR, CHANGE);
    // interrupts triggered only on change from white to black
    attachPinChangeInterrupt(IRa, HoleCrackIDISR, RISING); 
    attachPinChangeInterrupt(IRb, HoleCrackIDISR, RISING);
    attachPinChangeInterrupt(IRc, HoleCrackIDISR, RISING);
    attachPinChangeInterrupt(encoderRight, encoderRightPulseISR, RISING);    
}

/*** void loop(void);
 *   This main loop waits for a signal from the PIC to start the operation.
 */
void loop(){
    // If we should send to the PIC, then we wait to receive a byte from the PC
    // at start of program, we are receiving from pic
//    if (send_to_pic && Serial.available() > 0 && !incomingByte) {
//        incomingByte = Serial.read();
//        digitalWrite(13, HIGH);
//        delay(1000);
//        digitalWrite(13, LOW);
//        delay(1000);
//    }
    ArmsServo.write(0); // make sure arm servos are in the right spot
    if (operation){
        operation = false;
        start_operation();

//        digitalWrite(13, HIGH);
//        delay(1000);
//        digitalWrite(13, LOW);
//        delay(1000);
    }
}

/*** void start_operation(void);
 *   This function 1) resets the global variable values
 *                 2) calls moveForward();
 *                 3) calls returnToStart() when machine has reached the end of the lane
 *                 4) updates variables[44] array to send information back to PIC
 */
void start_operation (void) {
    // Reset values of the global variables
    i2cByte = '0';
    distance = 0;                // Tracks the current distance from start line (CHANGE LATER TO ALIGN WITH HL/CR SENSORS)
    int i = 0;
    for (i=0;i<20;i++){
        crack_dists[i]=0;
        hole_dists[i]=0;
    }
    crack_dists_idx = 0;            // to track the array index we're at for the cracks
    hole_dists_idx = 0;             // to track the array index we're at for the holes
    num_cones = 0;                  // to track number of cones deployed so far
    num_cracks = 0;                 // to track number of cracks detected so far
    num_holes = 0;                  // to track number of holes detected so far
    prev_dist = 999;             // Distance at which the previous crack or hole was detected (999-skipped))
    prev_CrHl = 2;                  // Previous crack or hole was: 0-hole, 1-crack, 2-skipped
    turn_dist = 0;               // keep track of turning distance (when returning to start)
    variables_idx = 0;            // for i2c communication later on
    back = false;                 // for i2c communication later on, true when information is being sent back
    // Reset global byte variables (char arrays)
    for (i=0; i<5; i++) {
        distance_byte[i] = '0';
        num_cones_byte[i] = '0';
        num_cracks_byte[i] = '0';
        num_holes_byte[i] = '0';
    }
    for (i=0; i<85; i++){
        crack_dists_byte[i]='0';
        hole_dists_byte[i]='0';
    }
    
    // Make the machine move forward, following the lane lines
    // moveForward() responds appropriately when holes and cracks are detected
    moveForward(); 

    // Return the machine back to the start line after completing the run
    returnToStart();

    // Write all integers and floats to their corresponding byte arrays
    // Distance travelled
    sprintf(distance_byte, "%d", round(distance));
    // Crack / hole distances
    int idx = 1;
    unsigned char temp[5]; //temporary array to be copied to byte array
    while (crack_dists[idx] != 0) { // prefilled with zeros, if next isn't zero
        sprintf(crack_dists_byte,"%d,",round(crack_dists[idx-1])); // put in previous
        memcpy(&crack_dists_byte[4*(idx-1)],temp,4); // copy over to correct byte array spot
        idx = idx + 1;       
    }
    sprintf(temp, "%d", round(crack_dists[idx-1])); // put in the last one, no comma
    memcpy(&crack_dists_byte[4*(idx-1)],temp,4);    // copy over 4 bytes - 3 for 3 digits, 1 for comma
    idx = 1;
    while (hole_dists[idx] != 0) { // prefilled with zeros, if next isn't zero
        sprintf(hole_dists_byte,"%d,",round(hole_dists[idx-1])); // put in previous
        memcpy(&hole_dists_byte[4*(idx-1)],temp,4); // copy over to correct byte array spot
        idx = idx + 1;       
    }
    sprintf(temp, "%d", round(hole_dists[idx-1])); // put in the last one, no comma
    memcpy(&hole_dists_byte[4*(idx-1)],temp,4);
    // Number of cones
    sprintf(num_cones_byte, "%d", num_cones);
    // Number of cracks
    sprintf(num_cracks_byte, "%d", num_cones);
    // Number of holes
    sprintf(num_holes_byte, "%d", num_cones);
    
    // Prepare variables to be written back to the PIC
//    variables[0] = distance;
//    for (i = 1; i <= 20; i++) {
//        variables[i] = crack_dists[i-1];
//    }
//    for (i = 21; i <= 40; i++) {
//        variables[i] = hole_dists[i-21];
//    }
//    variables[41] = num_cones;
//    variables[42] = num_cracks;
//    variables[43] = num_holes;
    i2cByte = '9';            // signal ready for variables to be transferred back to PIC
}

/*** void moveForward(void);
 *   This function 1) ensures the machine moves forward along the lane, making adjustments when necessary
 *                 2) checks the holeID, crackID1, crackID2, and crackID3 flags for hole or crack detection
 *                 3) updates the hole_dists and crack_dists lists accordingly
 *                 4) calls holeDetected() and crackDetected() accordingly
 */
void moveForward(void) {
    int n = 0;
    while (1) {
        n = n + 1;
        // set left motor moving forward
        digitalWrite(DCLefta, LOW);
        digitalWrite(DCLeftb, HIGH);
        analogWrite(DCLeftPWM, 110);
        // set right motor moving forward
        digitalWrite(DCRighta, LOW);
        digitalWrite(DCRightb, HIGH);
        analogWrite(DCRightPWM,110);
        if ((n%10000) == 0){
            Serial.print("IRa reads: ");
            Serial.println(digitalRead(IRa));
            Serial.print("IRb reads: ");
            Serial.println(digitalRead(IRb));
            Serial.print("IRc reads: ");
            Serial.println(digitalRead(IRc));
            Serial.print("IRLeft reads: ");
            Serial.println(digitalRead(IRLeft));
            Serial.print("IRRight reads: ");
            Serial.println(digitalRead(IRRight));
        }
        
        
        // set PWM signals and check the left and right readjustment tags
        // flags are automatically reset to false in the ISR when readjustment is complete
        if (IRLeftOff) {                  // if the left IR sensor is on the line
            analogWrite(DCLeftPWM,0);     // stop the left motor
            analogWrite(DCRightPWM,110);  // continue with the right motor
        }
        if (IRRightOff) {
            analogWrite(DCRightPWM,0);    // stop the right motor
            analogWrite(DCLeftPWM,110);   // continue with the left motor
        }
        if (((!IRRightOff) && (!IRLeftOff)) || ((IRRightOff) && (IRLeftOff))) {
            analogWrite(DCLeftPWM, 110);  // (range of PWM --> 0 to 255)
            analogWrite(DCRightPWM, 110);
        }
        
        // Check hole and crack detection flags
        if (holeID) {
            holeID = false;               // reset flag
            hole_dists[hole_dists_idx] = distance - robot_length + 2; // add hole distance to list (adjust for center of hole distance)
            // DELAY HERE --> TEST THE DELAY NEEDED 
            analogWrite(DCLeftPWM, 0);
            analogWrite(DCRightPWM, 0);
            holeDetected();
        }
        else if (crackID1){
            crackID1 = false;             // reset flag
            crack_dists[crack_dists_idx] = distance - robot_length + 1; // add crack distance to list
            // DELAY HERE --> TEST THE DELAY NEEDED
            analogWrite(DCLeftPWM, 0);
            analogWrite(DCRightPWM, 0);          
            crackDetected(1);
        }
        else if (crackID2){
            crackID2 = false;             // reset flag
            crack_dists[crack_dists_idx] = distance - robot_length + 1;
            // DELAY HERE --> TEST THE DELAY NEEDED        
            analogWrite(DCLeftPWM, 0);
            analogWrite(DCRightPWM, 0);          
            crackDetected(2);
        }
        else if (crackID3){
            crackID3 = false;             // reset flag
            crack_dists[crack_dists_idx] = distance - robot_length + 1;
            // DELAY HERE --> TEST THE DELAY NEEDED          
            analogWrite(DCLeftPWM, 0);
            analogWrite(DCRightPWM, 0);          
            crackDetected(3);
        }
        
        // Check if robot has reached the end of the lane
        if (endoflane == true) {
            break;                        // break out of loop / moveForward()
        }
    }
}

/*** void holeDetected(void);
 *   This function, called by moveForward() when flag holeID = true, does the following:
 *       1) signals the hole detected LED to turn on for one second
 *       2) updates the number of holes detected so far
 *       3) determines whether or not a cone needs to be deployed
 *       4) updates the prev_CrHl, prev_dist, and hole_dists_idx accordingly
 *       5) calls deployHole() as necessary
 */
void holeDetected(void){
    // Signal for hole detection
    digitalWrite(LEDhole, HIGH);
    delay(1000);
    digitalWrite(LEDhole,LOW);
    
    // Increase count of number of holes detected so far
    num_holes = num_holes + 1;

    // Determine whether or not a cone needs to be deployed
    if (prev_dist == 999) {
        deployHole();
        prev_dist = hole_dists[hole_dists_idx];
        prev_CrHl = 0;
    }
    else if ((15<(distance-prev_dist)) && ((distance-prev_dist)<20) && (prev_CrHl == 1)) {
        deployHole();                               // deploy at this distance only if previous was crack
        prev_dist = hole_dists[hole_dists_idx];
        prev_CrHl = 0;       
    }
    else if ((distance-prev_dist) >= 20) {
        deployHole();                               // deploy for any distance larger than 20
        prev_dist = hole_dists[hole_dists_idx];      
        prev_CrHl = 0;
    }
    else {                                          // otherwise, update global variables and skip hole
        prev_dist = 999;                            // recall: 999 for skipped
        prev_CrHl = 2;                              // recall: 2 for skipped
    }

    // Move hole distance list index forward     
    hole_dists_idx = hole_dists_idx + 1;
}

/*** void crackDetected(int num);
 *   This function, called by moveForward() when flag crackID1, crackID2, or crackID3 = true, does the following:
 *       1) signals the crack detected LED to turn on for one second
 *       2) updates the number of cracks detected so far
 *       3) determines whether or not a cone needs to be deployed
 *       4) updates the prev_CrHl, prev_dist, and hole_dists_idx accordingly
 *       5) calls deploycrack(num) as necessary
 */
void crackDetected(int num) {
    // Signal for crack detection
    digitalWrite(LEDcrack, HIGH);
    delay(1000);
    digitalWrite(LEDcrack,LOW);
    
    // Increase count of number of cracks detected so far
    num_cracks = num_cracks + 1;

    // Determine whether or not cones need to be deployed
    if (prev_dist == 999) {
        deployCrack(num);
        prev_dist = crack_dists[crack_dists_idx];
        prev_CrHl = 1;
    }
    else if ((10<(distance-prev_dist)) && ((distance-prev_dist)<20) && (prev_CrHl == 0)) {
        deployCrack(num);                           // deploy at this distance only if previous was crack
        prev_dist = crack_dists[crack_dists_idx];
        prev_CrHl = 1;
    }
    else if ((distance-prev_dist) >= 20) {
        deployCrack(num);                           // deploy for any distance greater than 20
        prev_dist = crack_dists[crack_dists_idx];
        prev_CrHl = 1;
    }
    else {                                          // otherwise, update global variables and skip crack
        prev_dist = 999;
        prev_CrHl = 2; 
    }

    // Move crack distance list index forward
    crack_dists_idx = crack_dists_idx + 1;  
}

/*** void deployHole(void);
 *   This function drops a cone over the centre of the lane where the holes are.
 */
void deployHole(void) {
    // dropping mechanism should already be in the middle
    // just drop one over the hole in the middle
    dropOne();
}

/*** void deployCrack(int num);
 *   This function 
 *        1) drops two cones over a crack, with positioning based on the input variable, num.
 *        2) moves the lateral movement mechanism as needed to achieve correct positions
 */
void deployCrack(int num) {
    // dropping mechanism should be starting off and ending at the middle 
    // need to move dropping mechanism to the correct position depending on input crack number
    if (num == 1) {
        // move to the left spot
        LRServo.write(ctsServoLeft); 
        delay(1361);                  // time to rotate 245 degrees to leftmost spot (from testing)
        LRServo.write(ctsServoStop);  // stop the servo
        dropOne();
        
        // move to the right spot
        LRServo.write(ctsServoRight);
        delay(1694);                  // time to rotate 305 degrees to right spot of configuration 1 
        LRServo.write(ctsServoStop);  // stop the servo
        dropOne();
        
        // move back to middle
        LRServo.write(ctsServoLeft);
        delay(1694-1361);             // rotate back to the middle
        LRServo.write(ctsServoStop);  // stop the servo
    }
    else if (num == 2) {
        // move to the left spot
        LRServo.write(ctsServoLeft);  // move slightly to the left
        delay(850);                   // rotate 153 degrees to leftmost spot
        LRServo.write(ctsServoStop);  // stop the servo
        dropOne();

        // move to the right spot
        LRServo.write(ctsServoRight); // move to right side
        delay(1694);                  // rotate 305 degrees to right spot of configuration 2
        LRServo.write(ctsServoStop);  // stop the servo
        dropOne();

        // move back to middle
        LRServo.write(ctsServoLeft); 
        delay(1694-850);              // rotate back to the middle
        LRServo.write(ctsServoStop);
    }
    else if (num == 3) {
        // move to the rightmost spot
        LRServo.write(ctsServoRight); 
        delay(1361);                  // rotate 245 degrees to rightmost spot
        LRServo.write(ctsServoStop);
        dropOne();

        // move to the left spot
        LRServo.write(ctsServoLeft);
        delay(1694);                  // rotate 305 degrees to left spot of configuration 3
        LRServo.write(ctsServoStop);
        dropOne();

        // move back to the middle
        LRServo.write(ctsServoRight);
        delay(1694-1361);             // rotate back to the middle
        LRServo.write(ctsServoStop);
    }
}

/*** void dropOne(void);
 *   This function controls the dropping mechanism arms to release one cone only.
 *   Requirement for physical setup during integration: 
 *        ensure servo is calibrated to position at angle 0.
 */
void dropOne(void) {
    num_cones = num_cones + 1;  // update number of cones deployed
    int pos = 0;
    // manipulate arms to drop a cone
    for (pos = 0; pos <= 60; pos += 1) { // goes from 0 degrees to 60 degrees
        // in steps of 1 degree
        ArmsServo.write(pos);              // tell servo to go to position in variable 'pos'
        delay(15);                       // waits 15ms for the servo to reach the position
    }
    for (pos = 60; pos >= 0; pos -= 1) { // goes from 60 degrees to 0 degrees
        ArmsServo.write(pos);              // tell servo to go to position in variable 'pos'
        delay(15);                       // waits 15ms for the servo to reach the position
    }
}

/*** void returnToStart(void);
 *   This function: 1) turns the robot around (left)
 *                  2) drives the robot back to the start line
 *                  3) stops after the robot is completely behind the start line
 */
void returnToStart(void) {
    // To turn the machine around to move back to the start line
    while (!endofturn) {              // when the robot isn't done turning yet, keep turning left
        analogWrite(DCLeftPWM,0);     // left motor unmoving
        analogWrite(DCRightPWM,110);  // right motor moves (in semicircle with left wheel as pivot)
    }
    while (!backatstart) {            // move forward until it reaches the start
        // set left motor moving forward
        digitalWrite(DCLefta, LOW);
        digitalWrite(DCLeftb, HIGH);
        analogWrite(DCLeftPWM,110);
        // set right motor moving forward
        digitalWrite(DCRighta, LOW);
        digitalWrite(DCRightb, HIGH);
        analogWrite(DCRightPWM,110);
    }
    analogWrite(DCLeftPWM,0);
    analogWrite(DCRightPWM,0);
}


/********** Interrupt Service Routines **********/
/***** For read and write requests from the PIC,
 *     IR sensor hole and crack detection,
 *     IR sensor left / right adjustment detection,
 *     and wheel encoder distance increment / updates. 
 *     Set flags which other functions check.
 *****/

/* Callback for when the master transmits data */
void receiveEvent(void){ // for emergency stop
    uint8_t x = Wire.read(); // receive byte
    Serial.println((char)x); // print to serial output as char (ASCII representation)... for debugging
    if (x=='1') {
        operation = true;
    }
}

/* Callback for when the master requests data */
void requestEvent(void){ 
    if (i2cByte == '0') {
        Wire.write(i2cByte); // Respond with message of 1 byte
    }
//    else if (i2cByte == '9') { // send variables
//        if (variables_idx == 0) {
//            Wire.write(i2cByte);
//        }
//        Wire.write(variables[variables_idx]);
//        variables_idx = variables_idx + 1;
//    }
    else if ((i2cByte == '9') && (!back)){ // send variables
        // if havent sent the signal to pic to request info yet
        Wire.write(i2cByte);
        back = true;
        i2cByte = '2'; // signal start of sending distance
        send_idx = 0;
    }
    else if (i2cByte = '2'){
        Wire.write(distance_byte[send_idx]);
        send_idx = send_idx + 1;
        if (send_idx == 5) {
            i2cByte = '3'; // move on to next thing to send
            send_idx = 0; // reset the index to 0 for next byte array
        }
    }
    else if (i2cByte = '3'){
        Wire.write(crack_dists_byte[send_idx]);
        send_idx = send_idx + 1;
        if (send_idx == 85) {
            i2cByte = '4';
            send_idx = 0;
        }
    }
    else if (i2cByte = '4'){
        Wire.write(hole_dists_byte[send_idx]);
        send_idx = send_idx + 1;
        if (send_idx == 85) {
            i2cByte = '5';
            send_idx = 0;
        }
    }
    else if (i2cByte = '5'){
        Wire.write(num_cones_byte[send_idx]);
        send_idx = send_idx + 1;
        if (send_idx == 5) {
            i2cByte = '6'; // move on to next thing to send
            send_idx = 0; // reset the index to 0 for next byte array
        }        
    }
    else if (i2cByte = '6'){
        Wire.write(num_cracks_byte[send_idx]);
        send_idx = send_idx + 1;
        if (send_idx == 5) {
            i2cByte = '7'; // move on to next thing to send
            send_idx = 0; // reset the index to 0 for next byte array
        }        
    }
    else if (i2cByte = '7'){
        Wire.write(num_holes_byte[send_idx]);
        send_idx = send_idx + 1;
        if (send_idx == 5) {
            i2cByte = '0'; // end of sending
            send_idx = 0; // reset the index to 0 for next byte array
        }
    }
}

/* Interrupt routine for encoder pulses --> to track distance travelled */
void encoderRightPulseISR (void) {
    if (digitalRead(encoderRight)) { // float check that encoder is indeed at 1
        distance = distance + 1.0210176124; // from calculation
        if (distance > 420) { // extra distance to be safe
            endoflane = true; // main loop makes this turn when it's true
        }
        if (distance > 420 + 91.1061869544) {
            endofturn = true;
        }
        if (distance > 420 + 91.1061869544 + 440) { // extra factor to ensure it's fully behind start
            backatstart = true;
        }
    }
}

/* Interrupt routine of hole / crack IR sensors 
 *  and differentiates between a hole versus the various crack positions
 */
void HoleCrackIDISR (void) { 
    // (motors will be stopped by moveForward() when the flag is checked)
    if (!digitalRead(IRa) && digitalRead(IRb) && !digitalRead(IRc)) { // only sensor B detects
        holeID = true;
    }
    else if (digitalRead(IRa) && digitalRead(IRb) && !digitalRead(IRc)) {
        crackID1 = true;
    }
    else if (digitalRead(IRa) && digitalRead(IRb) && digitalRead(IRc)) {
        crackID2 = true;
    }
    else if (!digitalRead(IRa) && digitalRead(IRb) && digitalRead(IRc)) {
        crackID3 = true;
    }
}

/* Interrupt routine to adjust direction of robot travel based on left / right IR sensors */
/* IR Sensors output 1 for black and 0 for white */
void LRAdjustISR (void) {
    if (!digitalRead(IRLeft) && digitalRead(IRRight)) {       // if right IR is on black
        IRRightOff = true; //set flag checked by moveForward()
        IRLeftOff = false;
    }
    else if (digitalRead(IRLeft) && !digitalRead(IRRight)) {  // if left detects black
        IRLeftOff = true;
        IRRightOff = false;
    }
    else if ((!digitalRead(IRLeft) && !digitalRead(IRRight)) || (digitalRead(IRLeft) && digitalRead(IRRight))){
        IRLeftOff = false;      // if both detect black, assume it's because of the lane width variance
        IRRightOff = false;     // and set both flags as false
    }
}
