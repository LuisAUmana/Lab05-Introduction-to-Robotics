/*
  This code makes the RSLK robot drive straight, rotate 90 degrees, and drive in a circle of X cm radius
  Luis Umana and Alex Crotts, 2022-02-23
*/

// Include the RSLK library to control the motors and read the encoders
#include "SimpleRSLK.h"

// Define the parameters of the robot and the "driving course"
int MotorSpeed = 12;            // Slow motor speed for stability
float WheelDiameter = 6.985;     // In centimeters
float StraightDistance = 75;      // In centimeters
float PulsePerRev = 360;          // Number of encoder pulses the microcontroller reads per 1 wheel rotation
float WheelBase = 13.335;       // In centimeters
float TurnCCWDeg = 90;            // Degree of rotation before starting the circle

// Calculate the number of encoder pulses needed to complete each phase of the driving course
double Straight_pulses = StraightDistance/((WheelDiameter * PI)/(PulsePerRev));      // Number of pulses needed to drive X centimeters straight

double TurnCCW_pulses = (TurnCCWDeg/360)*(WheelBase * PI)/(WheelDiameter * PI) * (PulsePerRev);      // Number of pulses needed to rotate 90 degrees in place

double Circle_Inner_Wheel_Pulses = 2*(StraightDistance - 0.5*WheelBase) * PI/(WheelDiameter * PI) * PulsePerRev;      // Number of pulses needed for the inner wheel to drive in an X cm radius circle
double Circle_Outer_Wheel_Pulses = 2*(StraightDistance + 0.5*WheelBase) * PI/(WheelDiameter * PI) * PulsePerRev;      // Number of pulses needed for the outer wheel to drive in an X cm radius circle
double SpeedRatio = Circle_Outer_Wheel_Pulses/Circle_Inner_Wheel_Pulses;      // Ratio of outer wheel encoder pulses to inner wheel encoder pulses for driving in a circle
double LeftMotorSpeed = MotorSpeed/SpeedRatio;      // Make the right motor run at a proportionally higher speed than the left motor when driving in a circle based on total encoder pulses


void setup() {  
  // Setup the RSLK to perform necessary functions and initialize the encoders
  setupRSLK();
  resetLeftEncoderCnt();      // Initialize the left encoder
  resetRightEncoderCnt();     // Initialize the right encoder
  Serial.begin(9600);         // Begin the serial monitor 
}

void Drive_Straight() {
  // Function for driving straight for X centimeters
  resetLeftEncoderCnt();
  resetRightEncoderCnt();
  enableMotor(BOTH_MOTORS);
  setMotorDirection(BOTH_MOTORS, MOTOR_DIR_FORWARD);      // Set both motors to drive forward
  setMotorSpeed(BOTH_MOTORS, MotorSpeed);                 // Set both motors to the same speed
  int L_Pulse_Count = 0;      // Zero the left encoder pulse count
  int R_Pulse_Count = 0;      // Zero the right encoder pulse count

  while((L_Pulse_Count < Straight_pulses) || (R_Pulse_Count < Straight_pulses)) {
    // Run this loop until the number of pulses read by the microcontroller reaches the calculated pulses above
    L_Pulse_Count = getEncoderLeftCnt();      // Read the left encoder value
    R_Pulse_Count = getEncoderRightCnt();     // Read the right encoder value

    if((L_Pulse_Count + 1 < R_Pulse_Count)){
      // If the left motor is driving slower than the right, speed up the left motor and slow down the right
      setMotorSpeed(LEFT_MOTOR, ++MotorSpeed);      // Speed up the left motor in increments of 1
      setMotorSpeed(RIGHT_MOTOR, --MotorSpeed);     // Slow down the right motor in increments of 1
    }

    if((R_Pulse_Count + 1 < L_Pulse_Count)){
      // If the right motor is driving slower than the left, speed up the right motor and slow down the left
      setMotorSpeed(RIGHT_MOTOR, ++MotorSpeed);     // Speed up the right motor in incremements of 1
      setMotorSpeed(LEFT_MOTOR, --MotorSpeed);      // Slow down the left motor in increments of 1
    }
    
    if(L_Pulse_Count >= Straight_pulses){
      // If the number of pulses reaches the calculated value, turn off the motors and stop running the function
      disableMotor(LEFT_MOTOR);     // Turn off the left motor
      disableMotor(RIGHT_MOTOR);    // Turn off the right motor
      }

      // Print encoder counts to the serial monitor for debugging
      Serial.print("Driving Straight Now");
      Serial.print("\t");
      Serial.print("Left Encoder: ");
      Serial.print(L_Pulse_Count);
      Serial.print("\t");
      Serial.print("Right Encoder: ");
      Serial.println(R_Pulse_Count);
      delay(100);
  }
}

void Rotate_CCW() {
  // Function for rotating 90 degrees CCW in place
  resetLeftEncoderCnt();
  resetRightEncoderCnt();
  enableMotor(BOTH_MOTORS);
  setMotorDirection(LEFT_MOTOR, MOTOR_DIR_BACKWARD);      // Set the left motor to drive backward
  setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_FORWARD);      // Set the right motor to drive forward
  setMotorSpeed(BOTH_MOTORS, MotorSpeed);                 // Set the motor to the same speed
  int L_CCW_Pulse_Count = 0;      // Zero the encoder count
  int R_CCW_Pulse_Count = 0;      // Zero the encoder count

  while((L_CCW_Pulse_Count < TurnCCW_pulses - 10) || (R_CCW_Pulse_Count < TurnCCW_pulses - 10)) {
    // Run this loop until the number of pulses read by the microcontroller reaches the calculated value above
    L_CCW_Pulse_Count = getEncoderLeftCnt();      // Read the left encoder value
    R_CCW_Pulse_Count = getEncoderRightCnt();     // Read the right encoder value

    if(L_CCW_Pulse_Count >= TurnCCW_pulses -10 || R_CCW_Pulse_Count >= TurnCCW_pulses - 10) {
      // If the number of pulses reaches the calculated value, turn off the motors and stop running the function
      disableMotor(LEFT_MOTOR);       // Turn off the left motor
      disableMotor(RIGHT_MOTOR);      // Turn off the right motor
      delay(1000);
    }

    // Print encoder counts to the serial monitor for debugging
    Serial.print("Turning CCW Now");
    Serial.print("\t");
    Serial.print("Left Encoder CCW Turn: ");
    Serial.print(L_CCW_Pulse_Count);
    Serial.print("\t");
    Serial.print("Right Encoder CCW Turn: ");
    Serial.println(R_CCW_Pulse_Count);
    delay(100);
  }
}

void Drive_Circle() {
  // Function for driving in a circle of radius X centimeters
  resetLeftEncoderCnt();
  resetRightEncoderCnt();
  enableMotor(BOTH_MOTORS);
  setMotorDirection(BOTH_MOTORS, MOTOR_DIR_FORWARD);      // Set both motors to drive forward
  setMotorSpeed(LEFT_MOTOR, LeftMotorSpeed);          // Set the left motor to the MotorSpeed value defined above
  setMotorSpeed(RIGHT_MOTOR, MotorSpeed);    // Since the right motor is driving further than the left, set the speed of the right motor to drive at the proportionally higher speed calculated above
  int L_Circle_Pulse_Count = 0;     // Zero the encoder count
  int R_Circle_Pulse_Count = 0;     // Zero the encoder count

  while((R_Circle_Pulse_Count < Circle_Outer_Wheel_Pulses + 25)) {
    // Run this loop until the number of pulses read by the microcontroller reaches the calculated value above
    L_Circle_Pulse_Count = getEncoderLeftCnt();      // Read the left encoder value
    R_Circle_Pulse_Count = getEncoderRightCnt();     // Read the right encoder value

    if((R_Circle_Pulse_Count + 10 < L_Circle_Pulse_Count * SpeedRatio)) {
      // If the right motor is driving significantly slower than the SpeedRatio has specified, then speed up the right motor
      setMotorSpeed(RIGHT_MOTOR, MotorSpeed + 1);
    }

    if((R_Circle_Pulse_Count + 10 > L_Circle_Pulse_Count * SpeedRatio)) {
       // If the right motor is driving significantly faster than the SpeedRatio has specified, then slow down the right motor
      setMotorSpeed(RIGHT_MOTOR, MotorSpeed - 1);
    }

    if((R_Circle_Pulse_Count >= Circle_Outer_Wheel_Pulses + 25)) {
      // If the number of pulses reaches the calculated value, turn off the motors and stop running the function
      disableMotor(LEFT_MOTOR);       // Turn off the left motor
      disableMotor(RIGHT_MOTOR);      // Turn off the right motor
      delay(1000);
    }

    // Print encoder counts to the serial monitor for debugging
    Serial.print("Driving in a Circle Now");
    Serial.print("\t");
    Serial.print("Left Encoder Circle: ");
    Serial.print(L_Circle_Pulse_Count);
    Serial.print("\t");
    Serial.print("Right Encoder Circle: ");
    Serial.println(R_Circle_Pulse_Count);
    delay(100);
  } 
}

void loop() {
  // Make the robot perform the following functions sequentially with a one second delay between each phase 
  Serial.print("Straight Pulses: \t");   
  Serial.println(Straight_pulses);
  Serial.print("TurnCCW Pulses: \t");
  Serial.println(TurnCCW_pulses);
  Serial.print("Circle Inner Wheel Pulses: \t");
  Serial.println(Circle_Inner_Wheel_Pulses);
  Serial.print("Circle Outer Wheel Pulses: \t");
  Serial.println(Circle_Outer_Wheel_Pulses);
  Serial.print("Speed Ratio: \t");
  Serial.println(SpeedRatio);
  Serial.print("Left Motor Speed: \t");
  Serial.println(LeftMotorSpeed);
  
  delay(3000);         // Wait 3 seconds for setting the robot in position
  Drive_Straight();     // Perform the function for driving straight X centimeters
  delay(1000);
  Rotate_CCW();         // Perform the function for rotating 90 degrees CCW
  delay(1000);
  Drive_Circle();       // Perform the function for driving in a circle of radius X cm
  delay(1000);
  Rotate_CCW();         // Perform the function for rotating 90 degrees CCW
  delay(1000);
  Drive_Straight();     // Perform the function for driving straight back to the original start position
}
/*
  This code makes the RSLK robot drive straight, rotate 90 degrees, and drive in a circle of X cm radius
  Luis Umana and Alex Crotts, 2022-02-23
*/

// Include the RSLK library to control the motors and read the encoders
#include "SimpleRSLK.h"

// Define the parameters of the robot and the "driving course"
int MotorSpeed = 12;            // Slow motor speed for stability
float WheelDiameter = 6.985;     // In centimeters
float StraightDistance = 75;      // In centimeters
float PulsePerRev = 360;          // Number of encoder pulses the microcontroller reads per 1 wheel rotation
float WheelBase = 13.335;       // In centimeters
float TurnCCWDeg = 90;            // Degree of rotation before starting the circle

// Calculate the number of encoder pulses needed to complete each phase of the driving course
double Straight_pulses = StraightDistance/((WheelDiameter * PI)/(PulsePerRev));      // Number of pulses needed to drive X centimeters straight

double TurnCCW_pulses = (TurnCCWDeg/360)*(WheelBase * PI)/(WheelDiameter * PI) * (PulsePerRev);      // Number of pulses needed to rotate 90 degrees in place

double Circle_Inner_Wheel_Pulses = 2*(StraightDistance - 0.5*WheelBase) * PI/(WheelDiameter * PI) * PulsePerRev;      // Number of pulses needed for the inner wheel to drive in an X cm radius circle
double Circle_Outer_Wheel_Pulses = 2*(StraightDistance + 0.5*WheelBase) * PI/(WheelDiameter * PI) * PulsePerRev;      // Number of pulses needed for the outer wheel to drive in an X cm radius circle
double SpeedRatio = Circle_Outer_Wheel_Pulses/Circle_Inner_Wheel_Pulses;      // Ratio of outer wheel encoder pulses to inner wheel encoder pulses for driving in a circle
double LeftMotorSpeed = MotorSpeed/SpeedRatio;      // Make the right motor run at a proportionally higher speed than the left motor when driving in a circle based on total encoder pulses


void setup() {  
  // Setup the RSLK to perform necessary functions and initialize the encoders
  setupRSLK();
  resetLeftEncoderCnt();      // Initialize the left encoder
  resetRightEncoderCnt();     // Initialize the right encoder
  Serial.begin(9600);         // Begin the serial monitor 
}

void Drive_Straight() {
  // Function for driving straight for X centimeters
  resetLeftEncoderCnt();
  resetRightEncoderCnt();
  enableMotor(BOTH_MOTORS);
  setMotorDirection(BOTH_MOTORS, MOTOR_DIR_FORWARD);      // Set both motors to drive forward
  setMotorSpeed(BOTH_MOTORS, MotorSpeed);                 // Set both motors to the same speed
  int L_Pulse_Count = 0;      // Zero the left encoder pulse count
  int R_Pulse_Count = 0;      // Zero the right encoder pulse count

  while((L_Pulse_Count < Straight_pulses) || (R_Pulse_Count < Straight_pulses)) {
    // Run this loop until the number of pulses read by the microcontroller reaches the calculated pulses above
    L_Pulse_Count = getEncoderLeftCnt();      // Read the left encoder value
    R_Pulse_Count = getEncoderRightCnt();     // Read the right encoder value

    if((L_Pulse_Count + 1 < R_Pulse_Count)){
      // If the left motor is driving slower than the right, speed up the left motor and slow down the right
      setMotorSpeed(LEFT_MOTOR, ++MotorSpeed);      // Speed up the left motor in increments of 1
      setMotorSpeed(RIGHT_MOTOR, --MotorSpeed);     // Slow down the right motor in increments of 1
    }

    if((R_Pulse_Count + 1 < L_Pulse_Count)){
      // If the right motor is driving slower than the left, speed up the right motor and slow down the left
      setMotorSpeed(RIGHT_MOTOR, ++MotorSpeed);     // Speed up the right motor in incremements of 1
      setMotorSpeed(LEFT_MOTOR, --MotorSpeed);      // Slow down the left motor in increments of 1
    }
    
    if(L_Pulse_Count >= Straight_pulses){
      // If the number of pulses reaches the calculated value, turn off the motors and stop running the function
      disableMotor(LEFT_MOTOR);     // Turn off the left motor
      disableMotor(RIGHT_MOTOR);    // Turn off the right motor
      }

      // Print encoder counts to the serial monitor for debugging
      Serial.print("Driving Straight Now");
      Serial.print("\t");
      Serial.print("Left Encoder: ");
      Serial.print(L_Pulse_Count);
      Serial.print("\t");
      Serial.print("Right Encoder: ");
      Serial.println(R_Pulse_Count);
      delay(100);
  }
}

void Rotate_CCW() {
  // Function for rotating 90 degrees CCW in place
  resetLeftEncoderCnt();
  resetRightEncoderCnt();
  enableMotor(BOTH_MOTORS);
  setMotorDirection(LEFT_MOTOR, MOTOR_DIR_BACKWARD);      // Set the left motor to drive backward
  setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_FORWARD);      // Set the right motor to drive forward
  setMotorSpeed(BOTH_MOTORS, MotorSpeed);                 // Set the motor to the same speed
  int L_CCW_Pulse_Count = 0;      // Zero the encoder count
  int R_CCW_Pulse_Count = 0;      // Zero the encoder count

  while((L_CCW_Pulse_Count < TurnCCW_pulses - 10) || (R_CCW_Pulse_Count < TurnCCW_pulses - 10)) {
    // Run this loop until the number of pulses read by the microcontroller reaches the calculated value above
    L_CCW_Pulse_Count = getEncoderLeftCnt();      // Read the left encoder value
    R_CCW_Pulse_Count = getEncoderRightCnt();     // Read the right encoder value

    if(L_CCW_Pulse_Count >= TurnCCW_pulses -10 || R_CCW_Pulse_Count >= TurnCCW_pulses - 10) {
      // If the number of pulses reaches the calculated value, turn off the motors and stop running the function
      disableMotor(LEFT_MOTOR);       // Turn off the left motor
      disableMotor(RIGHT_MOTOR);      // Turn off the right motor
      delay(1000);
    }

    // Print encoder counts to the serial monitor for debugging
    Serial.print("Turning CCW Now");
    Serial.print("\t");
    Serial.print("Left Encoder CCW Turn: ");
    Serial.print(L_CCW_Pulse_Count);
    Serial.print("\t");
    Serial.print("Right Encoder CCW Turn: ");
    Serial.println(R_CCW_Pulse_Count);
    delay(100);
  }
}

void Drive_Circle() {
  // Function for driving in a circle of radius X centimeters
  resetLeftEncoderCnt();
  resetRightEncoderCnt();
  enableMotor(BOTH_MOTORS);
  setMotorDirection(BOTH_MOTORS, MOTOR_DIR_FORWARD);      // Set both motors to drive forward
  setMotorSpeed(LEFT_MOTOR, LeftMotorSpeed);          // Set the left motor to the MotorSpeed value defined above
  setMotorSpeed(RIGHT_MOTOR, MotorSpeed);    // Since the right motor is driving further than the left, set the speed of the right motor to drive at the proportionally higher speed calculated above
  int L_Circle_Pulse_Count = 0;     // Zero the encoder count
  int R_Circle_Pulse_Count = 0;     // Zero the encoder count

  while((R_Circle_Pulse_Count < Circle_Outer_Wheel_Pulses + 25)) {
    // Run this loop until the number of pulses read by the microcontroller reaches the calculated value above
    L_Circle_Pulse_Count = getEncoderLeftCnt();      // Read the left encoder value
    R_Circle_Pulse_Count = getEncoderRightCnt();     // Read the right encoder value

    if((R_Circle_Pulse_Count + 10 < L_Circle_Pulse_Count * SpeedRatio)) {
      // If the right motor is driving significantly slower than the SpeedRatio has specified, then speed up the right motor
      setMotorSpeed(RIGHT_MOTOR, MotorSpeed + 1);
    }

    if((R_Circle_Pulse_Count + 10 > L_Circle_Pulse_Count * SpeedRatio)) {
       // If the right motor is driving significantly faster than the SpeedRatio has specified, then slow down the right motor
      setMotorSpeed(RIGHT_MOTOR, MotorSpeed - 1);
    }

    if((R_Circle_Pulse_Count >= Circle_Outer_Wheel_Pulses + 25)) {
      // If the number of pulses reaches the calculated value, turn off the motors and stop running the function
      disableMotor(LEFT_MOTOR);       // Turn off the left motor
      disableMotor(RIGHT_MOTOR);      // Turn off the right motor
      delay(1000);
    }

    // Print encoder counts to the serial monitor for debugging
    Serial.print("Driving in a Circle Now");
    Serial.print("\t");
    Serial.print("Left Encoder Circle: ");
    Serial.print(L_Circle_Pulse_Count);
    Serial.print("\t");
    Serial.print("Right Encoder Circle: ");
    Serial.println(R_Circle_Pulse_Count);
    delay(100);
  } 
}

void loop() {
  // Make the robot perform the following functions sequentially with a one second delay between each phase 
  Serial.print("Straight Pulses: \t");   
  Serial.println(Straight_pulses);
  Serial.print("TurnCCW Pulses: \t");
  Serial.println(TurnCCW_pulses);
  Serial.print("Circle Inner Wheel Pulses: \t");
  Serial.println(Circle_Inner_Wheel_Pulses);
  Serial.print("Circle Outer Wheel Pulses: \t");
  Serial.println(Circle_Outer_Wheel_Pulses);
  Serial.print("Speed Ratio: \t");
  Serial.println(SpeedRatio);
  Serial.print("Left Motor Speed: \t");
  Serial.println(LeftMotorSpeed);
  
  delay(3000);         // Wait 3 seconds for setting the robot in position
  Drive_Straight();     // Perform the function for driving straight X centimeters
  delay(1000);
  Rotate_CCW();         // Perform the function for rotating 90 degrees CCW
  delay(1000);
  Drive_Circle();       // Perform the function for driving in a circle of radius X cm
  delay(1000);
  Rotate_CCW();         // Perform the function for rotating 90 degrees CCW
  delay(1000);
  Drive_Straight();     // Perform the function for driving straight back to the original start position
}
