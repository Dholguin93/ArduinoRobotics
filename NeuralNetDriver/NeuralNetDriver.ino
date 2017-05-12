/********
 * By: Nevo Mantel & Diego Holguin
 * CSC375
 * Project 2 - Artificial Neural Network
 * Prof. J. Blake
 ********/
 
#include <ANN_EEPROM.h>
#include "MePort.h"
#include "MeMCore.h"

#define TWO_SENSOR // If Two Sensor
#define LEARN_STATE 0 // State to Learn
#define ROTATY_STATE 1 // State to Rotate
#define MOVE_STATE 2  // State to Move after Learning and Rotating

// Outside Devices 
MeLineFollower lineFinder(PORT_4); // LineFinder Sensor
Me7SegmentDisplay disp(PORT_3); // 7 - Segment Display Reference
MeDCMotor rightMotor(M1); // Right Motor Reference
MeDCMotor leftMotor(M2); // Left Motor Reference

// Forward Function Declarations
void LearningState();
void RotateState();
void MoveState();

int training; // Number of training attempts
int currentState; // The current state of the robot
int previousState;  // The previous state of the 2 Line Sensor
bool previousBoolState; // The previous state of the 1 Line Sensor
int moveSpeed;  // The speed of robot
int analogThreshold = 512; // Threshold of analog value that determines if the button is pressed or not 
bool buttonPressed;  // The button was pressed

float learningRate = 0.8; // The rate of learning
float momentum = 0.9; // The momentum rate

void setup() {
  training = 0; // Initializing the training count
  
  buttonPressed = false;  // Initializing button pressed
  
  moveSpeed = 140;  // Initializing Speed
  
  #ifdef TWO_SENSOR
    inputNodes = 2; // Input Nodes for 2 line sensor
    hiddenNodes = 4;  // Hidden Nodes for 2 line sensor
    outputNodes = 4;  // Output Nodes for 2 line sensor
    patternCount = 4; // Pattern Count for 2 line sensor
    uint16_t patterns[] = {58, 5, 22, 41};  // Patterns for 2 line sensor
  #else
    inputNodes = 1; // Input Nodes for 1 line sensor
    hiddenNodes = 4;  // Hidden Nodes for 1 line sensor
    outputNodes = 2;  // Output Nodes for 1 line sensor
    patternCount = 2; // Pattern Count for 1 line sensor
    uint16_t patterns[] = {1, 6};  // Patterns for 1 line sensor
  #endif
  targetSuccess = 0.0008; // Target success rate
  curSuccess = -1.0;  // Current Success rate
  
  // Open serial connection
  Serial.begin(115200);
  
  disp.init();  // Initializing display
  
  disp.set(BRIGHTNESS_2); // Brightening the display
  
  while (!Serial) {}
  
  createArrays(); // Creating Arrays
  
  createWeights(0.5); // Creating Weights
  
  writeToEEPROM(patterns);  // Writing patterns to memory
  
  int addressB = 0; // Creates a temp address
  
  readEEPROM(&addressB);  // Reads from address 0 in memory  
}

void loop() 
{
  int buttonValue = analogRead(A7); // The value of the button
  
  if(buttonValue < analogThreshold && !buttonPressed) // Check if the button is pressed
  { 
    buttonPressed = true; // Button was pressed
    currentState = ROTATY_STATE;  // Makes current state rotate
  } 
  
  if(buttonPressed)
  { 
    disp.display(training); // Display the training count
    
    if(currentState == LEARN_STATE) LearningState();  // Learn State Function
    else if (currentState == ROTATY_STATE) RotateState();  // Rotate State Function
    else if (currentState == MOVE_STATE) MoveState();  // Move State Function
  }
}

/*************************************************
 * This function will train the neural network 
 * with the data brought from the sensors or sensor.
 **************************************************/
void LearningState(){
   #ifdef TWO_SENSOR
   
    // Obtain sensor related data, and based upon results return, call the appropriete function
    int sensorState = lineFinder.readSensors();
    
    switch(sensorState)
    {
      case S1_IN_S2_IN: // Sensor 1 and 2 are inside of black line
      
        if(sensorState != previousState)
        { 
          if(trainOnce(0,learningRate, momentum) > targetSuccess)
          { 
            training++; // Training count incremented
            
            previousState = sensorState;  // Make previous state the sensor state
            
            currentState = ROTATY_STATE;  // Go to Rotate State
            
          } 
          else currentState = MOVE_STATE;
        }
        else currentState = ROTATY_STATE;
        
        break;
      case S1_IN_S2_OUT: // Sensor 2 is outside of black line
      
        if(sensorState != previousState)
        { 
          if(trainOnce(3,learningRate, momentum) > targetSuccess)
          { 
            training++; // Training count incremented
            
            previousState = sensorState;  // Make previous state the sensor state
            
            currentState = ROTATY_STATE;  // Go to Rotate State
          } 
          else currentState = MOVE_STATE;
        }
        else currentState = ROTATY_STATE;
        
        break;
      case S1_OUT_S2_IN: // Sensor 1 is outside of black line
        //Send 0,1 (01,0110)
        if(sensorState != previousState){ // If the sensor state is not equal to previous state
          if(trainOnce(2,learningRate, momentum) > targetSuccess){  // While the error rate is greater than target rate
            //Serial.print("\n");
            //Serial.print(trainOnce(2,learningRate, momentum),4);
            training++; // Training count incremented
            previousState = sensorState;  // Make previous state the sensor state
            currentState = ROTATY_STATE;  // Go to Rotate State
          } else {  // If the error rate is less than go to Move State
            currentState = MOVE_STATE;
          }
        } else {  // If the sensor state is equal to previous state then go to Rotate State
          currentState = ROTATY_STATE;
        }
        break;
      case S1_OUT_S2_OUT: // Sensor 1 and 2 are outside of black line
        //Send 0,0 (00,0101)
        if(sensorState != previousState){ // If the sensor state is not equal to previous state
          if(trainOnce(1,learningRate, momentum) > targetSuccess){  // While the error rate is greater than target rate
            //Serial.print("\n");
            //Serial.print(trainOnce(1,learningRate, momentum),4);
            training++; // Training count incremented
            previousState = sensorState;  // Make previous state the sensor state
            currentState = ROTATY_STATE;  // Go to Rotate State
          } else {  // If the error rate is less than go to Move State
            currentState = MOVE_STATE;
          }
        } else {  // If the sensor state is equal to previous state then go to Rotate State
          currentState = ROTATY_STATE;
        }
        break;
      default: 
        break;
    }
   #else
    // Obtain sensor related data, and based upon results return, call the appropriete function
    bool sensorState = lineFinder.readSensor1();
    switch(sensorState)
    {
      case true:
      //Send 1 (1,10)
       if(sensorState != previousBoolState){ // If the sensor state is not equal to previous state boolean
        if(trainOnce(1,learningRate, momentum) > targetSuccess){  // While the error rate is greater than target rate
          training++; // Training count incremented
          previousBoolState = sensorState;  // Make previous state the sensor state
          currentState = ROTATY_STATE;  // Go to Rotate State
        } else {
          currentState = MOVE_STATE;
        }
       }
        break;
      case false:
      //Send 0 (0,01)
      if(sensorState != previousBoolState){ // If the sensor state is not equal to previous state boolean
        if(trainOnce(0,learningRate, momentum) > targetSuccess){  // While the error rate is greater than target rate
          training++; // Training count incremented
          previousBoolState = sensorState;  // Make previous state the sensor state
          currentState = ROTATY_STATE;  // Go to Rotate State
        } else {
          currentState = MOVE_STATE;  // Go to Move State
        }
      }
        break;
      default: 
        break;
    }
   #endif
}

/*************************************************
 * This function will rotate the robot.
 **************************************************/
void RotateState(){
  leftMotor.run(-moveSpeed);
  rightMotor.run(-moveSpeed);
  currentState = LEARN_STATE;
}

/*************************************************
 * This function will move the robot based on the evaluate 
 * function in the header file, based on the output given.
 **************************************************/
void MoveState(){
  #ifdef TWO_SENSOR
    uint8_t inp[inputNodes];  // Creates input array of size input nodes
    float outp[outputNodes];  // Creates output array of size output nodes
    // Obtain sensor related data, and based upon results return, call the appropriete function
    int sensorState = lineFinder.readSensors();
    switch(sensorState)
    {
      case S1_IN_S2_IN: // Sensor 1 and 2 are inside of black line
        //Evaluate 1,1 (11,1010)
        inp[0] = 1;
        inp[1] = 1;
        evaluate(inp,outp); // Evaluate the Input 
        break;
      case S1_IN_S2_OUT: // Sensor 2 is outside of black line
        //Evaluate 1,0 (10,1001)
        inp[0] = 1;
        inp[1] = 0;
        evaluate(inp,outp);  // Evaluate the Input 
        break;
      case S1_OUT_S2_IN: // Sensor 1 is outside of black line
        //Evaluate 0,1 (01,0110)
        inp[0] = 0;
        inp[1] = 1;
        evaluate(inp,outp); // Evaluate the Input 
        break;
      case S1_OUT_S2_OUT: // Sensor 1 and 2 are outside of black line
        //Evaluate 0,0 (00,0101)
        inp[0] = 0;
        inp[1] = 0;
        evaluate(inp,outp); // Evaluate the Input 
        break;
      default: 
        break;
    }
    if(outp[0] > 0.5 && outp[2] > 0.5){ // Check if both motors need to be on
      leftMotor.run(-moveSpeed);
      rightMotor.run(moveSpeed);
    } else if (outp[0] > 0.5 && outp[2] <= 0.5){  // Check if left motor is on and right motor is off
      leftMotor.run(moveSpeed);
      rightMotor.run(moveSpeed);
    } else if (outp[0] <= 0.5 && outp[2] > 0.5){  // Check if left motor is off and right motor is on
      leftMotor.run(-moveSpeed);
      rightMotor.run(-moveSpeed);
    } else if (outp[0] <= 0.5 && outp[2] <= 0.5) {  // Check if both motors are off
      leftMotor.run(moveSpeed);
      rightMotor.run(-moveSpeed);
    }
  #else
    uint8_t inp[inputNodes];  // Creates input array of size input nodes
    float outp[outputNodes];  // Creates output array of size output nodes
    // Obtain sensor related data, and based upon results return, call the appropriete function
    bool sensorState = lineFinder.readSensor1();
    switch(sensorState)
    {
      case true:
      //Send 1 (1,10)
        inp[0] = 1;
        evaluate(inp,outp); // Evaluate the Input 
        break;
      case false:
      //Send 0 (0,01)
        inp[0] = 0;
        evaluate(inp,outp); // Evaluate the Input 
        break;
      default: 
        break;
    }
    if(outp[0] > 0.5){  // Check if left motor is on
      leftMotor.run(-moveSpeed);
      rightMotor.stop();
    } else {  // Check if right motor is on
      leftMotor.stop();
      rightMotor.run(moveSpeed);
    }
  #endif
}
