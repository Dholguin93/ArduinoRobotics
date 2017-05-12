// Writen by Diego & Nevo designed, tested, debugged, and implemented the Line Follower
// Chris also tested, and debugged, but single-handedly created nascar 
#include <Time.h>
#include <RH_ASK.h>
#include <SPI.h>
#include "MePort.h"
#include "MeMCore.h"
#include <RHReliableDatagram.h>

// Writen by Diego
#define ULTRASONIC_MIN_DISTANCE 10
#define NEVO_ADDRESS 53
#define DIEGO_ADDRESS 43
#define CHRIS_ADDRESS 63
#define STATE_ZERO   0
#define STATE_ONE   1
#define STATE_TWO 2

// Constants 
const int STEPTIME = 14; 
const int TURNTIME = 44;
const int PREFEREDMINSTEPS = 3; 
const int current_robot = DIEGO_ADDRESS;

MeDCMotor rightMotor(M1); // Right Motor Reference
MeDCMotor leftMotor(M2); // Left Motor Reference
MeLineFollower lineFinder(PORT_4); /* Line Finder module can only be connected to PORT_3, PORT_4, PORT_5, PORT_6 of base shield. */
MeUltrasonicSensor ultraSensor(PORT_2); /* Ultrasonic module can ONLY be connected to port 3, 4, 6, 7, 8 of base shield. */
Me7SegmentDisplay disp(PORT_3);

// Forward Function Declarations 
void MoveRobotLeft(uint8_t _speed);
void MoveRobotRight(uint8_t _speed);
void MoveForward(uint8_t _speed);
void MoveRight(uint8_t _speed);
void MoveLeft(uint8_t _speed);
void MoveBack(uint8_t _speed);
void StopMovement(); 
void TimingISR();
void recieveButtonMessage();
boolean robotWithinRange();
void sendButtonMessage();

// MBot speeds used for turning and moving forward 
uint8_t forwardSpeed = 235; 
uint8_t turnSpeed = 210; 

int stepCount, stepGrowth, distance; 
int analogThreshold = 512; // Threshold of analog value that determines if the button is pressed or not 

// THIS IS THE IMPORTANT STUFF !
// Lots of turn --> small number
// Not lots of turns --> bigger number 
double stepTime = STEPTIME; // How long the MBot moves forward  
double turnTime = TURNTIME; // How long the MBot turns 
unsigned long raceInMiliSeconds; 

unsigned char second, minute = 0 , hour = 12; 
uint8_t TimeDisp[] = { 0x00, 0x00, 0x00, 0x00 };
long    lastTime  = 0;

int state; 
double growthForSteps; 
boolean buttonPressed, endProgram, raceFinished, nascar;


// Singleton instance of the radio driver
RH_ASK driver;
// Class to manage message delivery and receipt, using the driver declared above
RHReliableDatagram manager(driver, DIEGO_ADDRESS);

void setup() 
{
  state = STATE_ZERO;
  nascar = false;
  raceFinished = false; 
  //growthForSteps = (((double)STEPTIME)/100.0); 
  
  Serial.begin(9600);
  if (!manager.init())
    Serial.println("init failed");
}

// Dont put this on the stack:
uint8_t recieveBuf[RH_ASK_MAX_MESSAGE_LEN];
uint8_t sendBuf[RH_ASK_MAX_MESSAGE_LEN]; 

void loop()
{

  if(state == STATE_ZERO)
  {
    // On any pass after the first one, just display the time, incremented by the previous robot time value 
    if(second != 0) 
    {
      // Update array to reflect the proper values with respect to minutes and seconds 
      TimeDisp[0] = minute / 10;
      TimeDisp[1] = minute % 10;
      TimeDisp[2] = second / 10;
      TimeDisp[3] = second % 10;
      
      // Display the time 
      disp.display(TimeDisp);
    }
    else disp.display(state);

    
    if (manager.available())
    {
      // Wait for a message addressed to us from the client
      uint8_t len = sizeof(recieveBuf);
      uint8_t from;
      
      if (manager.recvfromAck(recieveBuf, &len, &from))
      {
        char robotID [3] = { 0 };
        memcpy(robotID, recieveBuf, 3); 
        Serial.print("Message: "); Serial.println(robotID);
        
        if(atoi(robotID) == current_robot)
        {
          Serial.println("GOING TO STATE 1");
          state = STATE_ONE; 
        }
      }
    }

    // Read in the analog input of the button 
    int buttonValue = analogRead(A7);  

    // If the button is pressed only on Diego's robot 
    if(buttonValue < analogThreshold && current_robot == DIEGO_ADDRESS) {state = STATE_ONE;} 
    // For everyone else, just send message to Diego's robot 
    else if (buttonValue < analogThreshold && (current_robot == NEVO_ADDRESS || current_robot == CHRIS_ADDRESS))
    {
      // Send data telling Diego to start moving 
      uint8_t data [] = "43"; 
      
      // Try to send this message atleast 4 times 
      manager.sendtoWait(data, sizeof(data), DIEGO_ADDRESS); 
      manager.sendtoWait(data, sizeof(data), DIEGO_ADDRESS);   
      manager.sendtoWait(data, sizeof(data), DIEGO_ADDRESS);   
      manager.sendtoWait(data, sizeof(data), DIEGO_ADDRESS); 
    }
  }
  else if (state == STATE_ONE)
  {
      disp.display(state);

    // Update runtime variables 
    stepTime += (growthForSteps * stepCount); 

    // Only keep track of time the time when in state 1 
    if(!raceFinished && millis() - lastTime >= 1000) { TimingISR(); lastTime = millis(); } 

    // Obtain sensor related data, and based upon results return, call the appropriete function
    int sensorState = lineFinder.readSensors();
        
    switch(sensorState)
    {
      case S1_IN_S2_IN: // Sensor 1 and 2 are inside of black line
      if(!robotWithinRange()) MoveForward(forwardSpeed); 
      else state = STATE_TWO;
       break;
       case S1_IN_S2_OUT: // Sensor 2 is outside of black line
        if(!robotWithinRange()) MoveRobotLeft(turnSpeed);
        else state = STATE_TWO;
       break;
       case S1_OUT_S2_IN: // Sensor 1 is outside of black line
        if(!robotWithinRange()) MoveRobotRight(turnSpeed);
        else state = STATE_TWO;
       break;
       case S1_OUT_S2_OUT: // Sensor 1 and 2 are outside of black line
        if(!robotWithinRange())
        {
          if(nascar) MoveRobotRight(turnSpeed);
          else MoveBack(forwardSpeed - 30); 
        }
        else state = STATE_TWO;
        break;
        default: break;
     }
  }
  else if (state == STATE_TWO)
  {   
    // Stop all movement of the robot!
    StopMovement();

    // Update array to reflect the proper values with respect to minutes and seconds 
    TimeDisp[0] = minute / 10;
    TimeDisp[1] = minute % 10;
    TimeDisp[2] = second / 10;
    TimeDisp[3] = second % 10;
    
    // Display the time 
    disp.display(TimeDisp);

    if(current_robot == DIEGO_ADDRESS || current_robot == NEVO_ADDRESS)
    {
      // Tell Nevo that he needs to start moving ! 
      uint8_t data [] = "53"; 
      
      // Try and attempt to send this message atleast 4 times 
      manager.sendtoWait(data, sizeof(data), NEVO_ADDRESS); 
      manager.sendtoWait(data, sizeof(data), NEVO_ADDRESS);   
      manager.sendtoWait(data, sizeof(data), NEVO_ADDRESS);   
      manager.sendtoWait(data, sizeof(data), NEVO_ADDRESS);  
    }

    // Wait 2 seconds, then restart to state 0  
    delay(2000); 

    // Set state to state 0 
    state = STATE_ZERO;

    // Set time to start at 0 
    lastTime = 0;   

    disp.display(state);
  }
}

void MoveRobotLeft(uint8_t _speed)
{
  MoveLeft(_speed);
}

void MoveRobotRight(uint8_t _speed)
{
  MoveRight(_speed);
}

void MoveForward(uint8_t _speed)
{ 
  leftMotor.run(-_speed);
  rightMotor.run(_speed); 
  stepCount++; 
}

void MoveLeft(uint8_t _speed)
{
  leftMotor.run(_speed);
  rightMotor.run(_speed); 
  stepCount = 0; 
  stepTime = STEPTIME; 
}

void MoveRight(uint8_t _speed)
{
  leftMotor.run(-_speed);
  rightMotor.run(-_speed); 
  stepCount = 0; 
  stepTime = STEPTIME; 
}

void MoveBack(uint8_t _speed)
{
  stepCount = 0;
  leftMotor.run(_speed);
  rightMotor.run(-_speed); 
}

void StopMovement()
{
  leftMotor.stop();
  rightMotor.stop();
}

boolean robotWithinRange()
{
  return (ultraSensor.distanceCm() < ULTRASONIC_MIN_DISTANCE) ? true : false; 
}

void TimingISR()
{
  second++;
  if(second == 60)
  {
    minute++;
    if(minute == 60)
    {
      hour++;
      if(hour == 24)
      {
        hour = 0;
      }
      minute = 0;
    }
    second = 0;
  }
}

