#define _USE_MATH_DEFINES
#include <movingAvg.h>
#include <FlexyStepper.h>
#include <Wire.h>
#include <hd44780.h>
#include <hd44780ioClass/hd44780_I2Clcd.h>
#include <math.h>
#include "button.h"
#include "solenoid.h"

enum MachineState{
  STATE_IDLE,
  STATE_STARTING,
  STATE_RUNNING,
  STATE_PUNCHING,
  STATE_STOPPING,
  STATE_JOG_FWD,
  STATE_JOG_BACK,
  STATE_EMERGENCY
};

Button startButton;
Button stopButton;
Button jogFwdButton;
Button jogBackButton;
Button safetyFeedbackButton;

Solenoid punchValve;

MachineState state;
FlexyStepper roller;
hd44780_I2Clcd lcd(0x3C); // 0x3C address as per datasheet & wired configuration
char lcdString[21] = {0}; // empty string for temporary usage by LCD updating code
movingAvg speedPotAvg(10);
movingAvg frequencyPotAvg(10);

// Motor info
const float gearboxRatio = 4.25; // 1:4.25
const uint16_t microsteps = 400;
const uint16_t stepsPerRev = microsteps * gearboxRatio;
const float rollerDiameter = 25.4; // in mm
volatile float motorRequiredSpeed = 0.0; 

// LCD geometry
const uint8_t LCD_COLS = 20;
const uint8_t LCD_ROWS = 4;

// Output pins
const uint8_t stepPin = Q0_1;
const uint8_t dirPin = Q0_2;
const uint8_t solenoidValvePin = Q0_0;

// Input pins
const uint8_t startBtnPin = I0_1;
const uint8_t stopBtnPin = I0_2;
const uint8_t forwardBtnPin = I0_3;
const uint8_t backwardBtnPin = I0_4;
const uint8_t safetyFeedbackPin = I0_5;
const uint8_t speedPotPin = I0_6;
const uint8_t frequencyPotPin = I0_7;

// Speed & punch frequency variables
const float maxLinearSpeed = 2.0; // metres per minute
const uint8_t maxPunchFrequency = 200; // punches per minute
volatile float linearSpeed = 0.0;
volatile uint8_t punchFrequency = 0;

// Misc
volatile uint16_t loopCount = 0;

void setup() {
  // put your setup code here, to run once:
  lcd.begin(LCD_COLS, LCD_ROWS);
  lcd.print("Starting");

  // Configure pin modes
  pinMode(startBtnPin, INPUT);
  pinMode(stopBtnPin, INPUT);
  pinMode(forwardBtnPin, INPUT);
  pinMode(backwardBtnPin, INPUT);
  pinMode(safetyFeedbackPin, INPUT);
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(solenoidValvePin, OUTPUT);
  
  roller.connectToPins(stepPin, dirPin);
  roller.setStepsPerRevolution(stepsPerRev);
  speedPotAvg.begin();
  frequencyPotAvg.begin();

  // Configure buttons
  startButton.pin = startBtnPin;
  stopButton.pin = stopBtnPin;
  jogFwdButton.pin = forwardBtnPin;
  jogBackButton.pin = backwardBtnPin;
  safetyFeedbackButton.pin = safetyFeedbackPin;

  // Configure valve
  punchValve.pin = solenoidValvePin;

  state = STATE_IDLE;
  lcd.clear();
}

void loop() {
  if (loopCount == 1000)
  {
    loopCount = 0;
  }

  // THE BIG OL' STATE MACHINE
  switch(state) {
    case STATE_IDLE:
      // Check and update state
      if(startButton.currentState == HIGH) 
      {
        state = STATE_STARTING;
      }
      break;

    case STATE_STARTING:
      // Fire cylinder once then set up for cyclic punching
      punchValve.fire();

      // Configure and start roller
      motorRequiredSpeed = (linearSpeed / (M_PI * (rollerDiameter / 1000))); // convert m/s to rps
      roller.setSpeedInRevolutionsPerSecond(motorRequiredSpeed);
      roller.setAccelerationInRevolutionsPerSecondPerSecond(motorRequiredSpeed);

      punchValve.fireCycle(punchFrequency);
      state = STATE_RUNNING;
      break;

    case STATE_RUNNING:
      // Jump state if cylinder is in-cycle
      if(punchValve.getCycleState() == true)
      {
        state = STATE_PUNCHING;
      }
      // Move to stopping state
      if(stopButton.currentState == HIGH)
      {
        state = STATE_STOPPING;
      }
      break;

    case STATE_PUNCHING:
      // Revert back to running state when cylinder retracted
      if(punchValve.getCycleState() == false)
      {
        state = STATE_RUNNING;
      }
      break;

    case STATE_STOPPING:
      punchValve.cancel();
      
      roller.setSpeedInRevolutionsPerSecond(0);
      roller.setAccelerationInRevolutionsPerSecondPerSecond(motorRequiredSpeed);

      if(roller.motionComplete())
      {
        state = STATE_IDLE;
      }
      break;

    case STATE_JOG_FWD:
      roller.setSpeedInRevolutionsPerSecond(0.5);
      roller.setAccelerationInRevolutionsPerSecondPerSecond(1.0);
      // Adjust state if command finished
      if(jogFwdButton.currentState == LOW)
      {
        roller.setSpeedInRevolutionsPerSecond(0);
        roller.setAccelerationInRevolutionsPerSecondPerSecond(10);
        state = STATE_IDLE;
      }
      break;

    case STATE_JOG_BACK:
      roller.setSpeedInRevolutionsPerSecond(-0.5);
      roller.setAccelerationInRevolutionsPerSecondPerSecond(1.0);
      // Adjust state if command finished
      if(jogFwdButton.currentState == LOW)
      {
        roller.setSpeedInRevolutionsPerSecond(0);
        roller.setAccelerationInRevolutionsPerSecondPerSecond(10);
        state = STATE_IDLE;
      }
      break;

    case STATE_EMERGENCY:
      // Stop piston in current position to allow for venting
      punchValve.stop();
      // Bring roller to a rapid stop
      roller.setSpeedInRevolutionsPerSecond(0);
      roller.setAccelerationInRevolutionsPerSecondPerSecond(100);

      // Emergency is finished with, return to idle
      if(safetyFeedbackButton.currentState == LOW)
      {
        state = STATE_IDLE;
      }
      break;
  }

  // Update moving averages
  speedPotAvg.reading(analogRead(speedPotPin));
  frequencyPotAvg.reading(analogRead(frequencyPotPin));

  // Update calculated values
  linearSpeed = map(speedPotAvg.getAvg(), 5, 500, 0, maxLinearSpeed);
  linearSpeed = constrain(linearSpeed, 0, maxLinearSpeed);
  punchFrequency = map(frequencyPotAvg.getAvg(), 5, 500, 0, maxPunchFrequency);
  punchFrequency = constrain(punchFrequency, 0, maxPunchFrequency);

  // Update button states
  startButton.update();
  stopButton.update();
  jogFwdButton.update();
  jogBackButton.update();
  safetyFeedbackButton.update();

  // Update punch valve state
  punchValve.update();

  // Update LCD at ~10 Hz
  if(loopCount % 100 == 0)
  {
    lcd.clear();
    memset(lcdString, 0, 21);
    snprintf(lcdString, 21, "ST: %d", state);
    lcd.print(lcdString);
    
    lcd.setCursor(0, 1);
    memset(lcdString, 0, 21);
    snprintf(lcdString, 21, "Frq %d punch/min", punchFrequency);
    lcd.print(lcdString);
    
    lcd.setCursor(0, 2);
    memset(lcdString, 0, 21);
    snprintf(lcdString, 21, "Spd %f.1 m/min", linearSpeed);
    lcd.print(lcdString);

    if(state == STATE_EMERGENCY)
    {
      lcd.setCursor(0, 3);
      lcd.print("Emergency stop!");
    }
  }

  // Bump states from other inputs
  if(jogFwdButton.currentState == HIGH && state == STATE_IDLE)
  {
    state = STATE_JOG_FWD;
  }

  if(jogBackButton.currentState == HIGH && state == STATE_IDLE)
  {
    state = STATE_JOG_BACK;
  }

  // Move to emergency state if 
  if(safetyFeedbackButton.currentState == HIGH)
  {
    state = STATE_EMERGENCY;
  }

  loopCount++;
}
