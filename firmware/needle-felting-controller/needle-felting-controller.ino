#define _USE_MATH_DEFINES
#include <ContinuousStepper.h>
#include <movingAvg.h>
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
  STATE_JOG_BCK,
  STATE_EMERGENCY
};

Button startButton;
Button stopButton;
Button jogFwdButton;
Button jogBackButton;
Button safetyFeedbackButton;

Solenoid punchValve;

MachineState state;

#define WIRECLOCK 400000L
hd44780_I2Clcd lcd(0x3C); // 0x3C address as per datasheet & wired configuration
char lcdString[21] = {0}; // empty string for temporary usage by LCD updating code
movingAvg speedPotAvg(5);
movingAvg frequencyPotAvg(5);

// Motor info
const float gearboxRatio = 4.25; // 1:4.25
const uint16_t microsteps = 400;
const uint16_t stepsPerRev = microsteps * gearboxRatio;
const float rollerDiameter = 40.0; // in mm
volatile float motorRequiredSpeed = 0.0;

// LCD geometry
const uint8_t LCD_COLS = 20;
const uint8_t LCD_ROWS = 4;

// Output pins
const uint8_t stepPin = Q0_1;
const uint8_t dirPin = Q0_2;
const uint8_t solenoidValvePin = Q0_3;

// Input pins
const uint8_t startBtnPin = I0_1;
const uint8_t stopBtnPin = I0_2;
const uint8_t forwardBtnPin = I0_4;
const uint8_t backwardBtnPin = I0_3;
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
volatile unsigned long lastLoopTime = 0;
volatile char floatTempString[8] = {0};

//AccelStepper roller(AccelStepper::DRIVER, stepPin, dirPin);
ContinuousStepper<StepperDriver> roller;

void setup() {
  lcd.begin(LCD_COLS, LCD_ROWS);
  lcd.clear();
  lcd.print("Starting");
  delay(5000);
  Serial.begin(115200);
  Serial.println("Starting");

  roller.begin(stepPin, dirPin);
  // Configure pin modes
  pinMode(startBtnPin, INPUT);
  pinMode(stopBtnPin, INPUT);
  pinMode(forwardBtnPin, INPUT);
  pinMode(backwardBtnPin, INPUT);
  pinMode(safetyFeedbackPin, INPUT);
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(solenoidValvePin, OUTPUT);
  
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

  //roller.setMaxSpeed(32767);
  //roller.setSpeed(0);

  state = STATE_IDLE;
  lcd.clear();
}

void loop() {
  roller.loop();

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
        Serial.println("Moving to state starting");
        lcd.setCursor(0, 0);
        lcd.print("Starting");
        lcd.setCursor(0, 3);
        lcd.print("Controls locked");

        // Fire cylinder once then set up for cyclic punching
        punchValve.fire();

        // Configure and start roller
        motorRequiredSpeed = (linearSpeed / (M_PI * (rollerDiameter / 1000)) / 60) * stepsPerRev; // convert m/s to steps per second
        roller.spin(motorRequiredSpeed);

        state = STATE_STARTING;
      }
      break;

    case STATE_STARTING:
      if(roller.speed() == motorRequiredSpeed)
      {
        lcd.setCursor(0, 0);
        lcd.print("Running!");
        Serial.println("Moving to state running");
        punchValve.fireCycle(punchFrequency);
        state = STATE_RUNNING;
      }
      break;

    case STATE_RUNNING:
      // Jump state if cylinder is in-cycle
      if(punchValve.getCycleState() == true)
      {
        Serial.println("Moving to state punching");
        state = STATE_PUNCHING;
      }
      // Move to stopping state
      if(stopButton.currentState == HIGH)
      {
        lcd.setCursor(0, 0);
        lcd.print("Stopping");
        Serial.println("Moving to state stopping");
        state = STATE_STOPPING;
      }
      break;

    case STATE_PUNCHING:
      // Revert back to running state when cylinder retracted
      if(punchValve.getCycleState() == false)
      {
        Serial.println("Moving to state running");
        state = STATE_RUNNING;
      }
      break;

    case STATE_STOPPING:
      punchValve.cancel();
      roller.stop();

      if(roller.speed() == 0)
      {
        Serial.println("Moving to state idle from stopping");
        state = STATE_IDLE;
      }
      break;

    case STATE_JOG_FWD:
      roller.spin((0.1*stepsPerRev));
      // Adjust state if command finished
      if(jogFwdButton.currentState == LOW)
      {
        Serial.println("Moving to state idle from jog forward");
        roller.stop();
        state = STATE_IDLE;
      }
      break;

    case STATE_JOG_BCK:
      roller.spin(-(0.1*stepsPerRev));
      // Adjust state if command finished
      if(jogBackButton.currentState == LOW)
      {
        Serial.println("Moving to state idle from jog back");
        roller.stop();
        state = STATE_IDLE;
      }
      break;

    case STATE_EMERGENCY:
      // Stop piston in current position to allow for venting
      punchValve.stop();
      roller.stop();

      // Emergency is finished with, return to idle
      if(safetyFeedbackButton.currentState == LOW)
      {
        punchValve.cancel();
        state = STATE_IDLE;
      }
      break;
  }

  // Update moving averages at ~20 Hz
  if(loopCount % 5 == 0 && state == STATE_IDLE)
  {
    speedPotAvg.reading(analogRead(speedPotPin));
    frequencyPotAvg.reading(analogRead(frequencyPotPin));

    // Update calculated values
    linearSpeed = map(speedPotAvg.getAvg(), 0, 500, 0, (maxLinearSpeed * 100));
    linearSpeed = constrain(linearSpeed, 0, (maxLinearSpeed * 100)) / 100;
    punchFrequency = map(frequencyPotAvg.getAvg(), 0, 500, 1, maxPunchFrequency);
    punchFrequency = constrain(punchFrequency, 1, maxPunchFrequency);
  }

  // Update button states
  startButton.update();
  stopButton.update();
  jogFwdButton.update();
  jogBackButton.update();
  safetyFeedbackButton.update();

  // Update punch valve state
  punchValve.update();

  // Update LCD
  if(loopCount % 20 == 0 && state == STATE_IDLE)
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
    dtostrf(linearSpeed, 1, 2, floatTempString);
    snprintf(lcdString, 21, "Spd %s m/min", floatTempString);
    lcd.print(lcdString);
  }

  // Bump states from other inputs
  if(jogFwdButton.currentState == HIGH && state == STATE_IDLE)
  {
    Serial.println("Moving to state jog forward");
    state = STATE_JOG_FWD;
    lcd.setCursor(0, 3);
    lcd.print("Jog forward");
  }

  if(jogBackButton.currentState == HIGH && state == STATE_IDLE)
  {
    Serial.println("Moving to state jog back");
    state = STATE_JOG_BCK;
    lcd.setCursor(0, 3);
    lcd.print("Jog back");
  }

  // Move to emergency state if 
  if(safetyFeedbackButton.currentState == HIGH && state != STATE_EMERGENCY)
  {
    Serial.println("Moving to state emergency");
    state = STATE_EMERGENCY;
    lcd.setCursor(0, 3);
    lcd.print("Emergency stop!");
  }

  if(millis() - 1 > lastLoopTime)
  {
    loopCount++;
  }
  lastLoopTime = millis();
}
