#include <Arduino.h>
#include <Zumo32U4.h>
#include <event_timer.h>
#include "button.h"

#define turn90 500
#define arbitrarySpeed 20
#define time360 200
// one encoder measures faster than the other, use this to compensate
#define wheelBias 0
// PID constants
#define speedKp 12
#define speedKi 1

enum States {IDLE, WALL_FOLLOW, LINE_FOLLOW, TURN_90, DRIVE_STRAIGHT, SPIN, TESTING};
States state = TESTING;
// instantiate classes
EventTimer timer;
Button buttonC(17);
Zumo32U4Motors motors;
Zumo32U4Encoders encoders;

// declare prototypes
void setPidSpeed(float targetLeft, float targetRight); // TODO
float readWallDistance(); // TODO
float wallSpeed(float distance); // TODO
bool lineDetected(); // TODO
bool irDetected(); // TODO
int readLineSensor(); // TODO
float lineSpeeds(int lineVals); // TODO
bool angleToFlat(); // TODO
void configTimer();

// global variables, try to minimize
bool readyToPID;
volatile int16_t countsLeft;
volatile int16_t countsRight;

void setup() {
  buttonC.Init();
  configTimer();
}

void loop() {
  switch (state) {
    case IDLE: {
      // set constant speed
      setPidSpeed(0, 0);
      // check transition condition;
      if (buttonC.CheckButtonPress()) timer.Start(1000);
      if (timer.CheckExpired()) {
        state = WALL_FOLLOW;
        timer.Cancel();
      }
    }

    case WALL_FOLLOW: {
      // measure val
      float distance = readWallDistance();
      // PID calculation here
      float speeds = wallSpeed(distance);
      // set PID speed target
      setPidSpeed(0,0); // placeholder
      // check transition condition
      if (lineDetected()) {
        state = TURN_90;
        timer.Start(turn90);
      }
    }

    case LINE_FOLLOW: {
      // measure val
      int lineVals = readLineSensor();
      // PID calculation here
      float speeds = lineSpeeds(lineVals);
      // set PID speed target
      setPidSpeed(0,0); // placeholder
      // check transition condition
      if (irDetected()) {
        state = TURN_90;
        timer.Start(turn90);
      }
    }

    case TURN_90: {
      static bool doneOnce = false;
      // set constant speed
      setPidSpeed(arbitrarySpeed, 0);
      // check transition condition
      if (timer.CheckExpired()) {
        doneOnce ? state = DRIVE_STRAIGHT : state = LINE_FOLLOW;
      }
    }

    case DRIVE_STRAIGHT: {
      // set constant speed
      setPidSpeed(arbitrarySpeed, arbitrarySpeed);
      // check transition condition
      if (angleToFlat()) {
        state = SPIN;
        timer.Start(time360);
      }
    }

    case SPIN: {
      // set constant speed
      setPidSpeed(arbitrarySpeed, -arbitrarySpeed);
      // check transition condition
      if (timer.CheckExpired()) {
        timer.Cancel();
        state = IDLE;
      }
    }

    case TESTING: {
      static bool run = false;
      if (buttonC.CheckButtonPress()) run = !run;
      run ? setPidSpeed(10, 10) : setPidSpeed(0, 0);
      break;
    }
  }
}

void configTimer() {
  noInterrupts(); //disable interupts while we mess with the Timer4 registers
  //sets up timer 4
  TCCR4A = 0x00; // taken from example code, too scared to touch
  TCCR4B = 0x0C; // prescaler set to 2048
  TCCR4C = 0x04; // toggles pin 6 at the timer frequency
  TCCR4D = 0x00; // normal mode

  OCR4C = 141;   // overflow every
  TIMSK4 = 0x04; //enable overflow interrupt

  interrupts(); //re-enable interrupts
}

void setPidSpeed(float targetLeft, float targetRight) {
  if (!readyToPID) return; // bail if not ready to Pid
  readyToPID = false;
  // need to correct for difference in wheel encoders, wheelBias determined expirimentally
  if (targetRight != 0) targetRight += wheelBias;
  //for tracking previous counts
  static int16_t prevLeft = 0;
  static int16_t prevRight = 0;
  //error sum
  static int16_t sumLeft = 0;
  static int16_t sumRight = 0;

  // disable interrupts while reading from volatile variable
  noInterrupts();
  int16_t speedLeft = countsLeft - prevLeft;
  int16_t speedRight = countsRight - prevRight;
  prevLeft = countsLeft;
  prevRight = countsRight;
  interrupts();

  int16_t errorLeft = targetLeft - speedLeft;
  sumLeft += errorLeft;

  float effortLeft = speedKp * errorLeft + speedKi * sumLeft;

  int16_t errorRight = targetRight - speedRight;
  sumRight += errorRight;

  float effortRight = speedKp * errorRight + speedKi * sumRight;

  motors.setSpeeds(effortLeft, effortRight); //up to you to add the right motor
}

float readWallDistance() {
  return 0.0;
}
float wallSpeed(float distance) {
  return 0.0;
}
bool lineDetected() {
  return false;
}
bool irDetected() {
  return false;
}
int readLineSensor() {
  return 0;
}
float lineSpeeds(int lineVals) {
  return 1.0;
}
bool angleToFlat() {
  return false;
}


// pid timer ISR, set flag and get encoder vals
ISR(TIMER4_OVF_vect)
{
  //Capture a "snapshot" of the encoder counts for later processing
  countsLeft = encoders.getCountsLeft();
  countsRight = encoders.getCountsRight();

  readyToPID = true;
}
