#include <Arduino.h>
#include <Zumo32U4.h>
#include <event_timer.h>
#include "button.h"
#include "ultrasonic.h"

#define turn90 800
#define arbitrarySpeed 15
#define time360 200
// one encoder measures faster than the other, use this to compensate
#define wheelBias 0
// PID constants
#define speedKp 12
#define speedKi 1
// wall Pid constants
#define targetDist 20.0
#define wallKp .5
#define wallKd 5.0
#define baseSpeed 20.0
//line Pid constants
#define lineKp 0.1
#define lineKd 0.02

enum States {IDLE, WALL_FOLLOW, LINE_FOLLOW, TURN_90, DRIVE_STRAIGHT, SPIN, TESTING};
States state = IDLE;
// instantiate classes
EventTimer timer;
Button buttonC(17);
Zumo32U4Motors motors;
Zumo32U4Encoders encoders;
Zumo32U4LineSensors lineSensors;


// declare prototypes
void setPidSpeed(float targetLeft, float targetRight);
void wallPid(float distance);
bool lineDetected();
void linePid();
bool irDetected(); // TODO
bool angleToFlat(); // TODO
void configTimer();

// global variables, try to minimize
bool readyToPID;
volatile int16_t countsLeft;
volatile int16_t countsRight;
long count = 0;
uint16_t lineSensorValues[5];

void setup() {
  buttonC.Init();
  configTimer();
  configUltrasonic();
  lineSensors.initFiveSensors();
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
      break;
    }

    case WALL_FOLLOW: {
      // measure val
      float distance = getDist();
      // PID calculation here
      wallPid(distance);
      // check transition condition
      if (lineDetected()) {
        state = TURN_90;
        timer.Start(turn90);
      }
      break;
    }

    case LINE_FOLLOW: {
      // line Pid calc
      linePid();
      // check transition condition
      // if (irDetected()) {
      //   state = TURN_90;
      //   timer.Start(turn90);
      // }
      if (buttonC.CheckButtonPress()) state = IDLE;
      break;
    }

    case TURN_90: {
      static bool doneOnce = false;
      // set constant speed
      setPidSpeed(15, -8); // want this turn to be a little obtuse
      // check transition condition
      if (timer.CheckExpired()) {
        state = LINE_FOLLOW;
        timer.Cancel();
      }
      break;
    }

    case DRIVE_STRAIGHT: {
      // set constant speed
      setPidSpeed(arbitrarySpeed, arbitrarySpeed);
      // check transition condition
      if (angleToFlat()) {
        state = SPIN;
        timer.Start(time360);
      }
      break;
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
      float dist = getDist();
      static bool run = false;
      if (buttonC.CheckButtonPress()) run = !run;
      run ? wallPid(dist) : setPidSpeed(0, 0);

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

void wallPid(float distance) {
  static float prevWallError = 0;
  static float prevDist = 0;
  static float prevLeftSpeed = 0;
  static float prevRightSpeed = 0;
  // function is called continuously but distance is only updated occaisionally, if distance hasn't changed just uses previous speed
  if (distance == prevDist) {
    setPidSpeed(prevLeftSpeed, prevRightSpeed);
    return;
  }
  prevDist = distance;
  float wallError = targetDist - distance;

  float deltaError = wallError - prevWallError;

  float adjError = wallKp * wallError + wallKd * deltaError;

  if (adjError < -5) {
    adjError = -5;
  } else if (adjError > 10) {
    adjError = 10;
  }

  float leftspeed = baseSpeed + adjError/2;
  float rightSpeed = baseSpeed - adjError/2;

  prevLeftSpeed = leftspeed;
  prevRightSpeed = rightSpeed;

  setPidSpeed(leftspeed, rightSpeed);
  prevWallError = wallError;
}

bool lineDetected() {
  lineSensors.read(lineSensorValues, true);
  for (int i=0; i<5; i++) {
    if (lineSensorValues[i] <= 300) return true;
  }
  return false;
}

void linePid() {
  lineSensors.read(lineSensorValues, true);
  int lineError = lineSensorValues[1] - lineSensorValues[3];
  static int prevError = 0;
  int deltaError = lineError - prevError;

  float adjError = lineKp * lineError - lineKd * deltaError;

  prevError = lineError;


  setPidSpeed(baseSpeed + adjError, baseSpeed - adjError);

}

bool irDetected() {
  return false;
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
