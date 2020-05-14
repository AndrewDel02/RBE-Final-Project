#include <Arduino.h>
#include <Zumo32U4.h>
#include <event_timer.h>
#include "button.h"
#include "ultrasonic.h"
#include "filter.h"

// timer durationts
#define turn90 800
#define turn2 475
#define time360 200
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
// dimensions, in m
#define wheelTrack 0.098
#define speedScale 53.0

// state machine
enum States {IDLE, WALL_FOLLOW, LINE_FOLLOW, TURN_90, DRIVE_STRAIGHT, SPIN, TESTING, TURN2, WAIT_FOR_OK};
States state = IDLE;
// instantiate classes
EventTimer timer;
Button buttonC(17);
Zumo32U4Motors motors;
Zumo32U4Encoders encoders;
ComplementaryFilter filter;
Zumo32U4LineSensors lineSensors;
Zumo32U4ProximitySensors proxSensors;


// declare prototypes
void setPidSpeed(float targetLeft, float targetRight);
void wallPid(float distance);
bool lineDetected();
void linePid();
bool irDetected();
bool angleToFlat();
void configTimer();
bool checkSpinDone();

// global variables, try to minimize
bool readyToPID;
volatile int16_t countsLeft;
volatile int16_t countsRight;
uint16_t lineSensorValues[5];

void setup() {
  buttonC.Init();
  configTimer();
  configUltrasonic();
  lineSensors.initFiveSensors();
  proxSensors.initFrontSensor();
  filter.init();
}

void loop() {
  switch (state) {
    // starting state, do nothing
    // transition condition: timer triggered by button press
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

    // read wall distance then set speeds
    // transition condition: line detected
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

    // read line values then set speeds
    // transition condition: IR signal detected
    case LINE_FOLLOW: {
      // line Pid calc
      linePid();
      // check transition condition
      if (irDetected()) {
        Serial.println("message");
        state = TURN2;
        timer.Start(turn2);
      }
      if (buttonC.CheckButtonPress()) state = IDLE; // Estop
      break;
    }

    // turn 90 deg right
    // transition condition: timer expires
    case TURN_90: {
      // set constant speed
      setPidSpeed(15, -8); // want this turn to be a little obtuse to get a good line position
      // check transition condition
      if (timer.CheckExpired()) {
        state = LINE_FOLLOW;
        timer.Cancel();
      }
      break;
    }

    // turn 90 deg left
    // transition condition: tiemr expires
    case TURN2: {
      // set constant speed
      setPidSpeed(-20, 19);
      // check transition condition
      if (timer.CheckExpired()) {
        timer.Cancel();
        state = WAIT_FOR_OK;
        setPidSpeed(0, 0);
      }
      break;
    }

    // wait for IR signal to make sure robot in safe play
    // transition condition: IR signal
    case WAIT_FOR_OK: {
      // want to wait at bottom and top of ramp, so need a done once flag
      static bool doneOnce = false;
      // chill
      setPidSpeed(0, 0);
      // check transition condition
      if (irDetected()) {
        if (!doneOnce) {
          // done waiting at bottom of ramp
          state = DRIVE_STRAIGHT;
          doneOnce = true;
        } else {
          // done waiting at top of ramp
          state = SPIN;
          doneOnce = false;
        }

      }
      break;
    }

    // drive straight up ramp
    // transition condition: angle to flat
    case DRIVE_STRAIGHT: {
      // set constant speed
      setPidSpeed(20, 20);
      // check transition condition
      if (angleToFlat()) {
        state = WAIT_FOR_OK;
      }
      // E stop, really don't want to fall off the ramp (again)
      if (buttonC.CheckButtonPress()) state = WAIT_FOR_OK;
      break;
    }

    // incomplete, spin 360 deg
    // transition condition: turned 360 as measured by forward kinematics
    case SPIN: {
      // set constant speed
      setPidSpeed(15,-15);
      // check for transition condition
      if (checkSpinDone()) state = IDLE;

      if (buttonC.CheckButtonPress()) state = TESTING;

    }

    // testing state, used during development
    case TESTING: {
      static bool run = false;

      run ? setPidSpeed(15, -15) : setPidSpeed(0, 0);

      if (buttonC.CheckButtonPress()) run = !run;

      if (checkSpinDone()) run = false;

    }
  }
}

// configure timer and enable interrupts
void configTimer() {
  noInterrupts(); //disable interupts while we mess with the Timer4 registers
  //sets up timer 4
  TCCR4A = 0x00; // taken from example code, too scared to touch
  TCCR4B = 0x0C; // prescaler set to 2048
  TCCR4C = 0x04; // toggles pin 6 at the timer frequency
  TCCR4D = 0x00; // normal mode

  OCR4C = 141;
  TIMSK4 = 0x04; //enable overflow interrupt

  interrupts(); //re-enable interrupts
}

// take target speed and use PI controller to set speed
void setPidSpeed(float targetLeft, float targetRight) {
  if (!readyToPID) return; // bail if not ready to PID
  readyToPID = false;
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

  // get error
  int16_t errorLeft = targetLeft - speedLeft;
  sumLeft += errorLeft;
  int16_t errorRight = targetRight - speedRight;
  sumRight += errorRight;

  // use PI controller to get effort
  float effortLeft = speedKp * errorLeft + speedKi * sumLeft;
  float effortRight = speedKp * errorRight + speedKi * sumRight;
  // set speed
  motors.setSpeeds(effortLeft, effortRight); //up to you to add the right motor
}

// take distance and set speeds
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

  // PD controller
  float adjError = wallKp * wallError + wallKd * deltaError;

  // limit amount it can turn, especially when turning towards the wall, to avoid spin
  if (adjError < -5) {
    adjError = -5;
  } else if (adjError > 10) {
    adjError = 10;
  }

  float leftspeed = baseSpeed + adjError/2;
  float rightSpeed = baseSpeed - adjError/2;
  prevLeftSpeed = leftspeed;
  prevRightSpeed = rightSpeed;

  // set speeds
  setPidSpeed(leftspeed, rightSpeed);
  prevWallError = wallError;
}

// check for any line
bool lineDetected() {
  lineSensors.read(lineSensorValues, true);
  // want to be as forgiving as possible, just check for white on any sensor
  for (int i=0; i<5; i++) {
    if (lineSensorValues[i] <= 300) return true;
  }
  return false;
}

// read line values and set speed
void linePid() {
  lineSensors.read(lineSensorValues, true);
  // try to keep center left and right sensors equal
  int lineError = lineSensorValues[1] - lineSensorValues[3];
  static int prevError = 0;
  int deltaError = lineError - prevError;

  // PD controller
  float adjError = lineKp * lineError - lineKd * deltaError;

  prevError = lineError;
  // set speeds
  setPidSpeed(baseSpeed + adjError, baseSpeed - adjError);
}

// check for raw IR signal
bool irDetected() {
  proxSensors.lineSensorEmittersOff(); // briefly turn off emitters to avoid false positives
  if (proxSensors.readBasicFront()) return true; // basic read, doesn't interpret signal
  return false;
}

// check if robot has gone to flat after incline
bool angleToFlat() {
  float est;
  bool retVal = false;
  static bool onIncline = false;
  bool newReading = filter.CalcAngle(est); // get new est angle
  static int threshold = -30;

  if (!newReading) return false; // if nothing new don't bother calc

  // use hysteresis filter to make sure it goes up then down
  if (est < threshold && !onIncline) {
    threshold = 10;
    onIncline = true;
    Serial.println(est);
  }

  if (est > threshold && onIncline) {
    threshold = -30;
    onIncline = false;
    retVal = true;
  }

  return retVal;
}

// incomplete
bool checkSpinDone() {
  bool retVal = false;
  static float theta = 0;
  static int16_t prevLeft = 0;
  static int16_t prevRight = 0;
  static int prevTime = 0;
  // get current speed
  noInterrupts();
  int16_t speedLeft = countsLeft - prevLeft;
  int16_t speedRight = countsRight - prevRight;
  prevLeft = countsLeft;
  prevRight = countsRight;
  interrupts();

  // speedLeft = speedLeft / speedScale;
  // speedRight = speedRight / speedScale;

  int currentTime = millis();
  float deltaT = (currentTime - prevTime);

  // if (deltaT==0) return false;

  float omega = (speedRight - speedLeft) / wheelTrack;

  theta += omega * deltaT; // increment angle by new measurement


  prevTime = currentTime;


  if (theta < -1500) {
    retVal = true;
    theta = 0;
  }
  return retVal;

}


// pid timer ISR, set flag and get encoder vals
ISR(TIMER4_OVF_vect)
{
  //Capture a "snapshot" of the encoder counts for later processing
  countsLeft = encoders.getCountsLeft();
  countsRight = encoders.getCountsRight();

  readyToPID = true;
}
