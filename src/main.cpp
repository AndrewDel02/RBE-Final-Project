#include <Arduino.h>
#include <Zumo32U4.h>
#include <event_timer.h>
#include "button.h"

#define turn90 500
#define arbitrarySpeed 20
#define time360 200

enum States {IDLE, WALL_FOLLOW, LINE_FOLLOW, TURN_90, DRIVE_STRAIGHT, SPIN};
States state = IDLE;

EventTimer timer;
Button buttonA(14);

void setPidSpeed(int rightSpeed, int leftSpeed);
float readWallDistance();
float* wallSpeed(float distance);
bool lineDetected();
bool irDetected();
int* readLineSensor();
float* lineSpeeds(int lineVals[]);
bool angleToFlat();

void setup() {
  // put your setup code here, to run once:
  buttonA.Init();
}

void loop() {
  switch (state) {
    case IDLE: {
      // set constant speed
      setPidSpeed(0, 0);
      // check transition condition;
      if (buttonA.CheckButtonPress()) timer.Start(1000);
      if (timer.CheckExpired()) {
        state = WALL_FOLLOW;
        timer.Cancel();
      }
    }

    case WALL_FOLLOW: {
      // measure val
      float distance = readWallDistance();
      // PID calculation here
      float *speeds = wallSpeed(distance);
      // set PID speed target
      setPidSpeed(speeds[0], speeds[1]);
      // check transition condition
      if (lineDetected()) {
        state = TURN_90;
        timer.Start(turn90);
      }
    }

    case LINE_FOLLOW: {
      // measure val
      int *lineVals = readLineSensor();
      // PID calculation here
      float *speeds = lineSpeeds(lineVals);
      // set PID speed target
      setPidSpeed(speeds[0], speeds[1]);
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


  }
}
