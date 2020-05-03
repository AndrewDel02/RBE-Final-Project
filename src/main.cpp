#include <Arduino.h>
#include <Zumo32U4.h>

enum States {IDLE, WALL_FOLLOW, LINE_FOLLOW, TURN_90, DRIVE_STRAIGHT, SPIN};
States state = IDLE;
void setup() {
  // put your setup code here, to run once:
}

void loop() {
  // put your main code here, to run repeatedly:
}
