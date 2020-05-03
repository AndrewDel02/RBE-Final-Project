#include <Arduino.h>

class Button {
  int pinNum;
  int timerStart;
  enum buttonState {STABLE, UNSTABLE} currentState;
  int buttonVal;
  int lastButtonVal = HIGH;

public:
  Button(uint8_t button_pin) {
    pinNum = button_pin;
    currentState = STABLE;
  }

  void Init() {
    pinMode(pinNum, INPUT_PULLUP);
  }

  bool CheckButtonPress() {
    buttonVal = digitalRead(pinNum);
    switch (currentState) {
      case STABLE:
        if (buttonVal != lastButtonVal) {
          timerStart = millis();
          lastButtonVal = buttonVal;
          currentState = UNSTABLE;
          return false;
        } else {
          return false;
        }; break;

      case UNSTABLE:
        if (buttonVal != lastButtonVal) {
          timerStart = millis();
          lastButtonVal = buttonVal;
          return false;
        } else if ((millis() - timerStart >= 10) && buttonVal == HIGH) {
          currentState = STABLE;
          return true;
        } else {
          return false;
        } break;
    }
  }

};
