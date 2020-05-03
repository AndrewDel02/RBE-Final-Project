#include <Arduino.h>
#include <event_timer.h>

void EventTimer::Start(int ms) {
  duration = ms;
  start_time = millis();
  isRunning = true;
}

bool EventTimer::CheckExpired() {
  return (millis() - start_time >= duration) && isRunning;
}

void EventTimer::Cancel() {
  isRunning = false;
}
