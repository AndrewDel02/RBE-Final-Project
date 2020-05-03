#include <Arduino.h>

class EventTimer {
public:
  uint32_t duration, start_time;
  bool isRunning;
  void Start(int ms);
  bool CheckExpired();
  void Cancel();
};
