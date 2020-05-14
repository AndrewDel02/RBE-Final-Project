#include <Wire.h>
#include <Zumo32U4.h>

class ComplementaryFilter {
  const float K = .5;
  int prevTime = 0;
  int gyroPrevTime = 0; // could I use the same time value for both functions? probably, but I won't
  float prevEstAngle = 0;
  LSM303 compass;
  L3G gyro;
  float prevZ = 0;
public:
  void init();
  bool CalcAngle(float& est);
  bool getGyroZ(float &zVal);
};

// initialize gyro and accelerometer, configuring registers
void ComplementaryFilter::init() {
  Wire.begin();

  if (!compass.init())
  {
    // Failed to detect the compass.
    ledRed(1);
    while(1)
    {
      Serial.println(F("Failed to detect the compass."));
      delay(100);
    }
  }

  compass.enableDefault();

  if (!gyro.init())
  {
    // Failed to detect the gyro.
    ledRed(1);
    while(1)
    {
      Serial.println(F("Failed to detect gyro."));
      delay(100);
    }
  }

  gyro.enableDefault();


  uint8_t ctrl1 = compass.readReg(LSM303::CTRL1); // read register
  uint8_t newCtrl1 = (ctrl1 & 0x0F) | 0x60; // clear first 4 bits, then set them to 6, keep the rest intact
  compass.writeReg(LSM303::CTRL1, newCtrl1); // send to register

  uint8_t ctrl1Gyro = gyro.readReg(L3G::CTRL1); // read register
  uint8_t newCtrl1Gyro = (ctrl1Gyro & 0x0F) | 0xB0; // clear first 4 bits, then set to B
  gyro.writeReg(L3G::CTRL1, newCtrl1Gyro); // send to register
}

// calculate angle using complemetary filter, return true if new reading
bool ComplementaryFilter::CalcAngle(float& est) {
  bool retVal = false;
  uint8_t status = compass.readReg(LSM303::STATUS_A);
  uint8_t newReading = status & 0x80; // clear everything except MSB
  if (newReading) {
    compass.read();
    gyro.read();
    retVal = true;
  }

  int currentTime = millis();
  float deltaT = (currentTime - prevTime) / 100000.0;
  prevTime = currentTime;

  float obsAngle = atan2(-compass.a.x, compass.a.z); // calc obs

  float gammaY = (gyro.g.y-168.7) / 57.3; // converty to rad/s offset bias

  float predAngle = prevEstAngle + (gammaY * deltaT); // calc predicted

  est = (K * predAngle) + ((1-K)*obsAngle) * 57.3; // ComplementaryFilter

  prevEstAngle = est; // store for next iteration

  return retVal;
}

// incomplete, too much noise to be practical
bool ComplementaryFilter::getGyroZ(float &zVal) {
  bool retVal = false;
  uint8_t zStatus = gyro.readReg(L3G::STATUS);
  uint8_t newZStatus = zStatus & 0x04; // clear everything except ZDA bit
  if (newZStatus) {
    gyro.read();
    retVal = true;
  }

  int currentTime = millis();
  float deltaT = (currentTime - prevTime) / 1000.0;
  gyroPrevTime = currentTime;

  float gammaZ = gyro.g.z+246.22;

  zVal = prevZ + (gammaZ * deltaT);
  prevZ = zVal;

  return retVal;
}
