/* This example reads the raw values from the L3GD20H gyro and
LSM303D accelerometer and magnetometer on the Zumo 32U4, and
prints those raw values to the serial monitor.

The accelerometer readings can be converted to units of g using
the conversion factors specified in the "Sensor characteristics"
table in the LSM303 datasheet.  The default full-scale (FS)
setting is +/- 2 g, so the conversion factor is 0.61 mg/LSB
(least-significant bit).  A raw reading of 16384 would correspond
to 1 g.

The gyro readings can be converted to degrees per second (dps)
using the "Mechanical characteristics" table in the L3GD20H
datasheet.  The default sensitivity is 8.75 mdps/LSB
(least-significant bit).  A raw reading of 10285 would correspond
to 90 dps.

The magnetometer readings are more difficult to interpret and
will usually require calibration. */

#include <Wire.h>
#include <Zumo32U4.h>

class ComplementaryFilter {
  const float K = .5;
  int prevTime = 0;
  float prevEstAngle = 0;
  LSM303 compass;
  L3G gyro;
public:
  void init();
  bool CalcAngle(float& est);
};



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
