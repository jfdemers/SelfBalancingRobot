#ifndef _ROBOT_H
#define _ROBOT_H

#include "config.h"

#include "motors.h"
#include "mpu.h"

class Robot {
public:
  Robot();

  void setup();
  void run();

private:
  void initializeMPU();
  void checkVoltage();

  void waitForButton();
  void standUp();

  float voltage;
  bool voltageLow;

  float PitchEst;
  float BiasEst;

  Motors motors;
  MPU mpu;
};

#endif
