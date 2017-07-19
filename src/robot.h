#ifndef _ROBOT_H
#define _ROBOT_H

#include "config.h"

#include "motors.h"

class Robot {
public:
  Robot();

  void setup();
  void run();

private:
  void initializeMPU();
  void checkVoltage();

  void waitForButton();

  float voltage;
  bool voltageLow;

  Motors motors;
};

#endif
