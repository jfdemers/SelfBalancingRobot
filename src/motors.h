#ifndef _MOTORS_H
#define _MOTORS_H

#include "config.h"

class Motors {
public:
  Motors();

  void setPowerFactor(float value);
  void setPower(float p1, float p2);
  void stop();
  void standby();
  void resume();

private:
  float powerFactor;
};

#endif
