#ifndef _MOTORS_H
#define _MOTORS_H

#include "config.h"

class Motors {
public:
  Motors();

  void initialize();
  void setPowerFactor(float value);
  void setPower(float p1, float p2);
  void stop();
  void standby();
  void resume();

  inline float getVelocity() const { return velocity; }
  inline float getRotation() const { return rotation; }

  // Must be called often. It will update velocity and rotation
  // 10 times per second.
  void update();

private:
  float powerFactor;
  int topValue;

  float velocity;
  float wheelPosition;
  float rotation;

  long leftEncoderTicks;
  long rightEncoderTicks;

  long lastTime;
};

#endif
