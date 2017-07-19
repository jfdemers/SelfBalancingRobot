#include "Arduino.h"

#include "robot.h"

Robot robot;

void setup() {
  robot.setup();
}

void loop() {
  robot.run();
}
