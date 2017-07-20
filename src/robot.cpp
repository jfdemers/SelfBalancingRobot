#include "robot.h"

#include "display.h"
#include "motors.h"
#include "mpu.h"

#define DISPLAY_SPEED

float power = 0;
float increment = 0.01;

Robot::Robot() {
  // Set an initial value of 10V. It will change rapidly once we
  // start taking measurement.
  voltage = 11;
  voltageLow = false;
}

void Robot::setup() {
  Serial.begin(SerialSpeed);
  // Wait before sending commands to the display.
  delay(100);
  Display::setDisplay(Display::no_cursor);

  motors.initialize();
  motors.stop();
  motors.standby();

  mpu.initialize();

  Display::clear();
  Display::setPos(2, 0);
  Serial.print("Lay robot down");
  Display::setPos(0, 1);
  Serial.print("and press button");
  pinMode(LED_BUILTIN, OUTPUT);

  waitForButton();

  while (mpu.calibrateYGyro()) {

  }

  standUp();

  motors.resume();

  Display::clear();
  Display::setPos(0,0);
  Serial.print("Voltage:");
}

void Robot::run() {
  if (!voltageLow) {
    Display::setPos(9, 0);
    Serial.print(voltage, 1);
    checkVoltage();
    motors.update();

    /*if (power > 1.0f && increment > 0.0f) {
      increment = -0.01;
    }

    if (power < -1.0f && increment < 0.0f) {
      increment = 0.01;
    }*/

    motors.setPower(0.5f, 0.5f);

    #ifdef DISPLAY_SPEED
    Display::setPos(0, 1);
    Serial.print(motors.getVelocity(), 2);
    Display::setPos(8, 1);
    Serial.print(motors.getRotation());
    #endif

    power += increment;
  }
  else {
    Display::clear();
    Display::setPos(0,0);
    Serial.print("Voltage too low.");
    Display::setPos(0, 1);
    Serial.print("Charge batteries");
    motors.standby();
  }

  delay(100);
}

void Robot::checkVoltage() {
  // Filter the voltage value in case we have a bad reading. If the value
  // is ok, it will soon be reflected in the voltage value.
  float currentValue = analogRead(VoltSensorPin) * voltageUnit;
  voltage = (voltage * 49 + currentValue) / 50;

  if (voltage < 10) {
    voltageLow = true;
  }
}

void Robot::waitForButton() {
  // This is not the most efficient way to read the value of the pin, we could
  // just take the digital value, but it's not recommended to take digital
  // readings when we use the analog function on the chip because it can cause
  // invalid readings.
  while (analogRead(PushButtonPin) < 800) {
  }
}

void Robot::standUp() {
  Display::clear();
  Serial.print("Stand & Push Btn");

  int button_pushed = 0;
  unsigned long timer;
  while (!button_pushed) {  // Calculate and display pitch until button pushed
    timer = millis();
    PitchEst = 0;
    for (int i = 0; i < 50; i++) {  // Average 50 samples
      mpu.readIMUData();
      PitchEst += mpu.getAccAngle();
      if (analogRead(PushButtonPin) > 800) {
        button_pushed = 1;
      }
    }
    PitchEst /= 50.0f;

    Display::setPos(0, 0);
    Serial.print("Pitch: ");
    Serial.print(PitchEst);
    while (millis() < timer + 250) {
      if (analogRead(PushButtonPin) > 800) {
        button_pushed = 1;
      }
    }
  }
  BiasEst = 0;
}
