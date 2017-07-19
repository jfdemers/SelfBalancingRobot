#include "robot.h"

#include "PWM.h"              // PWM Frequency Library at https://code.google.com/archive/p/arduino-pwm-frequency-library/downloads
#include "EnableInterrupt.h"  // Enable Interrupt library
#include "digitalWriteFast.h" // DigitalWriteFast Library
#include "Display.h"

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
  Display::clear();
  Display::setPos(2, 0);
  Serial.print("Lay robot down");
  Display::setPos(0, 1);
  Serial.print("and press button");
  pinMode(LED_BUILTIN, OUTPUT);

  waitForButton();

  Display::clear();
  Display::setPos(0,0);
  Serial.print("Voltage:");
}

void Robot::run() {
  if (!voltageLow) {
    Display::setPos(9, 0);
    Serial.print(voltage, 1);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(10);
    digitalWrite(LED_BUILTIN, LOW);
    delay(10);
    checkVoltage();
  }
  else {
    Display::clear();
    Display::setPos(0,0);
    Serial.print("Voltage too low.");
    Display::setPos(0, 1);
    Serial.print(voltage, 4);
  }
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
