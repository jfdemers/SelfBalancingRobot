#include "motors.h"

#include "Wire.h"
#include "PWM.h"              // PWM Frequency Library at https://code.google.com/archive/p/arduino-pwm-frequency-library/downloads
#include "EnableInterrupt.h"  // Enable Interrupt library
#include "digitalWriteFast.h" // DigitalWriteFast Library

#define PWM_FREQUENCY   10000

Motors::Motors() {
  powerFactor = 1.0;

  // Setup the motor power pins
  pinMode(LeftMotorIn1, OUTPUT);
  pinMode(LeftMotorIn2, OUTPUT);
  pinMode(RightMotorIn1, OUTPUT);
  pinMode(RightMotorIn2, OUTPUT);
  pinMode(StandbyPin, OUTPUT);

  digitalWriteFast(LeftMotorIn1, LOW);
  digitalWriteFast(LeftMotorIn2, LOW);
  digitalWriteFast(RightMotorIn1, LOW);
  digitalWriteFast(RightMotorIn2, LOW);
  digitalWriteFast(StandbyPin, HIGH);

  // Setup the encoders pins
  pinMode(LeftEncoderPinA, INPUT);
  pinMode(LeftEncoderPinB, INPUT);
  pinMode(RightEncoderPinA, INPUT);
  pinMode(RightEncoderPinB, INPUT);

  // Active pullups resistors for encoder pins.
  digitalWrite(LeftEncoderPinA, HIGH);
  digitalWrite(LeftEncoderPinB, HIGH);
  digitalWrite(RightEncoderPinA, HIGH);
  digitalWrite(RightEncoderPinB, HIGH);

  // Setup the motors PWM pins
  InitTimersSafe();
  SetPinFrequency(LeftMotorPWM, PWM_FREQUENCY);
  SetPinFrequency(RightMotorPWM,  PWM_FREQUENCY);
}

void Motors::setPowerFactor(float value) {
  powerFactor = value;
}

void Motors::setPower(float p1, float p2) {
  if (p1 < 0) {
    digitalWriteFast(LeftMotorIn1, LOW);
    digitalWriteFast(LeftMotorIn2, HIGH);
  }
  else {
    digitalWriteFast(LeftMotorIn1, HIGH);
    digitalWriteFast(LeftMotorIn2, LOW);
  }

  if (p2 < 0) {
    digitalWriteFast(RightMotorIn1, LOW);
    digitalWriteFast(RightMotorIn2, HIGH);
  }
  else {
    digitalWriteFast(RightMotorIn1, HIGH);
    digitalWriteFast(RightMotorIn2, LOW);
  }

  uint8_t realP1 = max(abs(p1 * powerFactor) * 255, 255);
  uint8_t realP2 = max(abs(p2 * powerFactor) * 255, 255);
  pwmWrite(LeftMotorPWM, realP1);
  pwmWrite(RightMotorPWM, realP2);
}

void Motors::standby() {
  digitalWriteFast(StandbyPin, LOW);
}

void Motors::resume() {
  digitalWriteFast(StandbyPin, HIGH);
}

void Motors::stop() {
  digitalWriteFast(LeftMotorIn1, LOW);
  digitalWriteFast(LeftMotorIn2, LOW);
  digitalWriteFast(RightMotorIn1, LOW);
  digitalWriteFast(RightMotorIn2, LOW);
  pwmWrite(LeftMotorPWM, 0);
  pwmWrite(RightMotorPWM, 0);
}
