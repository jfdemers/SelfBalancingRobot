#include "motors.h"

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
