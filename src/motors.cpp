#include "motors.h"

#include "Wire.h"
#include "PWM.h"              // PWM Frequency Library at https://code.google.com/archive/p/arduino-pwm-frequency-library/downloads
#include "EnableInterrupt.h"  // Enable Interrupt library
#include "digitalWriteFast.h" // DigitalWriteFast Library
#include "display.h"

#define PWM_FREQUENCY   10000

//#define LEFT_ENCODER_FLAG   0x01
//#define RIGHT_ENCODER_FLAG  0x02

static volatile long LeftEncoderTicks = 0;
static volatile long RightEncoderTicks = 0;
//static volatile uint8_t flags = 0;

static volatile uint8_t LeftEncoderBValue = 0;
static volatile uint8_t RightEncoderBValue = 0;

static void handleLeftEncoderInt() {
  // Test transition; since the interrupt will only fire on 'rising' we don't need to read pin A
  LeftEncoderBValue = digitalReadFast(LeftEncoderPinB);   // read the input pin

  // and adjust counter + if A leads B
  LeftEncoderTicks += LeftEncoderBValue ? -1 : +1;
  //flags |= LEFT_ENCODER_FLAG;
}

static void handleRightEncoderInt() {
  RightEncoderBValue = digitalReadFast(RightEncoderPinB);

  RightEncoderTicks += RightEncoderBValue ? -1 : +1;
  //flags |= RIGHT_ENCODER_FLAG;
}

Motors::Motors() {
  powerFactor = 1.0;
}

void Motors::initialize() {
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

  // Enable interrupts for the encoders
  enableInterrupt(LeftEncoderPinA, handleLeftEncoderInt, RISING);
  enableInterrupt(RightEncoderPinA, handleRightEncoderInt, RISING);

  lastTime = millis();
}

void Motors::setPowerFactor(float value) {
  powerFactor = value;
}

void Motors::setPower(float p1, float p2) {
  if (p1 < 0.0f) {
    digitalWriteFast(LeftMotorIn1, LOW);
    digitalWriteFast(LeftMotorIn2, HIGH);
  }
  else {
    digitalWriteFast(LeftMotorIn1, HIGH);
    digitalWriteFast(LeftMotorIn2, LOW);
  }

  if (p2 < 0.0f) {
    digitalWriteFast(RightMotorIn1, LOW);
    digitalWriteFast(RightMotorIn2, HIGH);
  }
  else {
    digitalWriteFast(RightMotorIn1, HIGH);
    digitalWriteFast(RightMotorIn2, LOW);
  }

  unsigned int realP1 = 65535 * fmin(fabs(p1 * powerFactor), 1.0f);
  unsigned int realP2 = 65535 * fmin(fabs(p2 * powerFactor), 1.0f);

  /*Display::setPos(0, 0);
  Serial.print(p1); Serial.print("; "); Serial.print(p2);
  Display::setPos(0, 1);
  Serial.print(realP1); Serial.print("; "); Serial.print(realP2);*/

  pwmWriteHR(LeftMotorPWM, realP1);
  pwmWriteHR(RightMotorPWM, realP2);
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
  //pwmWrite(LeftMotorPWM, 0);
  //pwmWrite(RightMotorPWM, 0);
}

void Motors::update() {
  long currentTime = millis();

  if (currentTime - lastTime >= 100) {
    lastTime = currentTime;

    // Disable interrupts while we get the values of the encoder.
    noInterrupts();

    leftEncoderTicks = LeftEncoderTicks;
    rightEncoderTicks = RightEncoderTicks;

    interrupts();
  }
}
