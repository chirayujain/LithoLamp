#include <Wire.h>
#include "i2cEncoderMiniLib.h"
#include <AccelStepper.h>
#include <math.h>

#define ENCODER_ADDRESS 0x20  // Default I2C address of the encoder
#define INTERRUPT_PIN 3       // Interrupt pin connected to the encoder

#define STEPPER_PIN_1 22
#define STEPPER_PIN_2 24
#define STEPPER_PIN_3 26
#define STEPPER_PIN_4 28
int step_number = 0;

// Define stepper motor connections
// #define IN1 22
// #define IN2 24
// #define IN3 26
// #define IN4 28
// #define HALFSTEP 8
// #define STEPS_PER_REV 2048  // Steps per revolution for 28BYJ-48

i2cEncoderMiniLib encoder(ENCODER_ADDRESS);
// AccelStepper stepper(HALFSTEP, IN1, IN3, IN2, IN4);

volatile bool motorRunning = false;
volatile int motorSpeed = 5;  // Initial speed


// LED PWM settings
const int pwmPin = 18;      // GPIO18
const int pwmFreq = 25000;  // 25 kHz
const int pwmResolution = 16;
volatile int dutyCycle = 0;


void setup() {
  pinMode(INTERRUPT_PIN, INPUT_PULLUP);

  // LED PWM setup
  pinMode(pwmPin, OUTPUT);
  analogWriteFreq(pwmFreq);
  analogWriteResolution(pwmResolution);

  Wire.setSDA(4);
  Wire.setSCL(5);
  Wire.begin();
  Serial.begin(9600);

  encoder.begin(
    i2cEncoderMiniLib::WRAP_DISABLE | i2cEncoderMiniLib::DIRE_LEFT | i2cEncoderMiniLib::IPUP_ENABLE | i2cEncoderMiniLib::RMOD_X1);

  encoder.writeCounter(0);  // Start at 0
  encoder.writeMax(10);     // Set maximum value to 20
  encoder.writeMin(0);      // Set minimum value to 0
  encoder.writeStep(1);     // Set step size to 1

  // Set up callbacks
  encoder.onChange = onChange;
  encoder.onMax = onMaxValue;
  encoder.onMin = onMinValue;
  encoder.onButtonLongPush = onButtonLongPush;

  // Configure double push period (optional)
  encoder.writeDoublePushPeriod(50);  // 50 * 10ms = 500ms between pushes to consider it a double push

  encoder.autoconfigInterrupt();  // Enable interrupts

  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), updateEncoder, FALLING);
}

void loop() {
  analogWrite(pwmPin, dutyCycle);
}






void setup1() {
  // Stepper motor setup
  //   stepper.setMaxSpeed(100);
  //   stepper.setAcceleration(500);
  //   stepper.setSpeed(motorSpeed);
  pinMode(STEPPER_PIN_1, OUTPUT);
  pinMode(STEPPER_PIN_2, OUTPUT);
  pinMode(STEPPER_PIN_3, OUTPUT);
  pinMode(STEPPER_PIN_4, OUTPUT);
}


void loop1() {
  if (motorRunning) {
    OneStep(false);
    delay(50);
    digitalWrite(STEPPER_PIN_1, LOW);
    digitalWrite(STEPPER_PIN_2, LOW);
    digitalWrite(STEPPER_PIN_3, LOW);
    digitalWrite(STEPPER_PIN_4, LOW);
    delay(1000);
  }
}


void OneStep(bool dir) {
  if (dir) {
    switch (step_number) {
      case 0:
        digitalWrite(STEPPER_PIN_1, HIGH);
        digitalWrite(STEPPER_PIN_2, LOW);
        digitalWrite(STEPPER_PIN_3, LOW);
        digitalWrite(STEPPER_PIN_4, LOW);
        break;
      case 1:
        digitalWrite(STEPPER_PIN_1, LOW);
        digitalWrite(STEPPER_PIN_2, HIGH);
        digitalWrite(STEPPER_PIN_3, LOW);
        digitalWrite(STEPPER_PIN_4, LOW);
        break;
      case 2:
        digitalWrite(STEPPER_PIN_1, LOW);
        digitalWrite(STEPPER_PIN_2, LOW);
        digitalWrite(STEPPER_PIN_3, HIGH);
        digitalWrite(STEPPER_PIN_4, LOW);
        break;
      case 3:
        digitalWrite(STEPPER_PIN_1, LOW);
        digitalWrite(STEPPER_PIN_2, LOW);
        digitalWrite(STEPPER_PIN_3, LOW);
        digitalWrite(STEPPER_PIN_4, HIGH);
        break;
    }
  } else {
    switch (step_number) {
      case 0:
        digitalWrite(STEPPER_PIN_1, LOW);
        digitalWrite(STEPPER_PIN_2, LOW);
        digitalWrite(STEPPER_PIN_3, LOW);
        digitalWrite(STEPPER_PIN_4, HIGH);
        break;
      case 1:
        digitalWrite(STEPPER_PIN_1, LOW);
        digitalWrite(STEPPER_PIN_2, LOW);
        digitalWrite(STEPPER_PIN_3, HIGH);
        digitalWrite(STEPPER_PIN_4, LOW);
        break;
      case 2:
        digitalWrite(STEPPER_PIN_1, LOW);
        digitalWrite(STEPPER_PIN_2, HIGH);
        digitalWrite(STEPPER_PIN_3, LOW);
        digitalWrite(STEPPER_PIN_4, LOW);
        break;
      case 3:
        digitalWrite(STEPPER_PIN_1, HIGH);
        digitalWrite(STEPPER_PIN_2, LOW);
        digitalWrite(STEPPER_PIN_3, LOW);
        digitalWrite(STEPPER_PIN_4, LOW);
    }
  }
  step_number++;
  if (step_number > 3) {
    step_number = 0;
  }
}

void updateEncoder() {
  encoder.updateStatus();
}

void onChange(i2cEncoderMiniLib* enc) {
  int value = enc->readCounterLong();
  Serial.print("Value: ");
  Serial.println(value);

  dutyCycle = map(value, 0, 50, 0, 65535);  // Assuming encoder range 0-100

  if(value == 0){
    motorRunning = false;
  }
  if(value > 0){
    motorRunning = true;
  }

}

void onMaxValue(i2cEncoderMiniLib* enc) {
  Serial.println("Maximum value reached!");
}

void onMinValue(i2cEncoderMiniLib* enc) {
  Serial.println("Minimum value reached!");
}

void onButtonLongPush(i2cEncoderMiniLib* enc) {
  Serial.println("Button long pressed!");

  // motorRunning = !motorRunning;
  // if (motorRunning) {
  //   Serial.println("Motor started!");
  // } else {
  //   Serial.println("Motor stopped!");
  // }
  dutyCycle = 0;
  motorRunning = false;
}
