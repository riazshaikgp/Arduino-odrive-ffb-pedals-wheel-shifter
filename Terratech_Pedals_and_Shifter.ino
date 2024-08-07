#include <Joystick.h>
#include "DigitalWriteFast.h"
#include <QuadratureEncoder.h>
#include <HX711_ADC.h>

/*
Joystick_(
		uint8_t hidReportId = JOYSTICK_DEFAULT_REPORT_ID,
		uint8_t joystickType = JOYSTICK_TYPE_JOYSTICK,
        uint8_t buttonCount = JOYSTICK_DEFAULT_BUTTON_COUNT,
		uint8_t hatSwitchCount = JOYSTICK_DEFAULT_HATSWITCH_COUNT,
		bool includeXAxis = true,
		bool includeYAxis = true,
		bool includeZAxis = true,
		bool includeRxAxis = true,
		bool includeRyAxis = true,
		bool includeRzAxis = true,
		bool includeRudder = true,
		bool includeThrottle = true,
		bool includeAccelerator = true,
		bool includeBrake = true,
		bool includeSteering = true);
*/


#define ACCEL_PIN A0
#define CLUTCH_PIN A1
#define encoderPinA 2
#define encoderPinB 3
#define hx711D 4
#define hx711C 5
#define throttlePin A0
#define clutchPin A1
#define encoderPPR 20480
#define wheelmaxturn 450
#define wheelrange (encoderPPR / 360) * wheelmaxturn

#define motorPinA 7
#define motorPinB 8
#define motorPinPWM 9

#define WHEEL_MAX_ANGLE 450
#define WHEEL_MIN_ANGLE -450

#define MAX_PWM 200

#define buttons 18
#define hatswitch 0

Joystick_ Joystick(JOYSTICK_DEFAULT_REPORT_ID, JOYSTICK_TYPE_MULTI_AXIS,
                   buttons, hatswitch,
                   true /*x steering*/, true /*y brake*/, true /*z hshifter to implement*/, true /*rx throttle*/, true /*ry clutch*/, true /*rz ffb to implement*/,
                   false /*rudder*/, false /*throttle*/, false /*accelerator*/, false /*brake*/, false /*steering*/);

Encoders wheelEnc(encoderPinA, encoderPinB);

HX711_ADC brake(hx711D, hx711C);
const unsigned long gcCycleDelta = 1000;
const unsigned long gcButtonDelta = 500;
const unsigned long gcAnalogDelta = 25;
unsigned long gNextTime = 0;
unsigned int gCurrentStep = 0;

bool isOutOfRange = false;
int32_t forces[2] = { 0 };
Gains gains[2];
EffectParams effectparams[2];

volatile long value = 0;
int32_t g_force = 0;

int32_t currentPosition = 0;
volatile int8_t oldState = 0;
const int8_t KNOBDIR[] = {
  0, 1, -1, 0,
  -1, 0, 0, 1,
  1, 0, 0, -1,
  0, -1, 1, 0
};

void tick(void) {
  int sig1 = digitalReadFast(encoderPinA);
  int sig2 = digitalReadFast(encoderPinB);
  int8_t thisState = sig1 | (sig2 << 1);

  if (oldState != thisState) {
    currentPosition += KNOBDIR[thisState | (oldState << 2)];
    oldState = thisState;
  }
}

void setup() {
  Joystick.setXAxisRange(WHEEL_MIN_ANGLE, WHEEL_MAX_ANGLE);
  Joystick.setXAxis(0);

  Joystick.setYAxisRange(-32767, 32767);
  Joystick.setYAxis(0);

  Joystick.setRxAxisRange(-32767, 32767);
  Joystick.setRxAxis(-32767);

  Joystick.setRyAxisRange(-32767, 32767);
  Joystick.setBrake(-32767);

  Joystick.begin();
  brake.begin();
  long stabilisingtime = 2000;  // tare preciscion can be improved by adding a few seconds of stabilising time
  brake.start(stabilisingtime);
  brake.setCalFactor(696.0);
  brake.tare();

  cli();
  TCCR3A = 0;  //set TCCR1A 0
  TCCR3B = 0;  //set TCCR1B 0
  TCNT3 = 0;   //counter init
  OCR3A = 399;
  TCCR3B |= (1 << WGM32);  //open CTC mode
  TCCR3B |= (1 << CS31);   //set CS11 1(8-fold Prescaler)
  TIMSK3 |= (1 << OCIE3A);
  sei();
}

ISR(TIMER3_COMPA_vect) {
  Joystick.getUSBPID();
}

void loop() {

  float value = (float)wheelEnc.getEncoderCount()/(float)encoderPPR*360.0;

  if (value > WHEEL_MAX_ANGLE) {
    isOutOfRange = true;
    value = WHEEL_MAX_ANGLE;
  } else if (value < WHEEL_MIN_ANGLE) {
    isOutOfRange = true;
    value = WHEEL_MIN_ANGLE;
  } else {
    isOutOfRange = false;
  }

  Joystick.setXAxis(value);
  Joystick.setYAxis(getBrakingForce());
  Joystick.setRxAxis(getThrottleValue());
  Joystick.setRyAxis(getClutchValue());
  //Joystick.sendState();

  effectparams[0].springMaxPosition = WHEEL_MAX_ANGLE;
  effectparams[0].springPosition = value;
  effectparams[1].springMaxPosition = 255;
  effectparams[1].springPosition = 0;
  Joystick.setEffectParams(effectparams);
  Joystick.getForce(forces);

  if (!isOutOfRange) {
    if (forces[0] > 0) {
      digitalWrite(motorPinA, HIGH);
      digitalWrite(motorPinB, LOW);
      analogWrite(motorPinPWM, abs(forces[0]));
    } else {
      digitalWrite(motorPinA, LOW);
      digitalWrite(motorPinB, HIGH);
      analogWrite(motorPinPWM, abs(forces[0]));
    }
  } else {
    if (value < 0) {
      digitalWrite(motorPinA, LOW);
      digitalWrite(motorPinB, HIGH);
    } else {
      digitalWrite(motorPinA, HIGH);
      digitalWrite(motorPinB, LOW);
    }
    analogWrite(motorPinPWM, MAX_PWM);
  }
}



int16_t getBrakingForce() {
  brake.update();
  int16_t force = (int16_t)brake.getData();
  return force;
}

int16_t getThrottleValue() {
  int16_t val = analogRead(throttlePin);
  return val;
}

int16_t getClutchValue() {
  int16_t val = analogRead(clutchPin);
  return val;
}