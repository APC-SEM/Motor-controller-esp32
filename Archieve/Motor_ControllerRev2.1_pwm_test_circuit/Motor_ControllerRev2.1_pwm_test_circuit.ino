
#include "lowPass.h"
LowPass<1> lpTop(5, 1e3, true);

#define THROTTLE_PIN 32   // Throttle pin
#define THROTTLE_LOW 900  // These LOW and HIGH values are used to scale the ADC reading. More on this below
#define THROTTLE_HIGH 3095

#define AH_PIN 25  // Pins from the Teensy to the gate drivers. AH = A high, etc
#define AL_PIN 26
#define BH_PIN 27
#define BL_PIN 14
#define CH_PIN 12
#define CL_PIN 13

#define LED_PIN 2  // Built in LED


void setup() {
  // set PWM to drive state pin
  const int freq = 10000;
  // const int freq = 8000;
  const int resolution = 8;

  ledcSetup(2, freq, resolution);
  ledcSetup(3, freq, resolution);
  ledcSetup(4, freq, resolution);

  ledcAttachPin(AH_PIN, 2);
  ledcAttachPin(BH_PIN, 3);
  ledcAttachPin(CH_PIN, 4);

  pinMode(AL_PIN, OUTPUT);
  pinMode(BL_PIN, OUTPUT);
  pinMode(CL_PIN, OUTPUT);

  // pinMode(LED_PIN, OUTPUT);
  ledcSetup(5, freq, resolution);
  ledcAttachPin(LED_PIN, 5);
}

void loop() {
  int pwm = 100;//readThrottle();
  int ah = pwm;
  int bh = pwm;
  int ch = pwm;
  ledcWrite(2, ah);         //pwm on enable pin
  digitalWrite(AL_PIN, 1);  // set high state

  ledcWrite(3, ah);         //pwm on enable pin
  digitalWrite(BL_PIN, 1);  // set high state

  ledcWrite(4, ah);         //pwm on enable pin
  digitalWrite(CL_PIN, 1);  // set high state

  ledcWrite(5, ah);  //pwm on enable pin



  
}

uint8_t readThrottle() {
  int32_t adc = analogRead(THROTTLE_PIN);  // Note, analogRead can be slow!

  adc = lpTop.filt(adc);  // low pass filter on throttle input

  // Scale the output between 0 and 120, out of 0-255 (to limit the max speed of the car)
  adc = map(adc, THROTTLE_LOW, THROTTLE_HIGH, 0, 200);
  return adc;
}
