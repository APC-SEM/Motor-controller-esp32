/*
Monash Shell Eco Marathon team
- this is developed for SEM Asia 2024

Motor controller connection
(BIG 48V 1kW motor)

channel A - U - red (throttle pins)
channel B - W - black
channel C - V - yellow (cloest to hall sensors)

Hall 1  - white wire
Hall 2  - green wire
Hall 3  - blue wire

Written by AndrewJNg
*/

// user settings
#define debug true

/////////////////////////////////////////////////////////////////////////////////////
// Hall effect & User inputs pins

#define HALL_1_PIN 36  //white
#define HALL_2_PIN 39  //green
#define HALL_3_PIN 34  //blue
#define EN_PIN 35
#define THROTTLE_PIN 32

/////////////////////////////////////////////////////////////////////////////////////
// Mosfetr driver pinout
#define A_IN 25
#define A_SD 26
#define B_IN 27
#define B_SD 14
#define C_IN 12
#define C_SD 13

// Built in LED pin
#define LED_PIN 2
#include "motor.h"

/////////////////////////////////////////////////////////////////////////////////////
// Throttle
#include "lowPass.h"
LowPass<1> lpTop(5, 10, true);

int max_throttle = 0;
int min_throttle = 4096;
#include "memory.h"


// global variables
int stepstate = 0;
int throt_pwm = 0;

unsigned long previousMillis = 0;
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void setup() {
  // Setup serial terminal for debugging
  if (debug) Serial.begin(115200);
  motor_setup();

  /////////////////////////////////////////////////////////////////////////////////////
  // Read input from hall effect sensors and throttle
  pinMode(HALL_1_PIN, INPUT);
  pinMode(HALL_2_PIN, INPUT);
  pinMode(HALL_3_PIN, INPUT);

  pinMode(THROTTLE_PIN, INPUT);
  pinMode(EN_PIN, INPUT);

  /////////////////////////////////////////////////////////////////////////////////////
  // Calibration mode

  int throttle = analogRead(THROTTLE_PIN);
  if (throttle > (4096 / 2)) {
    previousMillis = millis();
    int throttle_percent = 0;
    unsigned long calib_time = millis() - previousMillis;

    // enter calibration mode
    if (debug) Serial.println("entered calibration mode");
    do {
      throttle = analogRead(THROTTLE_PIN);
      if (throttle > max_throttle) max_throttle = throttle;
      else if (throttle < min_throttle) min_throttle = throttle;

      throttle_percent = map(throttle, min_throttle, max_throttle, 0, 100);
      calib_time = millis() - previousMillis;

      if (debug) {
        Serial.print("current values  ");

        Serial.print("  min: ");
        Serial.print(min_throttle);
        Serial.print("  max: ");
        Serial.print(max_throttle);

        Serial.print("  percent: ");
        Serial.print(throttle_percent);
        Serial.print("  time: ");
        Serial.print(calib_time);
        Serial.println();
      }
    } while ((throttle_percent < 5) || (calib_time < 5000));  //exit when throttle exceeds 5% and 5 second has passed
    store_memory();
  }

  /////////////////////////////////////////////////////////////////////////////////////
  // Normal mode
  read_memory();
  if (debug) {
    Serial.println("entered normal mode");
    Serial.print("min: ");
    Serial.println(min_throttle);
    Serial.print("max: ");
    Serial.println(max_throttle);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void loop() {
  int hallState1 = digitalRead(HALL_1_PIN);
  int hallState2 = digitalRead(HALL_2_PIN);
  int hallState3 = digitalRead(HALL_3_PIN);

  int enable = digitalRead(EN_PIN);

  // sample throttle at 10Hz, to increase speed for hall sensors reading
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= 100) {
    previousMillis = currentMillis;
    throt_pwm = readThrottle();
  }

  /////////////////////////////////////////////////////////////////////////////////////
  // step sequencer using hall effect inputs
  if (hallState1 == 0 && hallState2 == 0 && hallState3 == 1) stepstate = 0;
  else if (hallState1 == 1 && hallState2 == 0 && hallState3 == 1) stepstate = 1;
  else if (hallState1 == 1 && hallState2 == 0 && hallState3 == 0) stepstate = 2;
  else if (hallState1 == 1 && hallState2 == 1 && hallState3 == 0) stepstate = 3;
  else if (hallState1 == 0 && hallState2 == 1 && hallState3 == 0) stepstate = 4;
  else if (hallState1 == 0 && hallState2 == 1 && hallState3 == 1) stepstate = 5;
  else {
    stepstate = 7;
    if (debug) {
      Serial.print("unstable step");

      Serial.print("    ");
      Serial.print("curr step= ");
      Serial.print(stepstate);
      Serial.print("  h1:");
      Serial.print(hallState1);
      Serial.print("  h2:");
      Serial.print(hallState2);
      Serial.print("  h3:");
      Serial.print(hallState3);
      Serial.println();
    }
  }
  /*
  // delay(1);
  delayMicroseconds(50);
  stepstate++;
  stepstate = stepstate % 6;
  */

  if (enable) {
    // write the output to the writePWM function, which changes mosfet driver outputs accordingly
    writePWM(stepstate, throt_pwm);

    if (debug) {
      Serial.print(analogRead(THROTTLE_PIN));
      Serial.print("  ");
      Serial.print(throt_pwm);
      Serial.print("  step= ");
      Serial.print(stepstate);
      Serial.print("  en= ");
      Serial.print(enable);
      Serial.print("  h1:");
      Serial.print(hallState1);
      Serial.print("  h2:");
      Serial.print(hallState2);
      Serial.print("  h3:");
      Serial.print(hallState3);
      Serial.println();
    }
  }

  else {
    writePWM(7, 0);
    if (debug) Serial.println("disabled");
  }
}

/* Read the throttle value from the ADC. Because our ADC can read from 0v-3.3v, but the throttle doesn't output over this whole range,
 * scale the throttle reading to take up the full range of 0-255
 */
#define max_pwm 200  // set highest pwm allowed, too high would cause bootstrap capacitors to not charge in time
uint8_t readThrottle() {

  int32_t adc = analogRead(THROTTLE_PIN);  // Note, analogRead can be slow!
  adc = lpTop.filt(adc);                   // low pass filter on throttle input

  // Scale the output between 0 and 120, out of 0-255 (to limit the max speed of the car)
  if (adc < min_throttle) adc = 0;
  else if (adc > max_throttle) adc = max_pwm;
  else adc = map(adc, min_throttle, max_throttle, 0, max_pwm);

  return adc;
}
