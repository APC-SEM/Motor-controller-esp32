#define THROTTLE_PIN 32   // Throttle pin
#define THROTTLE_LOW 900  // These LOW and HIGH values are used to scale the ADC reading. More on this below
#define THROTTLE_HIGH 3095
/*
#define THROTTLE_PIN 34   // Throttle pin
#define THROTTLE_LOW 230  // These LOW and HIGH values are used to scale the ADC reading. More on this below
#define THROTTLE_HIGH 380
*/
#define HALL_1_PIN 36  //white
#define HALL_2_PIN 39  //green
#define HALL_3_PIN 34  //blue
#define EN_PIN 35


#include "lowPass.h"

// LowPass<1> lpTop(0.1, 1e3, true);
LowPass<1> lpTop(5, 1e3, true);

// channel A - U - red
// channel B - W - black
// channel C - V - yellow

#define AH_PIN 25  // Pins from the Teensy to the gate drivers. AH = A high, etc
#define AL_PIN 26
#define BH_PIN 27
#define BL_PIN 14
#define CH_PIN 12
#define CL_PIN 13

#define LED_PIN 2  // Built in LED

#define HALL_OVERSAMPLE 4  // Hall oversampling count. More on this in the getHalls() function

uint8_t hallToMotor[8] = { 255, 255, 255, 255, 255, 255, 255, 255 };

int stepstate = 0;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void setup() {  // The setup function is called ONCE on boot-up

  // Setup serial terminal for debugging
  Serial.begin(115200);

  // set led status pin

  /////////////////////////////////////////////////////////////////////////////////////
  // Mosfet driver setup

  // set PWM to drive state pin
  const int freq = 20000;
  // const int freq = 1000;
  const int resolution = 8;

  ledcSetup(2, freq, resolution);
  ledcSetup(3, freq, resolution);
  ledcSetup(4, freq, resolution);

  ledcAttachPin(AH_PIN, 2);
  ledcAttachPin(BH_PIN, 3);
  ledcAttachPin(CH_PIN, 4);


  // pinMode(LED_PIN, OUTPUT);
  ledcSetup(5, freq, resolution);
  ledcAttachPin(LED_PIN, 5);


  ///////////////////////////////////////////////////

  // enable pins for mosfet driver
  pinMode(AL_PIN, OUTPUT);
  pinMode(BL_PIN, OUTPUT);
  pinMode(CL_PIN, OUTPUT);

  /////////////////////////////////////////////////////////////////////////////////////
  // Read analog input from hall effect sensors and throttle
  // analogReadResolution(10);
  pinMode(HALL_1_PIN, INPUT);  // Set the hall pins as input
  pinMode(HALL_2_PIN, INPUT);
  pinMode(HALL_3_PIN, INPUT);

  pinMode(THROTTLE_PIN, INPUT);
  pinMode(EN_PIN, INPUT);

  writePWM(7, 0);
  ledcWrite(5, 255);  //pwm on enable pin
  delay(1000);
  ledcWrite(5, 0);  //pwm on enable pin
  delay(1000);
  ledcWrite(5, 255);  //pwm on enable pin

  // identifyHalls();                  // Uncomment this if you want the controller to auto-identify the hall states at startup!
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void loop() {
  // The loop function is called repeatedly, once setup() is done
  int hallState1 = digitalRead(HALL_1_PIN);
  int hallState2 = digitalRead(HALL_2_PIN);
  int hallState3 = digitalRead(HALL_3_PIN);

  int enable = digitalRead(EN_PIN);
  int throt_pwm = readThrottle();

  // uint8_t throttle = readThrottle();  // readThrottle() is slow. So do the more important things 200 times more often
  // for(uint8_t i = 0; i < 200; i++)
  // {
  //   uint8_t hall = getHalls();              // Read from the hall sensors
  //   uint8_t motorState = hallToMotor[hall]; // Convert from hall values (from 1 to 6) to motor state values (from 0 to 5) in the correct order. This line is magic
  //   writePWM(motorState, throttle);         // Actually command the transistors to switch into specified sequence and PWM value
  // }

  /////////////////////////////////////////////////////////////////////////////////////
  // step sequencer using hall effect inputs
  if (hallState1 == 1 && hallState2 == 1 && hallState3 == 0) stepstate = 3;
  else if (hallState1 == 1 && hallState2 == 0 && hallState3 == 0) stepstate = 2;
  else if (hallState1 == 1 && hallState2 == 0 && hallState3 == 1) stepstate = 1;
  else if (hallState1 == 0 && hallState2 == 0 && hallState3 == 1) stepstate = 0;
  else if (hallState1 == 0 && hallState2 == 1 && hallState3 == 1) stepstate = 5;
  else if (hallState1 == 0 && hallState2 == 1 && hallState3 == 0) stepstate = 4;
  else stepstate = 7;

  if (enable) {

    // write the output to the writePWM function, which changes mosfet driver outputs accordingly
    stepstate = 1;
    throt_pwm = 127;
    // writePWM(stepstate, throt_pwm);
    writePhases(throt_pwm, throt_pwm, throt_pwm, 0, 0, 0);  // HIGH all


    /////////////////////////////////////////////////////////////////////////////////////

    // for (int i = 0; i < 6; i++) {
    // writePWM(i, throt_pwm);
    // writePWM(i, 200);
    // writePWM(4, throt_pwm);
    // int dutyCycle = throt_pwm;
    // writePhases(dutyCycle, 0, 0, 0, 0, 0);

    // writePWM(0, 200);
    // Serial.print(i);
    // delay(throt_pwm);
    // delay(2);

    // writePhases(50, 125, 125, 0, 0, 0);

    Serial.print(readThrottle());
    Serial.print("  ");
    Serial.print(analogRead(THROTTLE_PIN));
    // Serial.print(throt_pwm);
    Serial.print("  ");
    // // Serial.print(getHalls());
    Serial.print(stepstate);

    Serial.print("  ");
    Serial.print("  ");
    Serial.print(enable);
    Serial.print("  ");
    Serial.print(hallState1);
    Serial.print("  ");
    Serial.print(hallState2);
    Serial.print("  ");
    Serial.print(hallState3);
    Serial.print("  ");
    Serial.println();
    // }

  }

  else {
    writePWM(7, 0);
    // Serial.println("disable");
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/* Magic function to do hall auto-identification. Moves the motor to all 6 states, then reads the hall values from each one
 * 
 * Note, that in order to get a clean hall reading, we actually need to commutate to half-states. So instead of going to state 3, for example
 * we commutate to state 3.5, by rapidly switching between states 3 and 4. After waiting for a while (half a second), we read the hall value.
 * Finally, print it
 */

void identifyHalls() {
  for (uint8_t i = 0; i < 6; i++) {
    uint8_t nextState = (i + 1) % 6;  // Calculate what the next state should be. This is for switching into half-states
    // Serial.print("Going to ");
    // Serial.println(i);
    for (uint16_t j = 0; j < 200; j++)  // For a while, repeatedly switch between states
    {
      delay(1);
      writePWM(i, 20);
      delay(1);
      writePWM(nextState, 20);
    }
    hallToMotor[getHalls()] = (i + 2) % 6;  // Store the hall state - motor state correlation. Notice that +2 indicates 90 degrees ahead, as we're at half states
  }

  writePWM(0, 0);  // Turn phases off

  for (uint8_t i = 0; i < 8; i++)  // Print out the array
  {
    // Serial.print(hallToMotor[i]);
    // Serial.print(", ");
  }
  Serial.println();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/* This function takes a motorState (from 0 to 5) as an input, and decides which transistors to turn on
 * dutyCycle is from 0-255, and sets the PWM value.
 * 
 * Note if dutyCycle is zero, or if there's an invalid motorState, then it turns all transistors off
 */

void writePWM(uint8_t motorState, uint8_t dutyCycle) {
  if (dutyCycle == 0)  // If zero throttle, turn all off
    motorState = 255;

  if (motorState == 0)
    writePhases(0, dutyCycle, 0, 1, 0, 0);  // LOW A, HIGH B
  else if (motorState == 1)
    writePhases(0, 0, dutyCycle, 1, 0, 0);  // LOW A, HIGH C
  else if (motorState == 2)
    writePhases(0, 0, dutyCycle, 0, 1, 0);  // LOW B, HIGH C
  else if (motorState == 3)
    writePhases(dutyCycle, 0, 0, 0, 1, 0);  // LOW B, HIGH A
  else if (motorState == 4)
    writePhases(dutyCycle, 0, 0, 0, 0, 1);  // LOW C, HIGH A
  else if (motorState == 5)
    writePhases(0, dutyCycle, 0, 0, 0, 1);  // LOW C, HIGH B
  else
    writePhases(0, 0, 0, 0, 0, 0);  // All off
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/* Helper function to actually write values to transistors. For the low sides, takes a 0 or 1 for on/off
 * For high sides, takes 0-255 for PWM value
 */

void writePhases(uint8_t ah, uint8_t bh, uint8_t ch, uint8_t al, uint8_t bl, uint8_t cl) {
  if (ah > 0 && al == 0) {
    ledcWrite(2, ah);         //pwm on enable pin
    digitalWrite(AL_PIN, 1);  // set high state

    ledcWrite(5, ah);  //pwm led with enable pin
  } else if (ah == 0 && al > 0) {
    ledcWrite(2, 0);          //turn pwm off
    digitalWrite(AL_PIN, 1);  // turn on mosfet driver

    ledcWrite(5, 0);  //pwm led with enable pin
  } else {
    digitalWrite(AL_PIN, 0);  // set low state
    ledcWrite(2, 0);          //disable pwm on enable pin

    ledcWrite(5, 0);  //pwm led with enable pin
  }

  /////////////////////////////////////////////////////////////////////////////////////

  if (bh > 0 && bl == 0) {
    ledcWrite(3, bh);         //turn pwm off
    digitalWrite(BL_PIN, 1);  // turn on mosfet driver

  } else if (bh == 0 && bl > 0) {
    ledcWrite(3, 0);          //turn pwm off
    digitalWrite(BL_PIN, 1);  // turn on mosfet driver
  } else {
    digitalWrite(BL_PIN, 0);
    ledcWrite(3, 0);
  }

  /////////////////////////////////////////////////////////////////////////////////////

  if (ch > 0 && cl == 0) {
    ledcWrite(4, ch);         //turn pwm off
    digitalWrite(CL_PIN, 1);  // turn on mosfet driver
  } else if (ch == 0 && cl > 0) {
    ledcWrite(4, 0);          //turn pwm off
    digitalWrite(CL_PIN, 1);  // turn on mosfet driver
  } else {
    digitalWrite(CL_PIN, 0);
    ledcWrite(4, 0);
  }
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


/* Read hall sensors WITH oversamping. This is required, as the hall sensor readings are often noisy.
 * This function reads the sensors multiple times (defined by HALL_OVERSAMPLE) and only sets the output
 * to a 1 if a majority of the readings are 1. This really helps reject noise. If the motor starts "cogging" or "skipping"
 * at low speed and high torque, try increasing the HALL_OVERSAMPLE value
 * 
 * Outputs a number, with the last 3 binary digits corresponding to hall readings. Thus 0 to 7, or 1 to 6 in normal operation
 */

uint8_t getHalls() {
  uint8_t hallCounts[] = { 0, 0, 0 };
  for (uint8_t i = 0; i < HALL_OVERSAMPLE; i++)  // Read all the hall pins repeatedly, tally results
  {
    hallCounts[0] += digitalRead(HALL_1_PIN);
    hallCounts[1] += digitalRead(HALL_2_PIN);
    hallCounts[2] += digitalRead(HALL_3_PIN);
  }

  uint8_t hall = 0;

  if (hallCounts[0] >= HALL_OVERSAMPLE / 2)  // If votes >= threshold, call that a 1
    hall |= (1 << 0);                        // Store a 1 in the 0th bit
  if (hallCounts[1] >= HALL_OVERSAMPLE / 2)
    hall |= (1 << 1);  // Store a 1 in the 1st bit
  if (hallCounts[2] >= HALL_OVERSAMPLE / 2)
    hall |= (1 << 2);  // Store a 1 in the 2nd bit

  return hall & 0x7;  // Just to make sure we didn't do anything stupid, set the maximum output value to 7
}


/* Read the throttle value from the ADC. Because our ADC can read from 0v-3.3v, but the throttle doesn't output over this whole range,
 * scale the throttle reading to take up the full range of 0-255
 */

uint8_t readThrottle() {
  int32_t adc = analogRead(THROTTLE_PIN);  // Note, analogRead can be slow!

  adc = lpTop.filt(adc);  // low pass filter on throttle input

  // Scale the output between 0 and 120, out of 0-255 (to limit the max speed of the car)
  if (adc < THROTTLE_LOW) {
    adc = 0;
  } else {
    adc = map(adc, THROTTLE_LOW, THROTTLE_HIGH, 0, 200);
  }
  return adc;
}
