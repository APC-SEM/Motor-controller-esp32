void motor_setup();
void writePWM(uint8_t motorState, uint8_t dutyCycle);
void writePhases(uint8_t ah, uint8_t bh, uint8_t ch, uint8_t al, uint8_t bl, uint8_t cl);

void motor_setup()
{
  /////////////////////////////////////////////////////////////////////////////////////
  // Mosfet driver setup
  const int freq = 20000;
  const int resolution = 8;

  // SD pin setup for mosfet drivers
  ledcSetup(2, freq, resolution);
  ledcSetup(3, freq, resolution);
  ledcSetup(4, freq, resolution);

  ledcAttachPin(A_SD, 2);
  ledcAttachPin(B_SD, 3);
  ledcAttachPin(C_SD, 4);

  /////////////////////////////////
  // IN pin setup for mosfet drivers
  pinMode(A_IN, OUTPUT);
  pinMode(B_IN, OUTPUT);
  pinMode(C_IN, OUTPUT);

  /////////////////////////////////
  // indicator LED
  ledcAttachPin(LED_PIN, 2); 
  
  // disable all mosfets
  writePWM(7, 0);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/* This function takes a motorState (from 0 to 5) as an input, and decides which transistors to turn on
 * dutyCycle is from 0-255, and sets the PWM value.
 * 
 * Note if dutyCycle is zero, or if there's an invalid motorState, then it turns all transistors off
 */

void writePWM(uint8_t motorState, uint8_t dutyCycle) {
  if (dutyCycle == 0)  // If zero throttle, turn all off
    motorState = 7;

  if (motorState == 0)
    writePhases(0, dutyCycle, 0, 255, 0, 0);  // HIGH B, LOW A 
  else if (motorState == 1)
    writePhases(0, 0, dutyCycle, 255, 0, 0);  // HIGH C, LOW A
  else if (motorState == 2)
    writePhases(0, 0, dutyCycle, 0, 255, 0);  // HIGH C, LOW B 
  else if (motorState == 3)
    writePhases(dutyCycle, 0, 0, 0, 255, 0);  // HIGH A, LOW B 
  else if (motorState == 4)
    writePhases(dutyCycle, 0, 0, 0, 0, 255);  // HIGH A, LOW C
  else if (motorState == 5)
    writePhases(0, dutyCycle, 0, 0, 0, 255);  // HIGH B, LOW C
  else
    writePhases(0, 0, 0, 0, 0, 0);  // All off
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/* Helper function to actually write values to transistors. For the low sides, takes a 0 or 1 for on/off
 * For high sides, takes 0-255 for PWM value
 */

void writePhases(uint8_t ah, uint8_t bh, uint8_t ch, uint8_t al, uint8_t bl, uint8_t cl) {
  if (ah > 0 && al == 0) {  
    // HIGH SIDE
    digitalWrite(A_IN, 1);  // turn on High side mosfet
    ledcWrite(2, ah);       // pwm on SD pin
  } else if (ah == 0 && al > 0) { 
    // LOW SIDE
    digitalWrite(A_IN, 0);  // turn on Low side mosfet
    ledcWrite(2, al);       // pwm on SD pin
  } else {
    // FLOATING
    digitalWrite(A_IN, 0);  // this state doesn't matter
    ledcWrite(2, 0);        // set SD pin low
  }
  
  ///////////////////////////////

  if (bh > 0 && bl == 0) {
    digitalWrite(B_IN, 1);  
    ledcWrite(3, bh);      
  } else if (bh == 0 && bl > 0) {
    digitalWrite(B_IN, 0);  
    ledcWrite(3, bl);       
  } else {
    digitalWrite(B_IN, 0);  
    ledcWrite(3, 0);        
  }

  ///////////////////////////////

  if (ch > 0 && cl == 0) { 
    digitalWrite(C_IN, 1);  
    ledcWrite(4, ch);         
  } else if (ch == 0 && cl > 0) {
    digitalWrite(C_IN, 0);  
    ledcWrite(4, cl);          
  } else {
    digitalWrite(C_IN, 0);
    ledcWrite(4, 0);
  }
}