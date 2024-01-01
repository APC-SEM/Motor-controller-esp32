/*
Monash Shell Eco Marathon team
- motor controller pulse test code

code test capability
- test mosfet drivers pulse
- test transistors deadtime

Written by AndrewJNg
*/

// Mosfetr driver pinout 
#define A_IN 25 
#define A_SD 26
#define B_IN 27
#define B_SD 14
#define C_IN 12
#define C_SD 13

#define THROTTLE_PIN 32

// Built in LED pin
#define LED_PIN 2  

void setup() {
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
  
  pinMode(THROTTLE_PIN, INPUT);
  Serial.begin(115200);
}

void loop() {
  int dutyCycle =  map(analogRead(THROTTLE_PIN),0,4096,0,255);
  // writePhases(dutyCycle, dutyCycle, dutyCycle, 0, 0, 0);
  writePhases(dutyCycle, 0, 0, 0, 0, 0);
  delay(1);
  writePhases(0, 0, 0, 255, 0, 0);
  // writePhases(0, 0, 0, dutyCycle, dutyCycle, dutyCycle);
  delay(1);
  Serial.println(dutyCycle);
}

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
  
  /////////////////////////////////

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

  /////////////////////////////////

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
