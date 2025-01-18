#define THROTTLE_PIN 34   // Throttle pin
#define THROTTLE_LOW 150  // These LOW and HIGH values are used to scale the ADC reading. More on this below
#define THROTTLE_HIGH 255

#define AH_PIN 25  // Pins from the Teensy to the gate drivers. AH = A high, etc
#define AL_PIN 26
#define BH_PIN 27
#define BL_PIN 14
#define CH_PIN 12
#define CL_PIN 13

const int freq = 5000;
const int resolution = 8;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  analogReadResolution(8);
pinMode(AL_PIN,OUTPUT);
pinMode(BL_PIN,OUTPUT);
pinMode(CL_PIN,OUTPUT);

pinMode(AH_PIN,OUTPUT);
pinMode(BH_PIN,OUTPUT);
pinMode(CH_PIN,OUTPUT);

  ledcSetup(2, freq, resolution);
  ledcSetup(3, freq, resolution);
  ledcSetup(4, freq, resolution);

  ledcAttachPin(AL_PIN, 2);
  // ledcAttachPin(BH_PIN, 3);
  // ledcAttachPin(CH_PIN, 4);
//pinMode(,OUTPUT);
}

void loop() {
    // ledcWrite(2, readThrottle() );
    ledcWrite(2, 125);
digitalWrite(AH_PIN,HIGH);
digitalWrite(BH_PIN,HIGH);
digitalWrite(CH_PIN,HIGH);

delay(20);
    ledcWrite(2, 125);
digitalWrite(AH_PIN,LOW);
digitalWrite(BH_PIN,LOW);
digitalWrite(CH_PIN,LOW);

delay(20);
  // put your main code here, to run repeatedly:

  // Serial.print(readThrottle());
  // Serial.println();
// digitalWrite(AH_PIN,LOW);
// digitalWrite(BH_PIN,LOW);
// digitalWrite(CH_PIN,LOW);

// digitalWrite(AL_PIN,HIGH);
// digitalWrite(BL_PIN,HIGH);
// digitalWrite(CL_PIN,HIGH);

// delay(100);


// digitalWrite(AH_PIN,HIGH);
// digitalWrite(BH_PIN,HIGH);
// digitalWrite(CH_PIN,HIGH);
// delay(100);
}

uint8_t readThrottle() {
  int32_t adc = analogRead(THROTTLE_PIN);  // Note, analogRead can be slow!
  // adc = (adc - THROTTLE_LOW) << 8;
  // adc = adc / (THROTTLE_HIGH - THROTTLE_LOW);

  adc = map(adc, THROTTLE_LOW, THROTTLE_HIGH, 0, 255);

  if (adc > 200)  // Bound the output between 0 and 255
    return 200;

  if (adc < 40)
    return 0;

  return adc;
}
