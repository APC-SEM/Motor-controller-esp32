
#define AH_PIN 25             // Pins from the Teensy to the gate drivers. AH = A high, etc
#define AL_PIN 26
#define BH_PIN 27
#define BL_PIN 14
#define CH_PIN 12
#define CL_PIN 13


void setup() {
  // put your setup code here, to run once:
pinMode(AL_PIN,OUTPUT);
pinMode(BL_PIN,OUTPUT);
pinMode(CL_PIN,OUTPUT);

pinMode(AH_PIN,OUTPUT);
pinMode(BH_PIN,OUTPUT);
pinMode(CH_PIN,OUTPUT);
//pinMode(,OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:

digitalWrite(AH_PIN,LOW);
digitalWrite(AL_PIN,HIGH);

digitalWrite(BH_PIN,LOW);
digitalWrite(BL_PIN,HIGH);

digitalWrite(CH_PIN,LOW);
digitalWrite(CL_PIN,HIGH);

// digitalWrite(BH_PIN,LOW);
// digitalWrite(CH_PIN,LOW);

// digitalWrite(BL_PIN,HIGH);
// digitalWrite(CL_PIN,HIGH);

delay(1);

digitalWrite(AH_PIN,HIGH);
digitalWrite(AL_PIN,HIGH);

digitalWrite(BH_PIN,HIGH);
digitalWrite(BL_PIN,HIGH);

digitalWrite(CH_PIN,HIGH);
digitalWrite(CL_PIN,HIGH);

// digitalWrite(AL_PIN,LOW);
// digitalWrite(BL_PIN,LOW);
// digitalWrite(CL_PIN,LOW);

// digitalWrite(AH_PIN,HIGH);
// digitalWrite(BH_PIN,HIGH);
// digitalWrite(CH_PIN,HIGH);
delay(3);
}
