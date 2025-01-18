#include "driver/mcpwm.h"
#include <stdio.h>

// troubleshooting settings
#define debug 0

// Pin definition
#define AH 12 // Output pin for MCPWM0A // HIGH Side
#define AL 13 // Output pin for MCPWM0B // LOW Side

#define BH 27 // Output pin for MCPWM1A // HIGH Side
#define BL 14 // Output pin for MCPWM1B // LOW Side

#define CH 25 // Output pin for MCPWM2A // HIGH Side
#define CL 26 // Output pin for MCPWM2B // LOW Side

#define LED_PIN 2 // Status led

// Analog input pins
const uint ISENSE_PIN = 32;
// const uint VSENSE_PIN = 27;
const uint THROTTLE_PIN = 35;
const uint Hall1_Pin = 36;
const uint Hall2_Pin = 39;
const uint Hall3_Pin = 34;
// const uint THROTTLE_PIN = 28;



// Motor control system parameters
const int system_freq = 5000; // 5 kHz pwm freq
const int min_pwm_percent = 20; // Only turn on mosfets when above 20% to reduce in-rush current spike 
#define DEADTIME_US 30 // set 10 us deadtime for rising and falling edge (not inlcuding mosfet driver deadtime)
mcpwm_config_t pwm_config;  // initialize "pwm_config" structure
#include "motor.h"

// Throttle settings and memory setting
int min_throttle = 0;                // ADC value corresponding to minimum throttle, 0-4095
int max_throttle = 4096;             // ADC value corresponding to maximum throttle, 0-4095
#include "memory.h"


const int DUTY_CYCLE_MAX = 100;


const bool CURRENT_CONTROL = true;          // Use current control or duty cycle control
const int PHASE_MAX_CURRENT_MA = 800;      // If using current control, the maximum phase current allowed
const int BATTERY_MAX_CURRENT_MA = 3000;    // If using current control, the maximum battery current allowed
const int CURRENT_CONTROL_LOOP_GAIN = 50;  // Adjusts the speed of the current control loop
/*
int adc_isense = 0;
int adc_vsense = 0;
int adc_throttle = 0;

*/
int adc_bias = 0;
const int ADC_BIAS_OVERSAMPLE = 1000;

const int CURRENT_SCALING = 3.3 / 0.0005 / 50 / 4096 * 1000;
const int VOLTAGE_SCALING = (3.3 / 4096 ) * (((47 + 2.2) / 2.2) * 1000);
int voltage_mv = 0;
int current_ma = 0;
int current_target_ma = 0;
int duty_cycle = 0;

int stepstate = 0;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup() {
    // Setup mosfets to turn off
    motor_gpio_setup();
    
    // setup status LED
    ledcSetup(2, system_freq, 8);  
    ledcAttachPin(LED_PIN, 2);

    // setup input throttle 
    pinMode(THROTTLE_PIN, INPUT);
    pinMode(Hall1_Pin, INPUT);
    pinMode(Hall2_Pin, INPUT);
    pinMode(Hall3_Pin, INPUT);
    
    
    // Initialize serial monitor for debugging
    if (debug) {
      Serial.begin(115200);
      Serial.println("Enter duty cycle (0 to 100):");
    } 



///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Calibration mode
  static unsigned long previousMillis = 0;

  if (debug) Serial.println("entered calibration mode");
  int throttle = analogRead(THROTTLE_PIN);
  if (debug) Serial.println(throttle);


  if (throttle > (4096 / 2)) {  // only enter calibration mode if throttle is more than 50% of ADC range
    previousMillis = millis();
    int throttle_percent = 0;
    unsigned long calib_time = millis() - previousMillis;

    // initialise throttle value to the wrong exteme, to be updated by the program (min is 4096, so that any new value will update min)
    min_throttle = 4096;               
    max_throttle = 0;             

    /////////////////////////////////////////////////////////////////
    // enter calibration mode
    do {
      throttle = analogRead(THROTTLE_PIN);
      if (throttle > max_throttle) max_throttle = throttle;
      else if (throttle < min_throttle) min_throttle = throttle;

      throttle_percent = map(throttle, min_throttle, max_throttle, 0, 100);
      calib_time = millis() - previousMillis;


      // activate channel for LED to indicate calibration mode
      if(calib_time < 5000){  // Blink status led 1Hz to show it is within 5 second window
        if ((calib_time % 1000) < 500)  ledcWrite(2, 255); else ledcWrite(2, 0);
      }
      else ledcWrite(2, 0);

      /////////////////////////////////////////////////////////////////
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
    delay(500); // delay to store memory without issues
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

  // Current sensor calibration
    delay(100);
    for(uint i = 0; i < ADC_BIAS_OVERSAMPLE; i++)   // Find the zero-current ADC reading. Reads the ADC multiple times and takes the average
    {
        adc_bias += analogRead(ISENSE_PIN);
    }
    adc_bias /= ADC_BIAS_OVERSAMPLE;
    
  
  delay(500); // delay to ensure values are up to date from memory
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
float throttle_Read()
{
  static float throtDutyCycle = 0;

// Read From Throttle Pin
  int throttle = analogRead(THROTTLE_PIN);
  // if (throttle > max_throttle) max_throttle = throttle;
  // else if (throttle < min_throttle) min_throttle = throttle;

  throtDutyCycle =  map(throttle,min_throttle,max_throttle,0,100); // divide 100 to have 2 decimal place floating point
  if (throtDutyCycle>95)throtDutyCycle = 100;
  // if (throtDutyCycle>95)throtDutyCycle = 100;
  
  
  if (debug) {
    // Serial.print("throtDutyCycle: ");
    // Serial.println(throtDutyCycle);
  }
  return throtDutyCycle;
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop() {
    // input variable
    static float user_demand_dutyCycle = 0;

    // processing variable
    static float dutyCycleLoop= 0; //initial speed of zero
    static unsigned long lastUpdatedLoopTime = 0;

    // output variable
    static float dutyCycleOutput = 0;

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // read input throttle data
      user_demand_dutyCycle = throttle_Read();
      unsigned long currentTime = millis();


      

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Applying Linear Ramp function, with smoothen acceleration, hard braking and software speed recovery
      // Apply Linear Ramp Function
      // Smooth acceleration, hard braking, and software-controlled speed recovery are implemented here.
      // The control loop updates every 0.05 seconds (50ms).


    current_ma = (analogRead(32)-adc_bias)*CURRENT_SCALING;


      if((currentTime - lastUpdatedLoopTime)>= 50){ // 0.05s per cycle
          if((user_demand_dutyCycle < 5) && (dutyCycleLoop < min_pwm_percent)){ 
            dutyCycleLoop=min_pwm_percent-1; // set to 1 below the minimum amount to turn on mosfets, for fast response when loop is entered
            dutyCycleOutput = dutyCycleLoop;
          }
          else if(user_demand_dutyCycle > dutyCycleLoop){ 
            // Smooth acceleration: Increase the duty cycle incrementally.
            // if(current_ma < PHASE_MAX_CURRENT_MA)
              dutyCycleLoop += 1; // pwm% increase per cycle
              
            dutyCycleOutput = dutyCycleLoop;
          }
          else if (user_demand_dutyCycle <= dutyCycleLoop){
            // Deceleration: Decrease the duty cycle more aggressively.  Software ramping down to decelerate, so that when accelerate again, it won't be from zero
              dutyCycleLoop -= 2; // pwm% decrease per cycle
            dutyCycleOutput = user_demand_dutyCycle;
          }

          // Update the last time the control loop was processed.
          lastUpdatedLoopTime= millis();
      }
      
      if (debug) 
      {
        // Serial.print(current_ma);
        // Serial.print("  ");
        // Serial.print(adc_bias);
        // Serial.print("  ");

        // Serial.print(analogRead(Hall1_Pin));
        // Serial.print("  ");
        // Serial.print(analogRead(Hall2_Pin));
        // Serial.print("  ");
        // Serial.print(analogRead(Hall3_Pin));
        // Serial.print("  ");

        // Serial.print(digitalRead(Hall1_Pin));
        // Serial.print("  ");
        // Serial.print(digitalRead(Hall2_Pin));
        // Serial.print("  ");
        // Serial.print(digitalRead(Hall3_Pin));
        // Serial.print("  ");
        // Serial.println(dutyCycleOutput);
      }


  int hallState1 = digitalRead(Hall1_Pin);
  int hallState2 = digitalRead(Hall2_Pin);
  int hallState3 = digitalRead(Hall3_Pin);
  
  static int motorState=0;

  /////////////////////////////////////////////////////////////////////////////////////
  // step sequencer using hall effect inputs
  // /*
  if (hallState1 == 0 && hallState2 == 0 && hallState3 == 1) motorState = 0;
  else if (hallState1 == 1 && hallState2 == 0 && hallState3 == 1) motorState = 1;
  else if (hallState1 == 1 && hallState2 == 0 && hallState3 == 0) motorState = 2;
  else if (hallState1 == 1 && hallState2 == 1 && hallState3 == 0) motorState = 3;
  else if (hallState1 == 0 && hallState2 == 1 && hallState3 == 0) motorState = 4;
  else if (hallState1 == 0 && hallState2 == 1 && hallState3 == 1) motorState = 5;
  else {
    motorState = 7;
    // if (debug) {
      // Serial.print("unstable step");

      // Serial.print("    ");
      // Serial.print("curr step= ");
      // Serial.print(stepstate);
      // Serial.print("  h1:");
      // Serial.print(hallState1);
      // Serial.print("  h2:");
      // Serial.print(hallState2);
      // Serial.print("  h3:");
      // Serial.print(hallState3);
      // Serial.println();
    // }
  }

  
  // */



      
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      // Apply PWM Output to MOSFETs
      // Apply duty cycle limits to prevent very low or high outputs.
      // dutyCycleOutput = user_demand_dutyCycle;
      // dutyCycleOutput = 40;
      if(dutyCycleOutput < 20) dutyCycleOutput = 0; // Cut off below 20%.
      if(dutyCycleOutput > 100) dutyCycleOutput = 100; // Cap above 95% to 100%.

      // float complement = 100-dutyCycleOutput;
      // motorState=0;
    // if(motorState == 0)                         // LOW A, HIGH B
    //     writePhases(0, dutyCycleOutput, 0, 100, complement, 0);
    // else if(motorState == 1)                    // LOW A, HIGH C
    //     writePhases(0, 0, dutyCycleOutput, 100, 0, complement);
    // else if(motorState == 2)                    // LOW B, HIGH C
    //     writePhases(0, 0, dutyCycleOutput, 0, 100, complement);
    // else if(motorState == 3)                    // LOW B, HIGH A
    //     writePhases(dutyCycleOutput, 0, 0, complement, 100, 0);
    // else if(motorState == 4)                    // LOW C, HIGH A
    //     writePhases(dutyCycleOutput, 0, 0, complement, 0, 100);
    // else if(motorState == 5)                    // LOW C, HIGH B
    //     writePhases(0, dutyCycleOutput, 0, 0, complement, 100);
    // else                                        // All transistors off
    //     writePhases(0, 0, 0, 0, 0, 0);
    
    

      float complement = 0;
  if (motorState == 0)
    writePhases(0, dutyCycleOutput, 0, 100, complement, 0);  // HIGH B, LOW A 
  else if (motorState == 1)
    writePhases(0, 0, dutyCycleOutput, 100, 0, complement);  // HIGH C, LOW A
  else if (motorState == 2)
    writePhases(0, 0, dutyCycleOutput, 0, 100, complement);  // HIGH C, LOW B 
  else if (motorState == 3)
    writePhases(dutyCycleOutput, 0, 0, complement, 100, 0);  // HIGH A, LOW B 
  else if (motorState == 4)
    writePhases(dutyCycleOutput, 0, 0, complement, 0, 100);  // HIGH A, LOW C
  else if (motorState == 5)
    writePhases(0, dutyCycleOutput, 0, 0, complement, 100);  // HIGH B, LOW C
  else
    writePhases(0, 0, 0, 0, 0, 0);  // All off

    ledcWrite(2, dutyCycleOutput*2);  
    // delay(50);
    // motorState++; 
    // motorState = motorState % 6; // Ensures motorState stays in the range [0, 5]

    
      // Serial.print("  ");
      // Serial.print(hallState1);
      // Serial.print("  ");
      // Serial.print(hallState2);
      // Serial.print("  ");
      // Serial.print(hallState3);
      // Serial.print("  ");
      // Serial.println(motorState);


      // Serial.print(VOLTAGE_SCALING);
      // Serial.print("  ");

      // Serial.println(analogRead(33));
      // Serial.print(CURRENT_SCALING);
      // Serial.print("  ");

      // Serial.print((analogRead(32)));
      // Serial.print("  ");
      // Serial.println((analogRead(32)-adc_bias)*-CURRENT_SCALING);
      // Serial.println(analogRead(THROTTLE_PIN));
      // delay(50);
}