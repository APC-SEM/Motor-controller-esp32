#include "driver/mcpwm.h"
#include <stdio.h>

// troubleshooting settings
#define debug 1

// Pin definition
#define AH 25//12 // Output pin for MCPWM0A // HIGH Side
#define AL 26//13 // Output pin for MCPWM0B // LOW Side

#define BH 27 // Output pin for MCPWM1A // HIGH Side
#define BL 14 // Output pin for MCPWM1B // LOW Side

#define CH 12 // Output pin for MCPWM2A // HIGH Side
#define CL 13 // Output pin for MCPWM2B // LOW Side

#define LED_PIN 2 // Status led

// Analog input pins
// const uint ISENSE_PIN = 26;
// const uint VSENSE_PIN = 27;
// const uint THROTTLE_PIN = 28;
const uint THROTTLE_PIN = 32;

// Motor control system parameters
const int system_freq = 5000; // 5 kHz pwm freq
const int min_pwm_percent = 20; // Only turn on mosfets when above 20% to reduce in-rush current spike 
#define DEADTIME_US 10 // set 10 us deadtime for rising and falling edge (not inlcuding mosfet driver deadtime)
mcpwm_config_t pwm_config;  // initialize "pwm_config" structure
#include "motor.h"

// Throttle settings and memory setting
int min_throttle = 0;                // ADC value corresponding to minimum throttle, 0-4095
int max_throttle = 4096;             // ADC value corresponding to maximum throttle, 0-4095
#include "memory.h"



/*
const bool CURRENT_CONTROL = true;          // Use current control or duty cycle control
const int PHASE_MAX_CURRENT_MA = 6000;      // If using current control, the maximum phase current allowed
const int BATTERY_MAX_CURRENT_MA = 3000;    // If using current control, the maximum battery current allowed
const int CURRENT_CONTROL_LOOP_GAIN = 200;  // Adjusts the speed of the current control loop
const int CURRENT_SCALING = 3.3 / 0.0005 / 20 / 4096 * 1000;
const int VOLTAGE_SCALING = 3.3 / 4096 * (47 + 2.2) / 2.2 * 1000;
const int ADC_BIAS_OVERSAMPLE = 1000;

int adc_isense = 0;
int adc_vsense = 0;
int adc_throttle = 0;

int adc_bias = 0;
int voltage_mv = 0;
int current_ma = 0;
int current_target_ma = 0;
*/

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup() {
    // Setup mosfets to turn off
    motor_gpio_setup();
    
    // setup status LED
    ledcSetup(2, system_freq, 8);  
    ledcAttachPin(LED_PIN, 2);

    // setup input throttle 
    pinMode(THROTTLE_PIN, INPUT);
    
    
    // Initialize serial monitor for debugging
    if (debug) {
      Serial.begin(115200);
      Serial.println("Enter duty cycle (0 to 100):");
    } 
    // for(uint i = 0; i < ADC_BIAS_OVERSAMPLE; i++)   // Find the zero-current ADC reading. Reads the ADC multiple times and takes the average
    // {
    //     adc_bias += analogRead(ISENSE_PIN);
    // }
    // adc_bias /= ADC_BIAS_OVERSAMPLE;



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
  
  delay(500); // delay to ensure values are up to date from memory
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
float throttle_Read()
{
  static float throtDutyCycle = 0;

  /*
  // Serial.print for testing specific input throttle
  if (Serial.available() > 0) {

    // Read the input string
    String input = Serial.readStringUntil('\n');
    input.trim(); // Remove whitespace

    //Convert the string to a float
    float throttle = input.toFloat();
  }
  */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Read From Throttle Pin
  int throttle = analogRead(THROTTLE_PIN);
  // if (throttle > max_throttle) max_throttle = throttle;
  // else if (throttle < min_throttle) min_throttle = throttle;

  throtDutyCycle =  map(throttle,min_throttle,max_throttle,0,100); // divide 100 to have 2 decimal place floating point
  // throtDutyCycle = constrain(throttle,0,10000); // constraint it so it doesn't give unwanted results
  // throtDutyCycle /=100;
  
  if (debug) {
    Serial.print("throtDutyCycle: ");
    Serial.println(throtDutyCycle);
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

      if((currentTime - lastUpdatedLoopTime)>= 50){ // 0.05s per cycle
          if((user_demand_dutyCycle < 5) && (dutyCycleLoop < min_pwm_percent)){ 
            dutyCycleLoop=min_pwm_percent-1; // set to 1 below the minimum amount to turn on mosfets, for fast response when loop is entered
            dutyCycleOutput = dutyCycleLoop;
          }
          else if(user_demand_dutyCycle > dutyCycleLoop){ 
            // Smooth acceleration: Increase the duty cycle incrementally.
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
      
      if (debug) Serial.println(dutyCycleOutput);
      
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      // Apply PWM Output to MOSFETs
      // Apply duty cycle limits to prevent very low or high outputs.
      if(dutyCycleOutput < 20) dutyCycleOutput = 0; // Cut off below 20%.
      if(dutyCycleOutput > 90) dutyCycleOutput = 100; // Cap above 90% at 100%.

      // Update duty cycles
      // mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, dutyCycle);
      // mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, dutyCycleLoop);
      mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, 100-dutyCycleOutput);
      ledcWrite(2, dutyCycleOutput*2);   

      // RAW pwm input
      // mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, 100-user_demand_dutyCycle);
      // ledcWrite(2, user_demand_dutyCycle*2); 
            
    
}