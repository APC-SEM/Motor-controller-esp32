#include "driver/mcpwm.h"

void motor_gpio_setup();
void writePWM(uint motorState, uint duty, bool synchronous);
void writePhases(uint8_t ah, uint8_t bh, uint8_t ch, uint8_t al, uint8_t bl, uint8_t cl);

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// This setup is for a full 3 channel half bridge for a BLDC motor
void motor_gpio_setup()
{
   // Initialize GPIOs for half bridges (there are two MCPWM units, each capable of 6 PWM output or equilavent to 3 half bridges)
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, AH); // HIGH side
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, AL); // LOW side

    // mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1A, BH); // Timer 1, High side
    // mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1B, BL); // Timer 1, Low side
    
    // mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM2A, CH); // Timer 2, High side
    // mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM2B, CL); // Timer 2, Low side

    // For now we are using only 1 channel, so turn channel B and C off
    ///*
    pinMode(BH,OUTPUT);
    pinMode(BL,OUTPUT);
    pinMode(CH,OUTPUT);
    pinMode(CL,OUTPUT);

    digitalWrite(BH, LOW);   // turn the LED off by making the voltage LOW
    digitalWrite(BL, LOW);   // turn the LED off by making the voltage LOW
    digitalWrite(CH, LOW);   // turn the LED off by making the voltage LOW
    digitalWrite(CL, LOW);   // turn the LED off by making the voltage LOW
    //*/

    // set pwm_config for centered aligned pwm, set to 0 for start
    pwm_config.frequency = system_freq*2;                  // set to double the required frequency
    pwm_config.cmpr_a = 0;                                 // Duty cycle of PWMxA
    pwm_config.cmpr_b = 0;                                 // Note: the duty cycle of PWMxB is inverted (from where "100-x")
    pwm_config.counter_mode = MCPWM_UP_DOWN_COUNTER;       // creates symetrical waveforms at half frequency 5kHz
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;              // active high for PWMA

    // load settings to MCPWM timers
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);  
    // mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_1, &pwm_config);  
    // mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_2, &pwm_config);  

    // Configure deadtime to avoid signal overlap 
    mcpwm_deadtime_enable(MCPWM_UNIT_0, MCPWM_TIMER_0,
                          MCPWM_ACTIVE_HIGH_COMPLIMENT_MODE, // High-side and low-side complementary
                          DEADTIME_US *10, // Deadtime in 0.1us ticks for rising edge 
                          DEADTIME_US *10 // Deadtime in 0.1us ticks for falling edge
    );
    // mcpwm_deadtime_enable(MCPWM_UNIT_0, MCPWM_TIMER_1,
    //                       MCPWM_ACTIVE_HIGH_COMPLIMENT_MODE, // High-side and low-side complementary
    //                       DEADTIME_US *10, // Deadtime in 0.1us ticks for rising edge 
    //                       DEADTIME_US *10 // Deadtime in 0.1us ticks for falling edge
    // );
    // mcpwm_deadtime_enable(MCPWM_UNIT_0, MCPWM_TIMER_2,
    //                       MCPWM_ACTIVE_HIGH_COMPLIMENT_MODE, // High-side and low-side complementary
    //                       DEADTIME_US *10, // Deadtime in 0.1us ticks for rising edge 
    //                       DEADTIME_US *10 // Deadtime in 0.1us ticks for falling edge
    // );
    
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 0);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, 100);  // turn off low side at startup for BDC motor



    delay(500);
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/* This function takes a motorState (from 0 to 5) as an input, and decides which transistors to turn on
 * dutyCycle is from 0-255, and sets the PWM value.
 * 
 * Note if dutyCycle is zero, or if there's an invalid motorState, then it turns all transistors off
 */

void writePWM(uint motorState, uint duty, bool synchronous) {
    // Switch the transistors given a desired electrical state and duty cycle
    // motorState: desired electrical position, range of 0-5
    // duty: desired duty cycle, range of 0-255
    // synchronous: perfom synchronous (low-side and high-side alternating) or non-synchronous switching (high-side only) 

    if(duty == 0 || duty > 255)     // If zero throttle, turn both low-sides and high-sides off
        motorState = 255;

    // At near 100% duty cycles, the gate driver bootstrap capacitor may become discharged as the high-side gate is repeatedly driven
    // high and low without time for the phase voltage to fall to zero and charge the bootstrap capacitor. Thus, if duty cycle is near
    // 100%, clamp it to 100%.
    if(duty > 245)
        duty = 255;

    uint complement = 100;
    if(synchronous)
    {
        complement = duty;    // Provide switching deadtime by having duty + complement < 255
        // complement = max(0, 97-(int)duty);    // Provide switching deadtime by having duty + complement < 255
    }

    if(motorState == 0)                         // LOW A, HIGH B
        writePhases(0, duty, 0, 100, complement, 0);
    else if(motorState == 1)                    // LOW A, HIGH C
        writePhases(0, 0, duty, 100, 0, complement);
    else if(motorState == 2)                    // LOW B, HIGH C
        writePhases(0, 0, duty, 0, 100, complement);
    else if(motorState == 3)                    // LOW B, HIGH A
        writePhases(duty, 0, 0, complement, 100, 0);
    else if(motorState == 4)                    // LOW C, HIGH A
        writePhases(duty, 0, 0, complement, 0, 100);
    else if(motorState == 5)                    // LOW C, HIGH B
        writePhases(0, duty, 0, 0, complement, 100);
    else                                        // All transistors off
        writePhases(0, 0, 0, 0, 0, 0);
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/* Helper function to actually write values to transistors. For the low sides, takes a 0 or 1 for on/off
 * For high sides, takes 0-255 for PWM value
 */

void writePhases(uint8_t ah, uint8_t bh, uint8_t ch, uint8_t al, uint8_t bl, uint8_t cl) {

    // Phase A
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, ah);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, al); // A = (100 - A) 
    
    // // Phase B
    // mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, bh);
    // mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B, bl); // B = (100 - A) 
    
    // // Phase C
    // mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_A, ch);
    // mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_B, cl); // C = (100 - A) 



}


























