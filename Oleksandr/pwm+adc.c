/***********************************************************
 * 
 * Blink a LED using register approach and control a PWM
 * servo position based on ADC values.
 * (c) 2018-2024 Tomas Fryza, MIT license
 *
 * Developed using PlatformIO and AVR 8-bit Toolchain 3.6.2.
 * Tested on Arduino Uno board and ATmega328P, 16 MHz.
 * 
 ***********************************************************/

/* Defines ------------------------------------------------*/
#define LED_GREEN PB5   // AVR pin where green LED is connected
#define LED_1 PB1
#define LED_2 PB2
#define LED_3 PB3
#define BUTTON PD2
#define SHORT_DELAY 250 // Delay in milliseconds
#ifndef F_CPU
#define F_CPU 16000000 // CPU frequency in Hz required for delay
#endif

/* Includes -----------------------------------------------*/
#include <avr/io.h>     // AVR device-specific IO definitions
#include <util/delay.h> // Functions for busy-wait delay loops
#include <gpio.h>
#include <timer.h> 
#include <adc.h>

/* Function definitions -----------------------------------*/
void setupPWM_50Hz() {
    // Set Pin 9 (OC1A) as output
    DDRB |= (1 << PB1);
    
    // Configure Timer1 for Fast PWM mode, with ICR1 as TOP
    TCCR1A = (1 << WGM11) | (1 << COM1A1);  // Fast PWM, non-inverting mode on OC1A
    TCCR1B = (1 << WGM12) | (1 << WGM13) | (1 << CS11); // Prescaler = 8, Fast PWM mode
    
    // Set TOP value to get a 50Hz frequency
    ICR1 = 39999; // 50Hz period (20ms)
    
    // Set initial pulse width to 1ms (0 position)
    OCR1A = 1101; // 1ms pulse width corresponds to 0 degrees
}

int main(void) {
    // Initialize GPIO for LEDs and button
    GPIO_mode_output(&DDRB, LED_GREEN);
    GPIO_mode_output(&DDRB, LED_1);
    GPIO_mode_output(&DDRB, LED_2);
    GPIO_mode_output(&DDRB, LED_3);
    GPIO_mode_input_pullup(&DDRD, BUTTON);
  
    // Initialize ADC with AVCC as reference
    initADC(AVCC);

    // Initialize PWM for servo control
    setupPWM_50Hz();

    // Enable timers
    TIM1_ovf_enable();
    TIM2_ovf_1ms();

    // Initialize LEDs state
    GPIO_write_high(&PORTB, LED_1);
    GPIO_write_low(&PORTB, LED_2);
    GPIO_write_high(&PORTB, LED_3);

    while(1) {
        // Read analog values from pins A1, A2, and A3
        uint16_t adc1 = analogRead(A1);
        uint16_t adc2 = analogRead(A2);
        uint16_t adc3 = analogRead(A3);

        // Determine the highest ADC value
        uint16_t max = adc1;
        if(adc2 > max){
            max = adc2;
        }
        if(adc3 > max){
            max = adc3;
        }

        // Control LEDs based on the highest ADC value
        if(max == adc1){
            GPIO_write_high(&PORTB, LED_1);
            GPIO_write_low(&PORTB, LED_2);
            GPIO_write_low(&PORTB, LED_3);

            // Set PWM for position 1 (0 degrees)
            OCR1A = 1101; // 1ms pulse width for 0 degrees
        }
        else if(max == adc2){
            GPIO_write_low(&PORTB, LED_1);
            GPIO_write_high(&PORTB, LED_2);
            GPIO_write_low(&PORTB, LED_3);

            // Set PWM for position 2 (45 degrees)
            OCR1A = 2041; // 1.021ms pulse width for 45 degrees
        }
        else if(max == adc3){
            GPIO_write_low(&PORTB, LED_1);
            GPIO_write_low(&PORTB, LED_2);
            GPIO_write_high(&PORTB, LED_3);

            // Set PWM for position 3 (90 degrees)
            OCR1A = 2979; // 1.489ms pulse width for 90 degrees
        }

        // Delay to ensure changes are visible and the PWM signal remains stable
        _delay_ms(100);
    }
}
