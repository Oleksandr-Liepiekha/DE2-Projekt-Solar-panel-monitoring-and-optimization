/***********************************************************
 *
 * Blink a LED using register approach.
 * (c) 2018-2024 Tomas Fryza, MIT license
 *
 * Developed using PlatformIO and AVR 8-bit Toolchain 3.6.2.
 * Tested on Arduino Uno board and ATmega328P, 16 MHz.
 *
 ***********************************************************/

/* Defines ------------------------------------------------*/
#define LED_GREEN PB5 // AVR pin where green LED is connected
#define LED_1 PB1
#define LED_2 PB2
#define LED_3 PB3
#define LED_4 PB4
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
/***********************************************************
 * Function: Main function where the program execution begins
 * Purpose:  Toggle one LED and use delay library.
 * Returns:  none
 ***********************************************************/
int main(void)
{

  GPIO_mode_output(&DDRB, LED_GREEN);
  GPIO_mode_output(&DDRB, LED_1);
  GPIO_mode_output(&DDRB, LED_2);
  GPIO_mode_output(&DDRB, LED_3);
  GPIO_mode_input_pullup(&DDRD, BUTTON);

  // konstanty pro proud
  uint16_t konstanta = 185;
  // proměnná pro nastavení offsetu, polovina Vcc
  uint16_t offset = 2500;

  initADC(AVCC);

  TIM1_ovf_enable();
  TIM2_ovf_1ms();

  GPIO_write_high(&PORTB, LED_1);
  GPIO_write_low(&PORTB, LED_2);
  GPIO_write_high(&PORTB, LED_3);

  while (1)
  {
    // na photoresistory
    uint16_t adc1 = analogRead(A1);
    uint16_t adc2 = analogRead(A2);
    uint16_t adc3 = analogRead(A3);
    uint16_t adc4 = analogRead(A4);
    // ###############

    // najvetsi napeti na photoresistorech
    uint16_t max = adc1;
    if (adc2 > max)
    {
      max = adc2;
    }
    if (adc3 > max)
    {
      max = adc3;
    }
    if (adc4 > max)
    {
      max = adc4;
    }
    // #########################

    // nejvetsi napeti - sviti ledka (jen test)
    if (max == adc1)
    {
      GPIO_write_high(&PORTB, LED_1);
      GPIO_write_low(&PORTB, LED_2);
      GPIO_write_low(&PORTB, LED_3);
      GPIO_write_low(&PORTB, LED_4);
    }
    else if (max == adc2)
    {
      GPIO_write_low(&PORTB, LED_1);
      GPIO_write_high(&PORTB, LED_2);
      GPIO_write_low(&PORTB, LED_3);
      GPIO_write_low(&PORTB, LED_4);
    }
    else if (max == adc3)
    {
      GPIO_write_low(&PORTB, LED_1);
      GPIO_write_low(&PORTB, LED_2);
      GPIO_write_high(&PORTB, LED_3);
      GPIO_write_low(&PORTB, LED_4);
    }
    else if (max == adc4)
    {
      GPIO_write_low(&PORTB, LED_1);
      GPIO_write_low(&PORTB, LED_2);
      GPIO_write_low(&PORTB, LED_3);
      GPIO_write_high(&PORTB, LED_4);
    }
    // ####################################

    // proudovy sensor
    uint16_t proud_analog = analogRead(A0);
    // ################

    // vypocet proudu
    uint16_t napeti = 0;
    uint16_t proud = 0;
    uint16_t soucet = 0;
    // provedení stovky měření pro ustálení výsledku
    for (int i = 0; i < 100; i++)
    {
      // načtení hodnoty analogového vstupu
      proud_analog = analogRead(A0);
      // přepočet napětí na proud dle informací od výrobce
      napeti = (proud_analog * 5000.0) / 1023.0;
      proud = (napeti - offset) / konstanta;
      // uložení výsledku pro následné zprůměrování
      soucet += proud;
      delay(10);
    }

    // konecna hodnota proudu
    proud = soucet/100;

    _delay_ms(1000);
  }
}
