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
#include <oled.h>
#include <stdlib.h> // Pro itoa()

/* Function definitions -----------------------------------*/
/***********************************************************
 * Function: Main function where the program execution begins
 * Purpose:  Toggle one LED and use delay library.
 * Returns:  none
 ***********************************************************/


void oled_update(uint16_t proud, uint16_t rotation)
{
    char buffer[10];

    // Proud
    oled_gotoxy(12, 3);
    itoa(proud, buffer, 10);
    oled_puts(buffer);
    oled_puts(" mA");

    // Rotace
    oled_gotoxy(12, 4);
    oled_puts("             "); // Vymazání staré hodnoty
    itoa(rotation, buffer, 10);
    oled_puts(buffer);
    oled_puts(" degrees");

    oled_display();
}

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

  TIM1_ovf_1sec();
  TIM1_ovf_enable();
    
  initADC(AVCC);

  GPIO_write_high(&PORTB, LED_1);
  GPIO_write_low(&PORTB, LED_2);
  GPIO_write_high(&PORTB, LED_3);

  oled_init(OLED_DISP_ON);
  oled_clrscr();
  oled_charMode(DOUBLESIZE);
  oled_puts("  PROJEKT");
  oled_charMode(NORMALSIZE);
  oled_drawLine(0, 15, 120, 15, WHITE);
  oled_gotoxy(1, 3);
  oled_puts("Current:");
  oled_gotoxy(1, 4);
  oled_puts("Rotation:");
  oled_display();

  while (1)
  {
    // na photoresistory
    uint16_t adc1 = analogRead(A1);
    uint16_t adc2 = analogRead(A2);
    uint16_t adc3 = analogRead(A3);
    uint16_t adc4 = analogRead(A4);
    // ###############

    // Najít maximální hodnotu
    uint16_t max = adc1;
    uint16_t rotation = 0; // Proměnná pro rotaci

    if (adc2 > max)
    {
        max = adc2;
        rotation = 90;
    }
    if (adc3 > max)
    {
        max = adc3;
        rotation = 180;
    }
    if (adc4 > max)
    {
        max = adc4;
        rotation = 270;
    }
    if (max == adc1)
    {
        rotation = 0;
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
      _delay_ms(5000);
    }

    // konecna hodnota proudu
    proud = soucet/100;

    oled_update(proud, rotation);

    _delay_ms(1000);
  }
}
