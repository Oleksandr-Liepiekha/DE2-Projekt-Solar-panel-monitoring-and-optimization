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
/*#define LED_GREEN PB5 // AVR pin where green LED is connected
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
/*#include <avr/io.h>     // AVR device-specific IO definitions
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
/*int main(void)
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
*/

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
//#define LED_GREEN PB5 // AVR pin where green LED is connected
//#define LED_1 PB1
//#define LED_2 PB2
//#define LED_3 PB3
//#define LED_4 PB4
#define BUTTON PD2
#define SHORT_DELAY 250 // Delay in milliseconds
#ifndef F_CPU
#define F_CPU 16000000 // CPU frequency in Hz required for delay
#endif

/* Includes -----------------------------------------------*/
#include <avr/io.h>     // AVR device-specific IO definitions
#include <util/delay.h> // Functions for busy-wait delay loops
#include <avr/interrupt.h>
#include <gpio.h>
#include <timer.h>
#include <adc.h>
#include <oled.h>
#include <uart.h>
#include <stdlib.h> // Pro itoa()
#include <stdio.h>

/* Function definitions -----------------------------------*/
/***********************************************************
 * Function: Main function where the program execution begins
 * Purpose:  Toggle one LED and use delay library.
 * Returns:  none
 ***********************************************************/
void setupPWM_50Hz()
{
  // Set Pin 9 (OC1A) as output
  DDRB |= (1 << PB1);

  // Configure Timer1 for Fast PWM mode, with ICR1 as TOP
  TCCR1A = (1 << WGM11) | (1 << COM1A1);              // Fast PWM, non-inverting mode on OC1A
  TCCR1B = (1 << WGM12) | (1 << WGM13) | (1 << CS11); // Prescaler = 8, Fast PWM mode

  // Set TOP value to get a 50Hz frequency
  ICR1 = 39999; // 50Hz period (20ms)

  // Set initial pulse width to 1ms (0 position)
   // 1ms pulse width corresponds to 0 degrees
}

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
  oled_puts("        "); // Vymazání staré hodnoty
  oled_gotoxy(12, 4);
  itoa(rotation, buffer, 10);
  oled_puts(buffer);
  oled_puts(" deg");

  oled_display();
}

int main(void)
{

  /*GPIO_mode_output(&DDRB, LED_GREEN);
  GPIO_mode_output(&DDRB, LED_1);
  GPIO_mode_output(&DDRB, LED_2);
  GPIO_mode_output(&DDRB, LED_3);
  GPIO_mode_output(&DDRB, LED_4);
  GPIO_mode_input_pullup(&DDRD, BUTTON);*/

  uart_init(UART_BAUD_SELECT(9600, F_CPU));

  TIM1_ovf_enable();
  TIM1_ovf_1sec();

  initADC(AVCC);

  /*GPIO_write_high(&PORTB, LED_1);
  GPIO_write_low(&PORTB, LED_2);
  GPIO_write_high(&PORTB, LED_3);
  GPIO_write_low(&PORTB, LED_4);*/

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

  //uint16_t rotation = 0;
  //sei();
  /*uart_puts("\r\n");  // New line only
  uart_puts("Start using Serial monitor...\r\n");*/

  uint16_t rotation = 0;

    // konstanty pro proud
  uint16_t konstanta = 185;
  // proměnná pro nastavení offsetu, polovina Vcc
  uint16_t offset = 2500;
  // vypocet proudu
  uint16_t napeti = 0;
  uint16_t proud = 0;
  uint16_t soucet = 0;
  // provedení stovky měření pro ustálení výsledku


  // Set initial pulse width to 1ms (0 position)
  OCR1A = 1101; // 1ms pulse width corresponds to 0 degrees

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

  // Proměnná pro rotaci

  if (adc2 > max)
  {
    max = adc2;
    rotation = 60;
  }
  if (adc3 > max)
  {
    max = adc3;
    rotation = 120;
  }
  if (adc4 > max)
  {
    max = adc4;
    rotation = 180;
  }
  if (max == adc1)
  {
    rotation = 0;
  }

  // #########################

  // nejvetsi napeti - sviti ledka (jen test)
  if (max == adc1)
  {
    /*GPIO_write_high(&PORTB, LED_1);
    GPIO_write_low(&PORTB, LED_2);
    GPIO_write_low(&PORTB, LED_3);
    GPIO_write_low(&PORTB, LED_4);*/
    // Set PWM for position 1 (0 degrees = 0.553ms pulse) ; OCR1A = (0.553ms / 20ms) * 39999 = 1106
    OCR1A = 1106;
  }
  else if (max == adc2)
  {
   /*GPIO_write_low(&PORTB, LED_1);
    GPIO_write_high(&PORTB, LED_2);
    GPIO_write_low(&PORTB, LED_3);
    GPIO_write_low(&PORTB, LED_4);*/
    // Set PWM for position 2 (60 degrees = 1.177ms pulse) ; OCR1A = (1.177ms / 20ms) * 39999 = 2354
    OCR1A = 2354;
  }
  else if (max == adc3)
  {
    /*GPIO_write_low(&PORTB, LED_1);
    GPIO_write_low(&PORTB, LED_2);
    GPIO_write_high(&PORTB, LED_3);
    GPIO_write_low(&PORTB, LED_4);*/
    // Set PWM for position 3 (120 degrees = 1.801ms pulse) ; OCR1A = (1.801ms / 20ms) * 39999 = 3602
    OCR1A = 3602;
  }
  else if (max == adc4)
  {
    /*GPIO_write_low(&PORTB, LED_1);
    GPIO_write_low(&PORTB, LED_2);
    GPIO_write_low(&PORTB, LED_3);
    GPIO_write_high(&PORTB, LED_4);*/
    // Set PWM for position 4 (180 degrees = 2.425ms pulse) ; OCR1A = (2.425ms / 20ms) * 39999 = 4850
    OCR1A = 4850;
  }
  // ####################################

  // proudovy sensor
  uint16_t proud_analog = analogRead(A0);
  // ################

  for (int i = 0; i < 11; i++)
  {
    // načtení hodnoty analogového vstupu
    proud_analog = analogRead(A0);
    // přepočet napětí na proud dle informací od výrobce
    napeti = (proud_analog * 5000.0) / 1023.0;
    proud = (napeti - offset) / konstanta;
    // uložení výsledku pro následné zprůměrování
    soucet += proud;
    _delay_ms(50);
  }

  // konecna hodnota proudu
  proud = soucet / 10;

  setupPWM_50Hz();

  oled_update(proud, rotation);

  static char buffer[33];

  itoa(adc1, buffer, 10);
  itoa(adc2, buffer, 10);
  itoa(adc3, buffer, 10);
  itoa(adc4, buffer, 10);


  /*uart_puts("adc1 ");
  uart_putc(adc1);
  uart_puts("; adc2 ");
  uart_putc(adc2);
  uart_puts("; adc3 ");
  uart_putc(adc3);
  uart_puts("; adc4 ");
  uart_putc(adc4);
  uart_puts("; Position ");
  uart_putc(rotation);
  uart_puts("\n");*/


  _delay_ms(5000);
  }
}
