#include <avr/io.h>
#include <avr/interrupt.h>
#include "timer.h"
#include <twi.h>
#include <oled.h>
#include <stdlib.h>

volatile uint16_t last_adc_value = 0;

void uart_init(uint16_t ubrr) {
    UBRR0H = (uint8_t)(ubrr >> 8);
    UBRR0L = (uint8_t)ubrr;
    UCSR0B = (1 << TXEN0);
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

void uart_transmit(uint8_t data) {
    while (!(UCSR0A & (1 << UDRE0)));
    UDR0 = data;
}

void uart_print_string(const char *str) {
    while (*str) {
        uart_transmit(*str++);
    }
}

int main(void)
{
    oled_init(OLED_DISP_ON);
    oled_clrscr();

    oled_charMode(DOUBLESIZE);
    oled_puts("  PROJEKT");

    oled_charMode(NORMALSIZE);

    oled_drawLine(0, 15, 120, 15, WHITE);

    oled_gotoxy(1, 3);
    oled_puts("Current:");

    oled_gotoxy(14, 3);
    oled_puts("A");

    oled_gotoxy(1, 4);
    oled_puts("Rotation:");

    oled_gotoxy(14, 4);
    oled_puts("degrees");

    oled_display();

    // Configure ADC
    ADMUX |= (1 << REFS0);  // AVcc as reference
    ADMUX &= ~(1 << MUX3 | 1 << MUX2 | 1 << MUX1 | 1 << MUX0);  // ADC0
    ADCSRA |= (1 << ADEN) | (1 << ADIE) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);  // Enable ADC with interrupt


    while (1) {
        ;
    }

    return 0;
}

ISR(ADC_vect)
{
    uint16_t value = ADC;


}

ISR(TIMER1_OVF_vect)
{
    static uint8_t n_ovfs = 0;
    static uint8_t print_flag = 0;

    n_ovfs++;
    if (n_ovfs >= 20) {  // 1 second interval (assuming 1 overflow every 16ms)
        n_ovfs = 0;
        ADCSRA |= (1 << ADSC);  // Start ADC conversion

        print_flag = 1;  // Set the flag to print the data
    }

    if (print_flag) {
        // Print ADC value and PWM duty once every second
        char string[5];  // Increase buffer size for ADC values and PWM duty
        itoa(last_adc_value, string, 10);
        uart_print_string("ADC: ");
        uart_print_string(string);
        uart_print_string(" ");

        print_flag = 0;  // Reset the flag after printing
    }
}
