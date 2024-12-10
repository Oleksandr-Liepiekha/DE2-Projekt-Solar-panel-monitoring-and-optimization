# Project: Solar panel monitoring and optimization
____
## Team members
* Oleksandr Liepiekha (responsible for : current sensor, photoresistors )

* Ondřej Hrozek (responsible for: PWM signal, photoresistors, GitHub )

* Ivan Hinak (responsible for : OLED display, wiring diagram, GitHub )




## Parts used:

Servo motor:
![Servo motor](https://github.com/Oleksandr-Liepiekha/DE2-Projekt-Solar-panel-monitoring-and-optimization/blob/main/Images/Servo%20Motor.jpg?raw=true)

Oled display:
![Oled display](https://github.com/Oleksandr-Liepiekha/DE2-Projekt-Solar-panel-monitoring-and-optimization/blob/main/Images/Oled%20Display.jpg?raw=true)

Current sensor:
![Current sensor](https://github.com/Oleksandr-Liepiekha/DE2-Projekt-Solar-panel-monitoring-and-optimization/blob/main/Images/Current%20Sensor.jpg?raw=true)

Solar panel:
![Solar panel](https://github.com/Oleksandr-Liepiekha/DE2-Projekt-Solar-panel-monitoring-and-optimization/blob/main/Images/Solar%20Panel.jpg?raw=true)

Baterry:
![Baterry](https://github.com/Oleksandr-Liepiekha/DE2-Projekt-Solar-panel-monitoring-and-optimization/blob/main/Images/Battery.jpg?raw=true)

Arduino Uno used:
![Arduino Uno](https://github.com/Oleksandr-Liepiekha/DE2-Projekt-Solar-panel-monitoring-and-optimization/blob/main/Images/Arduino%20Uno.jpg?raw=true)


## Theoretical description and explanation
Our setup includes two circuits.

The first circuit uses four photoresistors connected to analog pins A1 to A4. Their resistance values are compared to determine the direction of light. Based on the smallest resistance value, the servo motor rotates the solar panel to one of four positions: 0° (initial position), 60°, 120°, or 180°. We measure the voltage across four voltage dividers, each consisting of a resistor and a photoresistor in series. The program outputs the resistance values of the photoresistors to the serial monitor.

The second circuit comprises a solar panel, a discharged rechargeable battery, and a current sensor that measures the battery's charging current. The servo motor's rotation and the charging current are displayed on an OLED screen.

## Wiring diagram
![Wiring diagram](https://github.com/user-attachments/assets/8c84cf52-3058-40d2-893a-458177c35be2)

## Codes
 ```c
   DE2_Project       
   ├── include         
   │   └── timer.h
   ├── lib          
   │   ├── adc          
   │   │   ├── adc.c
   │   │   └── adc.h
   │   ├── gpio           
   │   │   ├── gpio.c
   │   │   └── gpio.h
   │   ├── oled        
   │   │   ├── oled.h
   │   │   ├── oled.c
   │   │   └── font.h
   │   ├── twi           
   │   │   ├── twi.c
   │   │   └── twi.h
   │   └── uart           
   │       ├── uart.c
   │       └── uart.h
   ├── src            
   │   └── main.c
   ├── test          
   └── platformio.ini 
 ```



Our codes:

* OLED display
``````c
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
  ``````

* PWM signal:
``````c
  #include <avr/io.h>

void setupPWM_50Hz() {
    // Set Pin 9 (OC1A) as output
    DDRB |= (1 << PB1);
    
    // Configure Timer1 for Fast PWM mode, with ICR1 as TOP
    TCCR1A = (1 << WGM11) | (1 << COM1A1);  // Fast PWM, non-inverting mode on OC1A
    TCCR1B = (1 << WGM12) | (1 << WGM13) | (1 << CS11); // Prescaler = 8, Fast PWM mode
    
    // Set TOP value to get a 50Hz frequency
    // TOP = f_CPU / (Prescaler * Frequency) - 1
    // TOP = 16,000,000 / (8 * 50) - 1 = 39999
    ICR1 = 39999; // 50Hz period (20ms)
    
    // Set initial pulse width to 1ms
    // Pulse width = (OCR1A / TOP) * Period
    // OCR1A = (Pulse width / Period) * TOP

    // 1 position (0deg) OCR1A = (0,553ms / 20ms) * 39999 = 2000
    OCR1A = 1101;

    // 2 position (45deg) OCR1A = (1,021ms / 20ms) * 39999 = 2000
    //OCR1A = 2041;

    ...

     // 5 position  (180deg)OCR1A = (2,425ms / 20ms) * 39999 = 2000
    //OCR1A = 4851;
}

int main() {
    setupPWM_50Hz();
    
    while (1) {
        // The PWM signal runs continuously on pin 9 (OC1A).
    }
}

  ``````
* Main function

``````c
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
  ``````
## Results
* Short video with demonstration :https://youtu.be/1KhJNU5B0GU

## References
* VHDLwhiz: https://vhdlwhiz.com/product/vhdl-rc-servo-controller-using-pwm/
* Data sheet Current Sensor:https://navody.dratek.cz/navody-k-produktum/proudovy-senzor-acs712.html?fbclid=IwZXh0bgNhZW0CMTEAAR0c4ihuDLEantfFjd1U-c4-nEBtAvyrMEvjdgQ5THMz3d6KYHg4gV08uSc_aem_rM2CCd-NV4MII8P_j-1wNA
* Data sheet PWM:https://www.servocity.com/hs-485hb-servo/?fbclid=IwZXh0bgNhZW0CMTEAAR3aT5pAt8PamfAFMd18X1G4DhTAtVN_wJIVZQ8zXBtihQTZBjNh6JMiDZ8_aem_SmhI9yxwlVIMlfroiNAyUw
* Inspiration analogread:
