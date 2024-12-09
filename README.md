# Project: Solar panel monitoring and optimization

____
Parts used:

-fotky koponentů

Arduino Uno used:

![
## Team members
* Oleksandr Liepiekha (responsible for : current sensor, photoresistors )

* Ondřej Hrozek (responsible for: PWM signal, photoresistors, GitHub )

* Ivan Hinak (responsible for : OLED display, wiring diagram, GitHub )

## Theoretical description and explanation
___
-popis toho co děláme
## Hardware description of demo application
![Wiring diagram](https://github.com/user-attachments/assets/8c84cf52-3058-40d2-893a-458177c35be2)
-schéma
## Software description
___
-jak fungují naše kódy

Libraries used:

-link na knihovny

Links to our codes:

-link na kódy

* OLED display

* Photoresistors

* Current sensor

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


## Results
* Short video with demonstration :https://youtu.be/1KhJNU5B0GU
  
-video a fotky
## References

manuály
