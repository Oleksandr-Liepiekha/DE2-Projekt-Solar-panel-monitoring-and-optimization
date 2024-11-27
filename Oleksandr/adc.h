#ifndef __ADCAVR_H__
#define __ADCAVR_H__

#define _ADC_IRQ_EN // comment this line if you want to work without ADC interrupt

#include <avr/io.h>
#include <inttypes.h>

//#ifdef _ADC_IRQ_EN
//	#include <avr/interrupt.h>
//#endif



#if F_CPU <= 1000000UL
#define ADC_PRSC 3
#elif F_CPU <= 4000000UL
#define ADC_PRSC 5
#elif F_CPU <= 8000000UL
#define ADC_PRSC 6
#else
#define ADC_PRSC 7
#endif

#define BIT_SET(ADDRESS,BIT) (ADDRESS |= (1<<BIT))
#define BIT_CLEAR(ADDRESS,BIT) (ADDRESS &= ~(1<<BIT))
#define BIT_CHECK(ADDRESS,BIT) (ADDRESS & (1<<BIT))
#define BIT_FLIP(ADDRESS,BIT) (ADDRESS ^= (1<<BIT))

typedef enum{
	A0 = 				0x00,
	A1 = 				0x01,
	A2 = 				0x02,
	A3 = 				0x03,
	A4 = 				0x04,
	A5 = 				0x05,
	A6 = 				0x06,
	A7 = 				0x07,
	INT_TEMP_SENSOR  =  0x08,
	BANG_GAP_VOLATGE =	0x0E,
	ANALOG_GND		 =	0x0F
} ANALOG_CHANNEL;

typedef enum{
	AREF = 			0x00,
	AVCC =			0x01,
	AN_RESERVED =	0x02,
	INTERNAL_1_1 =	0x03,
} ANALOG_REF;

#ifdef _ADC_IRQ_EN

	volatile void (*fun_ptr)(void);
	void start_conversion();
	void ADC_attach_interrupt(void* f_ptr);
	void ADC_IRQ_disable();
	
#endif

void initADC(ANALOG_REF ref);
uint8_t setAnalogReference(ANALOG_REF ref);
uint16_t analogRead(ANALOG_CHANNEL adcCh);
float analogReadTemp(void);

#endif
