
#include "adc.h"

#ifdef _ADC_IRQ_EN
	void start_conversion()
	{
		ADCSRA |= (1<<ADSC);
	}

	void ADC_attach_interrupt(void* f_ptr)
	{
		ADCSRA |= (1<<ADIE);// enable interrupt
		fun_ptr = f_ptr;
	}

	void ADC_IRQ_disable()
	{
		ADCSRA &= ~(1<<ADIE);
	}
#endif

void initADC(ANALOG_REF ref) {
	BIT_CLEAR(ADMUX, ADLAR);
	ADCSRA |= (1<<ADEN); //enable ADC
	ADCSRA |= (ADC_PRSC<<ADPS0); // set prescaler 
	setAnalogReference(ref);
}

uint8_t setAnalogReference(ANALOG_REF ref){
	if (ref > 0x03 || ref == AN_RESERVED){
		return 0;
	}
	else {
		ADMUX |= (ref<<REFS0);
		return 1;
	}
}

uint16_t analogRead(ANALOG_CHANNEL adcCh){
	
	ADMUX  &= ~(0x0f << MUX0);
	ADMUX |= (adcCh & 0x0f);
	ADCSRA |= (1<<ADSC); //start conversion
	while (ADCSRA & (1<<ADSC));
	return ADC;
}

float analogReadTemp(void)
{
	uint8_t prevADMUX = ADMUX;
	ADMUX |= (AVCC<<REFS0); // set internal 1.1V reference
	uint16_t volatile adc = analogRead(INT_TEMP_SENSOR);
	ADMUX = prevADMUX;
	return (adc - 324.31)/1.22;

}
//#ifdef _ADC_IRQ_EN
//ISR(ADC_vect)
//{
//	fun_ptr();
//}
//#endif
