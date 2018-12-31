
#include "grbl.h"

// uint8_t analog_reference = DEFAULT;

void analog_init(void)
{
	//enable the analog converter, set the analog reference, prescaler, and start the first conversion
	ADCSRA = (1 << ADEN) | (1 << ADSC) | (1 << ADPS2);
}

int analog_read(uint8_t pin)
{
	uint8_t low, high;
	ADMUX = (pin & 0x07);

	// start the conversion
	sbi(ADCSRA, ADSC);

	// ADSC is cleared when the conversion finishes
	while (bit_is_set(ADCSRA, ADSC));

	// we have to read ADCL first; doing so locks both ADCL
	// and ADCH until ADCH is read.  reading ADCL second would
	// cause the results of each conversion to be discarded,
	// as ADCL and ADCH would be locked when it completed.
	low  = ADCL;
	high = ADCH;
	// combine the two bytes
	return (high << 8) | low;

}