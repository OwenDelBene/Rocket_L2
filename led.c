#include <avr/io.h>
#include <util/delay.h>

int main(void)
{
	//set PortB5 as an output using direct digial register
	DDRB |= (1 << DDB5);
	
	while(1)
	{
		//Set portB high
		PORTB |= (1 << PORTB5);
		
		//Macro from delay.h, wait in miliseconds
		//Calculates required no-operations for requested delay based
		// on clock speed
		_delay_ms(1000);


		//Set PortB low
		PORTB &= ~(1 << PORTB5); 

		_delay_ms(1000);
	}	

	
}


//Makefile notes: 
// Compile with gcc, optmize for speed, set clock speed to 16MHZ, microcontroller type =atmega328p, only compile, output led.o led.c
// Go through full linking process, produce led.bin from led.o 
// Removs .eeprom from led.bin and outputing format to intelhex format to led.hex 
// Use arduino process on ATMega328P at the usb path, with serial baude rate, source of data: to flash writingable memory led.hex
//
