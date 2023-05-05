#include <SPI.h>

bool SPI_writeBytes(int ss, uint8_t reg, uint8_t* data, int len)
{

    int bytesWritten = 0;
    PORTB &= ~(1 <<PORTB2) //Slave select LOW
    SPI.transfer(reg);
    for (int i=0; i<len; i++)
    {   
        SPI.transfer(data[i]);
    }


    PORTB |= (1 << PORTB2); //Slave select HIGH
}



bool SPI_readBytes()
