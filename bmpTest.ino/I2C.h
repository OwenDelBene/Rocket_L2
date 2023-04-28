
#ifndef I2C_H
#define I2C_H

#include <stdint.h>


bool readBytes(uint8_t slave_addr, uint8_t reg, uint8_t* data, size_t numBytes )
{

    Wire.write(reg);
    Wire.endTransmission();
    Wire.requestFrom(slave_addr, numBytes);
    delay(200);
    int i=0;
    
    while(Wire.available())
    {
      //Serial.println("wire is available");
        data[i++] = Wire.read();
    }
   // Serial.print(numBytes);
   // Serial.println(" total bytes read: ");
    for (int j=0; j<numBytes; j++)
    {
   //   Serial.println(data[j]);
    }
    
    return i==numBytes;
}

bool readByte(uint8_t slave_addr, uint8_t reg, uint8_t* data)
{

    Wire.write(reg);
    Wire.endTransmission();
    Wire.requestFrom(slave_addr, 1);
    int i=0;
    while(Wire.available())
    {
     // Serial.println("wire is available");
        (*data) = Wire.read();
        i++;
    }

    return i==1;
}


bool writeBytes(uint8_t reg, uint8_t* data, size_t numBytes)
{
    size_t bytesWritten = 0;
    bytesWritten += Wire.write(reg);
    bytesWritten += Wire.write(data,numBytes);
    Wire.endTransmission();
    return bytesWritten == (numBytes + 1);
}

bool writeByte(uint8_t reg, uint8_t data)
{
    size_t bytesWritten = 0;
    bytesWritten += Wire.write(reg);
    bytesWritten += Wire.write(data);
    Wire.endTransmission();
    return bytesWritten==2;
}






#endif
