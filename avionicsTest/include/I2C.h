
#ifndef I2C_H
#define I2C_H

#include <stdint.h>

//This library is intended to be used with arduino's wire library.


/**
 * @brief reads the value returned from a specified register on a slave address
 * 
 * @param slave_addr 7bit slave address of i2c device
 * @param reg register on the slave address
 * @param data pointer to place to store data
 * @param numBytes number of bytes requested
 * @return true If successful
 * @return false If unsuccessful
 */
bool readBytes(uint8_t slave_addr, uint8_t reg, uint8_t* data, uint8_t numBytes )
{
    Wire.beginTransmission(slave_addr);
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

/**
 * @brief reads the value returned from a specified register on a slave address
 * 
 * @param slave_addr 7bit slave address of i2c device
 * @param reg register on the slave address
 * @param data pointer to place to store data
 * @return true If successful
 * @return false If unsuccessful
 */

bool readByte(uint8_t slaveAddr, uint8_t reg, uint8_t* data)
{
    Wire.beginTransmission(slaveAddr);
    Wire.write(reg);
    Wire.endTransmission();
    
    Wire.requestFrom(slaveAddr,(uint8_t) 1);
    int i=0;
    while(Wire.available())
    {
     // Serial.println("wire is available");
        (*data) = Wire.read();
        i++;
    }

    return i==1;
}

/**
 * @brief Writes data to slave device
 * 
 * @param slaveAddr 7 bit slave address
 * @param reg register on slave to write to
 * @param data byte array containing data to be written
 * @param numBytes number of bytes to be written
 * @return true if successful
 * @return false if unsuccessful
 */
bool writeBytes(uint8_t slaveAddr, uint8_t reg, uint8_t* data, size_t numBytes)
{
    Wire.beginTransmission(slaveAddr);
    size_t bytesWritten = 0;
    bytesWritten += Wire.write(reg);
    bytesWritten += Wire.write(data,numBytes);
    if (bytesWritten !=numBytes+1){
    Serial.print("Wanted to write ");
    Serial.print(numBytes+1);
    Serial.print(" instead wrote: ");
    Serial.print(bytesWritten);
    }
    Wire.endTransmission();
    return bytesWritten == (numBytes + 1);
}

/**
 * @brief Writes data to slave device
 * 
 * @param slaveAddr 7 bit slave address
 * @param reg register on slave to write to
 * @param data byte containing data to be written
 * @return true if successful
 * @return false if unsuccessful
 */
bool writeByte(uint8_t slaveAddr, uint8_t reg, uint8_t data)
{
    Wire.beginTransmission(slaveAddr);
    size_t bytesWritten = 0;
    bytesWritten += Wire.write(reg);
    bytesWritten += Wire.write(data);
    if (bytesWritten !=2)
    {
      Serial.print("Wanted to write 2 bytes instead wrote: ");
      Serial.print(bytesWritten);
    }
    Wire.endTransmission();
    return bytesWritten==2;
}






#endif
