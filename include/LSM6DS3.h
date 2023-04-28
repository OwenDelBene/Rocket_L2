
#include "I2C.h"
//Noise in 16g mode 180 mug/sqrt(hz)
// <1.25mA

#define WHO_AM_I_REG  0x0F
#define WHO_AM_I 0x69
#define LSM6DS3_SLAVE_ADDR 0x6A   

#define LSM6DS3_ACCEL_ODR_SCALE_BANDWIDTH 0x10
#define LSM6DS3_ACCEL_FILTER_SELECTION 0x17
#define LSM6DS3_ACCEL_OUTPUT_ENABLE 0x18

#define LSM6DS3_TEMP_OUTPUT_LOWER 0x20
#define LSM6DS3_TEMP_OUTPUT_UPPER 0x21

#define LSM6DS3_ACCEL_OUTPUT_X_LOWER 0x28
#define LSM6DS3_ACCEL_OUTPUT_X_HIGHER 0x29
#define LSM6DS3_ACCEL_OUTPUT_Y_LOWER 0x2A
#define LSM6DS3_ACCEL_OUTPUT_Y_HIGHER 0x2B
#define LSM6DS3_ACCEL_OUTPUT_Z_LOWER 0x2C
#define LSM6DS3_ACCEL_OUTPUT_D_HIGHER 0x2D

#define LSM6DS3_FIFO_DATA_OUT_LOWER 0x3E
#define LSM6DS3_FIFO_DATA_OUT_LOWER 0x3F




void accelerometerInit(uint8_t odr = 10, uint8_t scale =3)
{
    Wire.beginTransmission(LSM6DS3_SLAVE_ADDR);
    uint8_t data  = (ODR << 4) | (scale << 2);
    writeBytes(LSM6DS3_ACCEL_ODR_SCALE_BANDWIDTH, &data, 1);

    writeByte(LSM6DS3_ACCEL_OUTPUT_ENABLE, 56);
}


bool accelerometerTest()
{
    uint8_t whoAmI;
    Wire.beginTransmission(LSM6DS3_SLAVE_ADDR);
    readByte(LSM6DS3_SLAVE_ADDR, WHO_AM_I_REG, &whoAmI);
    return whoAmI == WHO_AM_I; 
}


void accelX(uint16_t* ax)
{
    uint8_t LSB, MSB;
    Wire.beginTransmission(LSM6DS3_SLAVE_ADDR);
    readByte(LSM6DS3_SLAVE_ADDR, LSM6DS3_ACCEL_OUTPUT_X_HIGHER, &MSB);
    readByte(LSM6DS3_SLAVE_ADDR, LSM6DS3_ACCEL_OUTPUT_X_LOWER, &LSB);
    (*ax) = (MSB << 8) | LSB; 
}

void accelY(uint16_t* ay)
{
    uint8_t LSB, MSB;
    Wire.beginTransmission(LSM6DS3_SLAVE_ADDR);
    readByte(LSM6DS3_SLAVE_ADDR, LSM6DS3_ACCEL_OUTPUT_Y_HIGHER, &MSB);
    readByte(LSM6DS3_SLAVE_ADDR, LSM6DS3_ACCEL_OUTPUT_Y_LOWER, &LSB);
    (*ay) = (MSB << 8) | LSB; 
}

void accelZ(uint16_t* az)
{
    uint8_t LSB, MSB;
    Wire.beginTransmission(LSM6DS3_SLAVE_ADDR);
    readByte(LSM6DS3_SLAVE_ADDR, LSM6DS3_ACCEL_OUTPUT_Z_HIGHER, &MSB);
    readByte(LSM6DS3_SLAVE_ADDR, LSM6DS3_ACCEL_OUTPUT_Z_LOWER, &LSB);
    (*az) = (MSB << 8) | LSB; 
}


void temperature(uint16_t* temp)
{
    uint8_t LSB, MSB;
    Wire.beginTransmission(LSM6DS3_SLAVE_ADDR);
    readByte(LSM6DS3_SLAVE_ADDR, LSM6DS3_TEMP_OUTPUT_HIGHER, &MSB);
    readByte(LSM6DS3_SLAVE_ADDR, LSM6DS3_TEMP_OUTPUT_LOWER, &LSB);

    (*temp) = (MSB << 8) | LSB; 
}