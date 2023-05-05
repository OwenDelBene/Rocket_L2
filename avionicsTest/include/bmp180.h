//3.6V max
//SCL 3.4MHz
//Need pullup resistor 4.7kOhm
#include "I2C.h"


#ifndef BMP180_H
#define BMP180_H



#define BMP180_CALIB_DATA_SIZE        (22)
#define BMP180_calibration_params_AC1_MSB    (0)
#define BMP180_calibration_params_AC1_LSB    (1)
#define BMP180_calibration_params_AC2_MSB    (2)
#define BMP180_calibration_params_AC2_LSB    (3)
#define BMP180_calibration_params_AC3_MSB    (4)
#define BMP180_calibration_params_AC3_LSB    (5)
#define BMP180_calibration_params_AC4_MSB    (6)
#define BMP180_calibration_params_AC4_LSB    (7)
#define BMP180_calibration_params_AC5_MSB    (8)
#define BMP180_calibration_params_AC5_LSB    (9)
#define BMP180_calibration_params_AC6_MSB    (10)
#define BMP180_calibration_params_AC6_LSB    (11)
#define BMP180_calibration_params_B1_MSB     (12)
#define BMP180_calibration_params_B1_LSB     (13)
#define BMP180_calibration_params_B2_MSB     (14)
#define BMP180_calibration_params_B2_LSB     (15)
#define BMP180_calibration_params_MB_MSB     (16)
#define BMP180_calibration_params_MB_LSB     (17)
#define BMP180_calibration_params_MC_MSB     (18)
#define BMP180_calibration_params_MC_LSB     (19)
#define BMP180_calibration_params_MD_MSB     (20)
#define BMP180_calibration_params_MD_LSB     (21)


#define BMP180_SLAVE_ADDR (0xEE >> 1)
#define BMP180_OUT_XLSP 0xF8
#define BMP180_OUT_LSP 0xF7
#define BMP180_OUT_MSP 0xF6
#define BMP180_CTRL_MEASUREMENT_REG 0xF4
#define BMP180_RESET 0xE0
#define BMP180_CHIP_ID_REG 0xD0
#define BMP180_TEMPERATURE_CTRL 0x2E
#define BMP180_PRESSURE_CTRL 0xF4 //output sampling rate 3 for lower output sampling rate 0xB4, 0x74, and 0x34 for the lowest
#define BMP180_EEPROM_START_ADDR 0xAA
#define BMP180_EEPROM_DATA_LEN 22
#define BMP180_VERSION_REG 0xD1




#define BMP180_GET_BITSLICE(regvar, bitname) \
    ((regvar & bitname##__MSK) >> (bitname##__POS))
#define BMP180_SET_BITSLICE(regvar, bitname, val) \
    ((regvar & ~bitname##__MSK) | ((val << bitname##__POS) & bitname##__MSK))
#define BMP180_CHIP_ID__POS (0)
#define BMP180_CHIP_ID__MSK (0xFF)
#define BMP180_CHIP_ID__LEN (8)
#define BMP180_CHIP_ID__REG (BMP180_CHIP_ID_REG)

#define BMP180_ML_VERSION__POS (0)
#define BMP180_ML_VERSION__LEN (4)
#define BMP180_ML_VERSION__MSK (0x0F)
#define BMP180_ML_VERSION__REG (BMP180_VERSION_REG)

#define BMP180_AL_VERSION__POS (4)
#define BMP180_AL_VERSION__LEN (4)
#define BMP180_AL_VERSION__MSK (0xF0)
#define BMP180_AL_VERSION__REG (BMP180_VERSION_REG)


#define BMP180_ERROR 255

/**
 * @brief Container for calibration parameters
 * 
 */
struct BMP180_calibration
{
    uint16_t ac1;
    uint16_t ac2;
    uint16_t ac3;
    uint16_t ac4;
    uint16_t ac5;
    uint16_t ac6;
    uint16_t b1;
    uint16_t b2;
    uint16_t mb;
    uint16_t mc;
    uint16_t md;
};

struct BMP180
{
    struct BMP180_calibration calibration_params;
    uint8_t mode; //power mode
    uint8_t chip_id; //used for testing
    uint8_t device_addr; 
    uint8_t slave_addr = BMP180_SLAVE_ADDR;
    
    uint8_t ml_version; /**<ml version*/
    uint8_t al_version; /**<al version*/

    uint8_t sensortype; /**< sensor type*/
    int32_t param_b5; /**<pram*/
    int32_t number_of_samples; /**<sample calculation*/
    int16_t oversamp_setting; /**<oversampling setting*/
    int16_t sw_oversamp; /**<software oversampling*/


    uint8_t oss;


    double Pressure0 = 1013.25; //hPa 






    /**
     * @brief Computes B5 calibration parameter, taken from datasheet
     * 
     * @param calCoeff Calibration coefficient struct
     * @param ut uncalibrated temperature
     * @return int32_t B5 calibration parameter
     */
    int32_t computeB5(BMP180_calibration* calCoeff ,int32_t ut)
    {
        int32_t x1 = (((ut - (int32_t)calCoeff->ac6) * (int32_t)calCoeff->ac5) >> 15 );
        int32_t x2= ((int32_t)calCoeff->mc << 11) / (x1 + (int32_t)calCoeff->md);
        return x1+x2;
    }

    /**
     * @brief Populate the calibration Parameter structure
     * 
     */
    void getCalibrationParameters()
    {

        
        //Check chip ID to make sure

        uint8_t calibData[BMP180_EEPROM_DATA_LEN];
        readBytes(this->slave_addr,BMP180_EEPROM_START_ADDR, calibData, BMP180_EEPROM_DATA_LEN );

        this->calibration_params.ac1 =(int16_t)((((int32_t)((int8_t)calibData[BMP180_calibration_params_AC1_MSB])) << 8) |
                calibData[BMP180_calibration_params_AC1_LSB]);

        this->calibration_params.ac2 =
            (int16_t)((((int32_t)((int8_t)calibData[BMP180_calibration_params_AC2_MSB])) << 8) |
                calibData[BMP180_calibration_params_AC2_LSB]);
        this->calibration_params.ac3 =
            (int16_t)((((int32_t)((int8_t)calibData[BMP180_calibration_params_AC3_MSB])) << 8) |
                calibData[BMP180_calibration_params_AC3_LSB]);
        this->calibration_params.ac4 =
            (uint16_t)((((uint32_t)((uint8_t)calibData[BMP180_calibration_params_AC4_MSB])) << 8) |
                calibData[BMP180_calibration_params_AC4_LSB]);
        this->calibration_params.ac5 =
            (uint16_t)((((uint32_t)((uint8_t)calibData[BMP180_calibration_params_AC5_MSB])) << 8) |
                calibData[BMP180_calibration_params_AC5_LSB]);
        this->calibration_params.ac6 =
            (uint16_t)((((uint32_t)((uint8_t)calibData[BMP180_calibration_params_AC6_MSB])) << 8) |
                calibData[BMP180_calibration_params_AC6_LSB]);

        /*parameters B1,B2*/
        this->calibration_params.b1 =
            (int16_t)((((int32_t)((int8_t)calibData[BMP180_calibration_params_B1_MSB])) << 8) |
                calibData[BMP180_calibration_params_B1_LSB]);
        this->calibration_params.b2 =
            (int16_t)((((int32_t)((int8_t)calibData[BMP180_calibration_params_B2_MSB])) << 8) |
                calibData[BMP180_calibration_params_B2_LSB]);

        /*parameters MB,MC,MD*/
        this->calibration_params.mb =
            (int16_t)((((int32_t)((int8_t)calibData[BMP180_calibration_params_MB_MSB])) << 8) |
                calibData[BMP180_calibration_params_MB_LSB]);
        this->calibration_params.mc =
            (int16_t)((((int32_t)((int8_t)calibData[BMP180_calibration_params_MC_MSB])) << 8) |
                calibData[BMP180_calibration_params_MC_LSB]);
        this->calibration_params.md =
            (int16_t)((((int32_t)((int8_t)calibData[BMP180_calibration_params_MD_MSB])) << 8) |
                calibData[BMP180_calibration_params_MD_LSB]);
    }

    /**
     * @brief Initialize sensor structure
     * 
     */
    void Init()
    {
        
        uint8_t deviceInfo = 0;
        Serial.print("reading bytes from slave addr: ");
        readBytes(this->slave_addr, BMP180_CHIP_ID_REG, &deviceInfo, 1);
        this->chip_id =BMP180_GET_BITSLICE(deviceInfo, BMP180_CHIP_ID);

        this->number_of_samples = 1;
        this->oversamp_setting = 0;
        this->sw_oversamp = 0;

        readBytes(this->slave_addr, BMP180_VERSION_REG,&deviceInfo, 1 );

        this->ml_version = BMP180_GET_BITSLICE(deviceInfo, BMP180_ML_VERSION); /* get ML version */
        this->al_version = BMP180_GET_BITSLICE(deviceInfo, BMP180_AL_VERSION); /* get AL version */

        this->oss = 3; //most accurate but slowest

        getCalibrationParameters();
        Serial.println("init function ended" );
    }




    /**
     * @brief Read raw temperature registers and combine to one uint16
     * 
     * @param ut pointer to hold uncalibrated temperature
     */
    void _readRawTemp( uint16_t* ut)
    {
        
        writeByte(this->slave_addr  ,BMP180_CTRL_MEASUREMENT_REG, BMP180_TEMPERATURE_CTRL);
        //wait 4.5ms?
        uint8_t MSB, LSB;
        readBytes(this->device_addr, BMP180_OUT_MSP, &MSB, 1);
        readBytes(this->device_addr, BMP180_OUT_MSP, &LSB, 1);
        *ut = ((MSB << 8 ) | LSB); 

    }
    /**
     * @brief Read raw temperature registers and combine to one uin16
     * 
     * @param up pointer to hold uncalibrated pressure
     */
    void _readRawPressure( uint32_t* up)
    {
        
        writeByte(this->slave_addr, BMP180_CTRL_MEASUREMENT_REG, BMP180_PRESSURE_CTRL);
        //wait 4.5ms?
        uint8_t MSB, LSB, XLSB;
        readBytes(this->device_addr, BMP180_OUT_MSP, &MSB, 1);
        readBytes(this->device_addr, BMP180_OUT_LSP, &LSB, 1);
        readBytes(this->device_addr, BMP180_OUT_XLSP, &XLSB, 1);
        (*up) = ((MSB << 16) | (LSB << 8) | XLSB) >> (8-this->oss); 

    }
    /**
     * @brief Higher level function to get calibrated temperature
     * 
     * @return double containing calibrated temperature
     */
    double GetTemp()
    {
        uint16_t ut;
        _readRawTemp( &ut);
        double x = (ut -this->calibration_params.ac6) * this->calibration_params.ac5 / pow(2.0,15);
        double x2 = this->calibration_params.mc * pow(2, 11) / (x + this->calibration_params.md);
        return (x+x2+8.0) / pow(2,4);
    }
    /**
     * @brief Calculate calibrated pressure
     * 
     * @return pressure value
     */
    double GetPressure()
    {
        uint16_t ut;
        uint32_t up;    
        _readRawTemp( &ut);
        _readRawPressure(&up);
    int32_t  UT       = 0;
    int32_t  UP       = 0;
    int32_t  B3       = 0;
    int32_t  B5       = 0;
    int32_t  B6       = 0;
    int32_t  X1       = 0;
    int32_t  X2       = 0;
    int32_t  X3       = 0;
    int32_t  pressure = 0;
    uint32_t B4       = 0;
    uint32_t B7       = 0;

    
    B5 = computeB5(&this->calibration_params , ut);

    /* pressure calculation */
    B6 = B5 - 4000;
    X1 = ((int32_t)this->calibration_params.b2 * ((B6 * B6) >> 12)) >> 11;
    X2 = ((int32_t)this->calibration_params.ac2 * B6) >> 11;
    X3 = X1 + X2;
    B3 = ((((int32_t)this->calibration_params.ac1 * 4 + X3) << this->oss) + 2) / 4;

    X1 = ((int32_t)this->calibration_params.ac3 * B6) >> 13;
    X2 = ((int32_t)this->calibration_params.b1 * ((B6 * B6) >> 12)) >> 16;
    X3 = ((X1 + X2) + 2) >> 2;
    B4 = ((uint32_t)this->calibration_params.ac4 * (X3 + 32768L)) >> 15;
    B7 = (UP - B3) * (50000UL >> this->oss);
    
    if (B4 == 0) return BMP180_ERROR;                                     //safety check, avoiding division by zero

    if   (B7 < 0x80000000) pressure = (B7 * 2) / B4;
    else                   pressure = (B7 / B4) * 2;

    X1 = pow((pressure >> 8), 2);
    X1 = (X1 * 3038L) >> 16;
    X2 = (-7357L * pressure) >> 16;

    return pressure = pressure + ((X1 + X2 + 3791L) >> 4);
        

    }

    /**
     * @brief Calculate altitude based off pressure
     * 
     * @return current altitude
     */
    double getAltitude()
    {

    return 44330 * (1 - pow( this->GetPressure() / this->Pressure0, 1.0/5.255));//pressure change of 1hPa is 8.43 meters at sea level
    }

};


/**
 * @brief Used to verify i2c transaction
 * 
 * @param bmp bmp180 device
 * @return true if chip id is expected value
 * @return false otherwise
 */
bool verifySensor(BMP180* bmp )
{
    bool deviceAddr = (bmp->chip_id == 0x55);
    Serial.print("deviceAddr result: ");
    Serial.println(deviceAddr);
    Serial.println(bmp->chip_id);
    return deviceAddr;

}








#endif
