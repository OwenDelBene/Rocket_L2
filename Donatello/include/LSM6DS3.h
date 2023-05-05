
#include "I2C.h"

//Noise in 16g mode 180 mug/sqrt(hz)
// <1.25mA

#define WHO_AM_I_REG  0x0F
#define WHO_AM_I 0x69
//#define LSM6DS3_SLAVE_ADDR 0x35 
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
#define LSM6DS3_ACCEL_OUTPUT_Z_HIGHER 0x2D

#define LSM6DS3_FIFO_DATA_OUT_HIGHER 0x3E
#define LSM6DS3_FIFO_DATA_OUT_LOWER 0x3F


#define LSM6DS3_ACCEL_READY_INT1 0x0D
#define LSM6DS3_STATUS_REG 0x1E


struct LSM6DS3
{
  //output data rate
  uint8_t odr;
  //accelerometer scale in units of +-g
  uint8_t scale;

  //slave address
  uint8_t slaveAddr;
  //conversion of raw value to m/s^2
  double conversionFactor;

  /**
   * @brief Construct a new LSM6DS3 object
   * 
   * @param outputDataRate see datasheet
   * @param accelerometerScale unints of +-g, possible values 2, 4, 8, 16
   * @param slaveAddress default if SD0 is shorted with ground, (LSM6DS3_SLAVE_ADDR | 1) if shorted with vin
   */
  LSM6DS3(uint8_t outputDataRate = 6, uint8_t accelerometerScale =2, uint8_t slaveAddress =LSM6DS3_SLAVE_ADDR )
  { //anti aliasing filter?
    //odr=10 -> highest performance, scale = 1 -> +-16g
      this->odr = outputDataRate;
      this->scale = accelerometerScale;
      this->slaveAddr = slaveAddress;
      this->conversionFactor = .0305 * scale * 9.81 / 1000.0f;
      
  }
  /**
   * @brief Initialize sensor for calculating acceleration, see application note
   * 
   */
  void Init_A()
  {
      
      
      //enable acceleromter axis output

      writeByte(slaveAddr, LSM6DS3_ACCEL_OUTPUT_ENABLE, 0x38);


      uint8_t scaleSelection = getScaleSelection(scale);

      uint8_t data  = (odr << 4) | (scaleSelection << 2);

      writeByte(slaveAddr, LSM6DS3_ACCEL_ODR_SCALE_BANDWIDTH, data);//0x60);//High performance mode

      
      
      conversionFactor = .0305 * scale * 9.81 / 1000.0f;
      //accelerometer ready interrupt 1

      writeByte(slaveAddr, LSM6DS3_ACCEL_READY_INT1, 0x01);
      
  }
  /**
   * @brief Helper function to get the value to write to device for given scale in +-g
   * 
   * @param scale 
   * @return uint8_t 
   */
  uint8_t getScaleSelection(uint8_t scale)
  {
    if (scale ==2) return 0;
    if (scale ==16) return 1;
    if (scale ==4) return 2;
    if (scale ==8) return 3;
    return 0;
  }

  /**
   * @brief low level function to read acceleration x output registers and combine
   * 
   * @param ax output of uncalibrated  x acceleration
   */
  void _accelX(uint16_t* ax)
{
    uint8_t LSB, MSB;

    readByte(LSM6DS3_SLAVE_ADDR, LSM6DS3_ACCEL_OUTPUT_X_HIGHER, &MSB);
    readByte(LSM6DS3_SLAVE_ADDR, LSM6DS3_ACCEL_OUTPUT_X_LOWER, &LSB);
    (*ax) = (MSB << 8) | LSB; 
}
/**
   * @brief low level function to read acceleration y output registers and combine
   * 
   * @param ay output of uncalibrated  y acceleration
   */
void _accelY(uint16_t* ay)
{
    uint8_t LSB, MSB;

    readByte(LSM6DS3_SLAVE_ADDR, LSM6DS3_ACCEL_OUTPUT_Y_HIGHER, &MSB);
    readByte(LSM6DS3_SLAVE_ADDR, LSM6DS3_ACCEL_OUTPUT_Y_LOWER, &LSB);
    (*ay) = (MSB << 8) | LSB; 
    
}
/**
   * @brief low level function to read acceleration z output registers and combine
   * 
   * @param az output of uncalibrated  z acceleration
   */
void _accelZ(uint16_t* az)
{
    uint8_t LSB, MSB;

    readByte(LSM6DS3_SLAVE_ADDR, LSM6DS3_ACCEL_OUTPUT_Z_HIGHER, &MSB);
    readByte(LSM6DS3_SLAVE_ADDR, LSM6DS3_ACCEL_OUTPUT_Z_LOWER, &LSB);
    (*az) = (MSB << 8) | LSB; 
}


/**
 * @brief Get acceleration in the x direction
 * 
 * @return float acceleration in x direction
 */
float accelX()
{
  uint16_t ax;
  _accelX(&ax);
  return (int)ax * conversionFactor; //valid near earth surface
  
}

/**
 * @brief Get acceleration in the y direction
 * 
 * @return float acceleration in y direction
 */
float accelY()
{
  uint16_t ay;
  _accelY(&ay);
  return (int)ay * conversionFactor; //valid near earth surface
  
}

/**
 * @brief Get acceleration in the z direction
 * 
 * @return float acceleration in z direction
 */
float accelZ()
{
  uint16_t az;
  _accelZ(&az);
  return (int)az * conversionFactor; //valid near earth surface
  
}

/**
 * @brief Low level function to read raw temperature registers
 * 
 * @param temp combined value of raw temperature registers
 */
void _temperature(uint16_t* temp)
{
    uint8_t LSB, MSB;

    readByte(slaveAddr, LSM6DS3_TEMP_OUTPUT_UPPER, &MSB);
    readByte(slaveAddr, LSM6DS3_TEMP_OUTPUT_LOWER, &LSB);

    (*temp) = (MSB << 8) | LSB; 
}

/**
 * @brief Used for debugging, prints all accelerations and temperature
 * 
 */
  void printXYZ()
  {
      Serial.print("Accelerometer X: ");
      Serial.print(accelX());
      Serial.print(", Accelerometer Y: ");
      Serial.print(accelY());
      Serial.print(", Accelerometer Z: ");
      Serial.print(accelZ());
      Serial.println();
      uint16_t temp;
      _temperature(&temp);
      Serial.print("Temp: ");
      Serial.println(temp);
  }
  
};



/**
 * @brief Tests i2c connection, if unsuccessful check sd0 voltage
 * 
 * @return true if who am i returns expected value
 * @return false otherwise
 */
bool accelerometerTest()
{
    uint8_t whoAmI;
    readByte(LSM6DS3_SLAVE_ADDR, WHO_AM_I_REG, &whoAmI);
    Serial.println(whoAmI);
    return whoAmI == WHO_AM_I; 
}
