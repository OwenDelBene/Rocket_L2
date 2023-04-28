#include <Wire.h>
#include "bmp180.h"
#include "LSM6DS3.h"

uint16_t ay;
  double pressure;
BMP180 bmp;
void setup() {
  // put your setup code here, to run once:
  Wire.begin();
  Wire.setClock(100000); 
  Serial.begin(115200);
  Serial.println("testing bmp180");
  
  

  bmp180_Init(&bmp);
  accelerometerInit();
  Serial.println("Verifying accel");
  Serial.println(accelerometerTest());
  
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println("Accelerometer X: ");
  accelY(&ay);
  Serial.println(ay);
  Serial.println();
  
  
  Serial.println("Pressure: ");
  pressure = GetPressure(&bmp);
  Serial.println(pressure);
  Serial.println();
  
}
