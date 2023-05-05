#include <Wire.h>
#include "include/bmp180.h"
#include "include/LSM6DS3.h"

//device structures
BMP180 bmp = BMP180();
LSM6DS3 accel = LSM6DS3();

void setup() {
  Wire.begin();
//  Wire.setClock(100000); 
  Serial.begin(115200);

  //initialze sensors for collecting pressure/ acceleration readings
  bmp.Init();
  accel.Init_A();

  //check if i2c works properly
  Serial.println("Verifying accel");
  Serial.println(accelerometerTest());
  Serial.println();
  Serial.println();

  
}

void loop() {
  accel.printXYZ();
  Serial.println();
  delay(1000);
  Serial.print("BMP altitude ");
  Serial.println(bmp.getAltitude());
  
}
