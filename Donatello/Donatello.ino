//#include <avr/io.h>
//#include <util/delay.h>


#include <Wire.h> // Must include Wire library for I2C

#include "include/LSM6DS3.h"
#include "include/BMP180.h"
#include "include/State.h"







State state;





/*int main()
{
  
  //TODO standardize pinouts
  // DDRB |= (1<<5); // GroundIdle
  // DDRB |= (1<<4); // Ground success
  // DDRB |= (1<<2); // Deploy2
  // DDRB |= (1<<3); // Deploy1

  
  Serial.begin(9600);
  Serial.println("hi init");
  accel.init();
  Serial.println("hi accel");
  //_delay_ms(5000); //5 second delay to save data while ground idle
  State state;
  state.K_init();
  state.time = millis();
  while(1)
  {
    Serial.println("hi");
    //TODO investigate and standardize thresholds
    state.Update();
    Serial.println("hi2, \n");
    Serial.println(state.altitude) ; 
    Serial.println(state.velocity); 
    Serial.println(state.acceleration);
    Serial.println('\n');
    if (state.altitude < 50.0f && state.velocity < 5.0f) GroundIdle(state);

    if (state.velocity > 5.0f && state.acceleration > 2.0f && state.Ground) PoweredFlight(state);

    if (state.velocity > -1.0f && state.acceleration < -9.0f && state.altitude > 100.0f  && state.Powered) UnpoweredAscent(state);
    
    if (state.velocity < -5.0f && state.acceleration < 0.0f && state.Deploy1) UnpoweredDescent(state);

    if (state.velocity < 0.5f && state.acceleration < 0.1f && state.Deploy2) Landed(state);
    
    
    
  }

  return 0;
}*/



void setup()
{
  Serial.begin(115200);
//TODO standardize pinouts
  // DDRB |= (1<<5); // GroundIdle
  // DDRB |= (1<<4); // Ground success
  // DDRB |= (1<<2); // Deploy2
  // DDRB |= (1<<3); // Deploy1

  
  Serial.begin(9600);



  //_delay_ms(5000); //5 second delay to save data while ground idle
  
  state.K_init();
  state.time = millis();

}



void loop()
{
  Serial.println("hi");
    //TODO investigate and standardize thresholds
    state.Update();
    Serial.println("altitude, velocity, acceleration, \n");
    Serial.println(state.altitude) ; 
    Serial.println(state.velocity); 
    Serial.println(state.acceleration);
    Serial.println('\n');
    if (state.altitude < 50.0f && state.velocity < 5.0f) GroundIdle(state);

    if (state.velocity > 5.0f && state.acceleration > 2.0f && state.Ground) PoweredFlight(state);

    if (state.velocity > -1.0f && state.acceleration < -9.0f && state.altitude > 100.0f  && state.Powered) UnpoweredAscent(state);
    
    if (state.velocity < -5.0f && state.acceleration < 0.0f && state.Deploy1) UnpoweredDescent(state);

    if (state.velocity < 0.5f && state.acceleration < 0.1f && state.Deploy2) Landed(state);

}
