//#include <avr/io.h>
//#include <util/delay.h>

#include <Adafruit_BMP085.h>
#include <Wire.h> // Must include Wire library for I2C
#include <SparkFun_MMA8452Q.h>
#include "Kalman.h"


//Kalman filter constants
const int N_state = 3; //state vector for pos, vel, accel
const int N_obs = 2; //observing pos, accel

const float n_p = .17;
const float n_a = .12;

const float m1 = 0.1;
const float m2 = 0.1;
const float m3 = 0.8;

Adafruit_BMP085 bmp;
MMA8452Q accel;

class State
{
  public:
  float altitude;
  float velocity;
  float acceleration;
  bool Ground = false;
  bool Powered = false;
  bool Deploy1 = false;
  bool Deploy2 = false;
  bool Land = false;
  
  unsigned long time;
  Kalman<N_state, N_obs> K;
  BLA::Matrix<N_obs> obs;
  float dt;
  void K_init()
  {
      K.F = {1.0, 0.0, 0.0,
            0.0, 1.0, 0.0, 
            0.0, 0.0, 1.0}; //identity evolution matrix; 
      K.H = {1.0, 0.0, 0.0, 
            0.0, 0.0, 1.0};  //identity measurement matrix; 

      K.R = {n_p * n_p, 0.0,
            0.0, n_a * n_a}; // measurement covariance

      K.Q = {m1*m1, 0.0, 0.0,
            0.0, m2*m2, 0.0,
            0.0, 0.0, m3*m3}; //model covariance
    
    
  }

  void Measure()
  {
      this->obs(0) = bmp.readAltitude(101500);
      obs(1) = 0.0;
      if (accel.available())
      {
        accel.read();
        this->obs(1) = accel.getCalculatedZ() * -9.81;
      } 

  }
  void Update_k()
  {
    Serial.println("Update K");
    dt = (millis()-time)*.001;
    time = millis();

    K.F = {1.0, dt, dt*dt*.5,
         0.0, 1.0, dt, 
         0.0, 0.0, 1.0}; //identity evolution matrix; 
    
    Measure();
    K.update(obs);

  }
  void Update()
  {
    Serial.println("Update()");
    Update_k();
    this->altitude = K.x(0); 
    this->velocity = K.x(1);
    this->acceleration = K.x(2);
    this->time = millis();
    
  }
};





void GroundIdle(State& s)
{
  PORTB |= (1 << 5); //1 light indicates gound idle achieved


  s.Ground = true;
  
  
}


void PoweredFlight(State& s)
{

  PORTB &= ~(1 << 5); //turn lights off for flight modes
  s.Powered = true;  
  
}

void UnpoweredAscent(State& s)
{

  if (s.velocity < 0.1f) // deploy at apogee
  {
    PORTB |= (1 << 3);
    s.Deploy1 = true;
  }
}

void UnpoweredDescent(State& s)
{
  
  if (s.altitude < 300.0) //second deployment
  {
    PORTB |= (1 << 2);
    s.Deploy2 = true;
  }

}

void Landed(State& s)
{
  PORTB |= (1 << 5); //two lights indicate ground state achieved
  PORTB |= (1 << 4); 
}

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


State state;
void setup()
{
  Serial.begin(115200)
//TODO standardize pinouts
  // DDRB |= (1<<5); // GroundIdle
  // DDRB |= (1<<4); // Ground success
  // DDRB |= (1<<2); // Deploy2
  // DDRB |= (1<<3); // Deploy1

  
  Serial.begin(9600);

  accel.init();
  bmp.begin();

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

    if (accel.available())
    {
      accel.read();
      Serial.println("Calculatedz");
      Serial.println(accel.getCalculatedZ());
    }

    Serial.println("measured alt");
    Serial.println(bmp.readAltitude(1013.25));
}
