#include "Kalman.h"


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

  BMP180 bmp;
  LSM6DS3 accel;

    State()
    {
        bmp.Init();
        accel.Init_A();
    }
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
      this->obs(0) = bmp.getAltitude();
      obs(1) = 0.0;
      this->obs(1) = accel.accelZ();
       

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