#include <avr/io.h>
#include <util/delay.h>
//#include <vector>
#include "Adafruit_BMP085.h"


struct State
{
	std::vector<float> alts;
	std::vector<float> vel;
	float altitude;
	float velocity;
	float acceleration;
	bool Ground = false;
	bool Powered = false;
	bool Deploy1 = false;
	bool Deploy2 = false;
	bool Land = false;
	Adafruit_BMP085 bmp;	
	unsigned long time;
	void Update()
	{
		this->alts.push_back(s->bmp.readAltitude());
		this->vel.push_back(integrate(s->alts, s->time); 

		this->altitude = s->alt.back();
		this->velocity = s->vel.back();
		this->acceleration = integrate(s->vel, s->time);
		this->time = millis();

	}
};


float integrate(std::vector<float>& y, unsigned long& t)
{
	//TODO Reduce error/noise, consider imu/ additional sensors
	//Get time each loop takes
	//
	unsigned int index = y.size() -1;
	return (y.at(index) - y.at(index-1))/ (float)(millis()-t);
}



void GroundIdle(state* s)
{
	PORTB |= (1 << 5); //1 light indicates gound idle achieved
	s->Update();

	s->ground = true;
	
	
}


void PoweredFlight(state* s)
{

	s->Update();
	PORTB &= ~(1 << 5); //turn lights off for flight modes
	s->Powered = true;	
	
}

void UnpoweredAscent(state* s)
{
	s->Update();

	if (false) // deploy at apogee
	{
		PORTB |= (1 << 3);
		s->Deploy1 = true;
	}
}

void UnpoweredDescent(state* s)
{
	s->Update();
	
	if (false) //second deployment
	{
		PORTB |= (1 << 2);
		s->Deploy2 = true;
	}

}

void Landed(state* s)
{
	PORTB |= (1 << 5); //two lights indicate ground state achieved
	PORTB |= (1 << 4); 
}

int main()
{
	//TODO standardize pinouts
	DDRB |= (1<<5); // GroundIdle
	DDRB |= (1<<4); // Ground success
	DDRB |= (1<<2); // Deploy2
	DDRB |= (1<<3); // Deploy1
	
	State state;

	_delay_ms(5000); //5 second delay to save data while ground idle
	state->time = millis();
	state->alts.push_back(state->bmp.readAltitude());
	state->vel.push_back(0.0f);
	while(1)
	{
		//TODO investigate and standardize thresholds
		if (state->altitude < 50.0f && state->velocity < 5.0f) GroundIdle(state);

		if (state->velocity > 5.0f && state->acceleration > 2.0f && state->Ground) PoweredFlight(state);

		if (state->velocity > -1.0f && state->acceleration < -9.0f && state->altitude > 100.0f  && state->Powered) UnpoweredAscent(state);
		
		if (state->velocity < -5.0f && state->acceleration < 0.0f && state->Deploy1) UnpoweredDescent(state)

		if (state->velocity < 0.5f && state->acceleration < 0.1f && state->Deploy2) Landed(state);
		
	}

	
}
