# L2Rocket
Rocket flight software, simulations, and CAD for a high powered rocket preparing for a L2 certification. 

## Avionics

#### Arduino Uno:
Flight computer powered by 9V battery, armed with screw switch.

#### BMP085:
Barometric pressure sensor connected to Flight computer over I2C.

#### LSM6Ds3:
3 axis accelerometer and gyroscope connected to Flight computer over I2C.


## Structure
Plywood fins, cardboard aft airframe. Avionics bay, nosecone, forward airframe, motor retainer, 


## Software

#### State machine:
High powered rockets go through 4 states: ground idle, powered ascent, unpowered ascent, descent. 
During ground idle both sensors are initialized and begin returning data. The Kalman filter is intiailized and begins filtering altitude and vertial acceleration returned from the barometric pressure sensor and accelerometer respectively to return position velocity and acceleration. Leds are lit to elucidate the state on the launch rail. The initial altitude is recorded for reference.
The rocket enters powered ascent when it has detected sufficient vertical acceleration and sufficient altitude relative to the initial altitude. The rocket enters unpowered ascent when the vertical acceleration becomes negative. The ejection charge for the drogue parachute is fired when vertical velocity is zero. For safety the ejection charge can only be fired above a certain altitude. The rocket enters descent when its velocity becomes negative and for safety above a certain altitude. During descent the ejection charge for the main parachute is fired at an 300m above initial altitude as determined by openrocket simulations. 


#### Kalman Filter:
Takes input altitude and acceleration data and outputs position velocity and acceleration. Utilizes measurement noise from sensor datasheets. 

## Simulations

#### Openrocket:
Openrocket was used to simulate the rocket design with various J class rocket motors to determine the appropirate altitude for main chute release, and determine rocket stability. 

## Testing
#### Ejection testing
An esp32 hosted a http server to remotely control the flight computer, sending signals to trigger the ejection charges to ensure sufficient pressure for separation.
#### Hardware in the loop:
Sensors were turned off and Kalman filter was fed altitude and acceleration data with noise from recorded thrust curves for various J class rocket motors. State transitions were recorded with the current input data as well as the state vector returned by the kalman filter, to verify flight software state machine.
