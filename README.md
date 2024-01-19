# Payload_Recovery_System
## The rev 1.0  
Features: 
### Path construction 
> Constructs a line in small areas 200 X 200 km between the current location of system and the final target  
### Path following 
> Uses PID and distance calculation in spherical coordinates and finds the XTE (Cross Track Error) from the path. After that it adjusts the steering to follow it. 
### Course correction 
> Uses PID and course error to make the steering input and correct the heading.
### State Machine 
> Allows the system to recognize the in which stage of the flight it is and does the necessary actions.
> There are four states <ol>  <li> Ascend </li> <li>Descend </li> <li>Landing </li> <li>On the ground </li> </ol>
## Telemetry recording 
> Performs recording of the data received from sensors. Stores it in a CSV (Comma Separated Value) format.

## Sensor Suite
> ![Microcontroller Unit - Teency 4.1](https://github.com/MaterialI/Payload_Recovery_System/tree/main/Photos/teensy-4.1-cover.jpeg?raw=true "Microcontroller unit")
> ![IMU - MPU 9250 Breakout board](https://github.com/MaterialI/Payload_Recovery_System/tree/main/Photos/13762-01a.jpg?raw=true "Microcontroller unit")
> ![Barometer - MPL115a2](https://github.com/MaterialI/Payload_Recovery_System/tree/main/Photos/1893-02.jpg?raw=true "Microcontroller unit")


