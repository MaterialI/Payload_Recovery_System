# Payload_Recovery_System
## The rev 1.0  
Features: 
### Course correction 
> Uses PID and course error to make the steering input and correct the heading. May utilze both true heading and movement heading to calculate vector and magnitude of wind. Useful for 
### State Machine 
> Allows the system to recognize the in which stage of the flight it is and does the necessary actions.
> There are four states <ol>  <li> Ascend </li> <li>Descend </li> <li>Landing </li> <li>On the ground </li> </ol>
## Telemetry recording 
> Performs recording of the data received from sensors. Stores it in a CSV (Comma Separated Value) format.

## Sensor Suite
> ![Microcontroller Unit - Teency 4.1](https://github.com/MaterialI/Payload_Recovery_System/tree/main/Photos/teensy-4.1-cover.jpeg?raw=true )
> ![IMU - MPU 9250 Breakout board](https://github.com/MaterialI/Payload_Recovery_System/tree/main/Photos/13762-01a.jpg?raw=true )
> ![Barometer - MPL115a2](https://github.com/MaterialI/Payload_Recovery_System/tree/main/Photos/1893-02.jpg?raw=true )
> ![Servo - Parallax 360 continuous servo](https://github.com/MaterialI/Payload_Recovery_System/tree/main/Photos/900-00360_SPL.jpg?raw=true )
> ![GNSS antenna - Taoglas ADFGP.50A.07](https://github.com/MaterialI/Payload_Recovery_System/tree/main/Photos/ADFGP.50A.07.0100C_01-1000x1000.png?raw=true )
> ![GNSS breakout board - Ublox neo m9n](https://github.com/MaterialI/Payload_Recovery_System/tree/main/Photos/gps-15005_SPL.jpg?raw=true )

## Schematics
> ![Schematics including MCU, IMU, GNSS and MPL](https://github.com/MaterialI/Payload_Recovery_System/tree/main/Photos/image.png?raw=true )



