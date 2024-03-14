# Payload_Recovery_System
## The rev 1.0  
> ![Test version 1](https://github.com/MaterialI/Payload_Recovery_System/blob/main/Versions/Test%201.0/deploy/sketch_jan2a/sketch_jan2a.ino)
Features: 
### Course correction 
> Uses PID and course error to make the steering input and correct the heading. May utilze both true heading and movement heading to calculate vector and magnitude of wind. Useful for sensor fusion task and dead reckoning algorithm.
> Version 2 will include more sophisticated approach to sensor fusion and system control, as well as failure protection through dead reckoning and intelligent landing system.
### State Machine 
> Allows the system to recognize the in which stage of the flight it is and does the necessary actions.
> There are four states <ol>  <li> Ascend </li> <li>Descend </li> <li>Landing </li> <li>On the ground </li> </ol>
## Telemetry recording 
> Performs recording of the data received from sensors. Stores it in a CSV (Comma Separated Value) format.

## Sensor Suite
![Teensy 4.1](https://raw.githubusercontent.com/MaterialI/Payload_Recovery_System/main/Photos/teensy-4.1-cover.jpeg) 
Width: 400

![IMU - MPU 9250](https://raw.githubusercontent.com/MaterialI/Payload_Recovery_System/main/Photos/13762-01a.jpg) 
Width: 400

![Barometer - MPL115a2](https://raw.githubusercontent.com/MaterialI/Payload_Recovery_System/main/Photos/1893-02.jpg) 
Width: 400

![GNSS breakout board - Ublox neo m9n](https://raw.githubusercontent.com/MaterialI/Payload_Recovery_System/main/Photos/gps-15005_SPL.jpg) 
Width: 400

![GNSS antenna - Taoglas ADFGP.50A.07](https://raw.githubusercontent.com/MaterialI/Payload_Recovery_System/main/Photos/ADFGP.50A.07.0100C_01-1000x1000.png) 
Width: 400

![Servo - Parallax 360 continuous servo](https://raw.githubusercontent.com/MaterialI/Payload_Recovery_System/main/Photos/900-00360_SPL.jpg) 
Width: 400


## Schematics
- ![Schematics including MCU, IMU, GNSS and MPL](https://github.com/MaterialI/Payload_Recovery_System/tree/main/Photos/image.png?raw=true)
