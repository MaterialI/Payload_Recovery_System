// Testing 1.0
// The software is written to evaluate the steering mechanism and sensor suite. In PLR group, SFU Rocketry. 
// data retrieve is done throguh 2 loops 1) thread which deals with low frequency updates of gnss and barometer 2) main loop which responds to data and deals with high frequency data from IMU 
// state machine retrieves the data and takes actions with lower rate of processing (done purely for optimization purposes, however, rates can be increased to increase precision of the system)
// Written by Sviatoslav Rublov



#include <_Teensy.h>

#include <Wire.h>                                 //wire library to communicate
#include <TeensyThreads.h>                        //threads
#include <PID_v1.h>                               //PID
#include "SparkFun_u-blox_GNSS_Arduino_Library.h" //gps
#include <Adafruit_MPL3115A2.h>                   //barometer
#include <MPU9250.h>                              //Gyro, accel
// #include <arm_math.h>                             //complex math functions
#include <math.h>                                 //simple math functions
#include <SPI.h>                                  //spi library
#include <SD.h>                                   //sd card
#include <FeedBackServo.h>
// to store the datapoints of the path we use an array of pairs which belong to orthdrome path on sphere (lon, lat)
// between the points we use approximation to loxodrome path (heading is the same throughout the whole path)

// bootup -> ascend -> descend -> landing

#define BOOTUP_STATE 0
#define ASCEND_STATE 1
#define DESCEND_STATE 2
#define LANDING_STATE 3


bool TEST =  true; //to ignore GNSS module presence check and turn on serial print

// communication pins/ i2c adresses.
#define FEEDBACK_PIN 7
#define BUZZER_PIN 22
#define SERVO_PIN 6
#define IMU_ADDRESS 0x68

//instances
SFE_UBLOX_GNSS myGNSS;                             // defy a gps instance
Adafruit_MPL3115A2 baro = Adafruit_MPL3115A2();    // defy a baro instance
MPU9250 mpu;                                       // gyro, accel, compass
FeedBackServo servo = FeedBackServo(FEEDBACK_PIN); /////Servo

short state = BOOTUP_STATE; //initial state
bool fallDetected = false;
bool descendDetected = false; //todo implement the 2 step verification of fall. Based on the measurements of opening.

bool sensorsVerified = true; // assume that all sensors are working and responding, if not, the system will not exit from bootup state.

// threads
int baroGnssThr; //due to the sensor long response and delay of signal, send it on a separate thread for asynchronous data fetching. 
int gnssThr;

//mutexes
Threads::Mutex baroGnss;

// GPS location
float gpsHeading = 0.0; //measured over position shift. 
float latitude = 0.0;
float longitude = 0.0;
float groundSpeed = 0.0; // m/s measured by the GNSS

long int GNSSdeltaT = 0; // time delta between measurements to provide vertical speed by GPS
float gnssAltitude = 0.0; //measured by GNSS from Mean Sea Level (MSL)

// Modelling
int modelLatitude = 0;
int modelLongitude = 0;
float nextPointHeading = 0.0;

float magDecl = 15.75; // in degrees to the West in Vancouver, October 2023

// Accelerometer (used to measure free fall and landed state) also future todo dead reckoning
float accX = 0.0;
float accY = 0.0;
float accZ = 0.0;

// Euler (heading is used for the direction measure (measured in 0-360 deg)), pitch and roll can be used for debugging purposes
float roll = 0.0;
float pitch = 0.0;
float heading = 0.0;

// Barometer
float baroAltitude = 0.0;     // metres above sea level
float pressure = 0.0;         // Pa
float baroTemperature = 0.0; // Degrees Centigrade 

// barometer vertical speed calculation
float prevAltitude = 0.0; // m
float deltaT_baro = 0;    // used to measure the time between samples to calculate vertical speed.
float vspeed = 0.0;       // m/s barometer

float prevGnssAltitude = 0.0; // additional variable to measure vertical speed 
float deltaT_gnss = 0;
float gnssVspeed = 0.0;

// PathConstruction--------------------------

// final locations to be preprogrammed
float finLat = 49.1951;
float finLong = -123.1788;


// PID ----- ///////////////////////////////

// PID coefficients //to change
double cP = 0.5;
double cI = 0.5;
double cD = 0.2;


// pid setting (Can be used to set offsets to follow the path well, can be paired with track to make less adjustments to save the energy and make the flight better)
double allowedGoalRot = 0.0;
double pidInput = 0.0;
double steeringOutput = 0.0; //ranges from -180 to 180 with coefficient 1, to scale up or down the rotation, change rangeCoefficient variable
double rangeCoefficient = 1.0;
PID courseCorrection(&pidInput, &steeringOutput, &allowedGoalRot, cP, cI, cD, DIRECT); /// init course correction PID

// Telemetry recording
File telemetryFile;
String fileName = "telemetry"; //timestamp will be added upon setup. 
String timestamp = "";
String fileExtension = ".csv";
char telemetryBuffer[100]; 


//timings
long timeLoopSeparator = 0;
/////////////////////////////////////////// navigation


float distance2Points(float lat2, float lon2) // estimates distance in kilometres (accurate stuff)
{
  
  float lat1 = latitude / 180 * PI;
  float lon1 = longitude / 180 * PI;
  lat2 = lat2 / 180 * PI;
  lon2 = lon2 / 180 * PI;
  float pLat = pow(sin((lat1 - lat2) / 2), 2);
  float pLong = cos(lat1) * cos(lat2) * pow(sin((lon1 - lon2) / 2), 2);
  float d = 2 * asin(sqrt(pLat + pLong));
  return d * 6371.0;
}

float setCourse2Points(float finalLat, float finalLong) //sets course using great circle. Implies concstantly changing course over long distances. 
{
  
  float startLat = latitude / 180 * PI;
  float startLong = longitude / 180 * PI;
  finalLat = finalLat / 180 * PI;
  finalLong = finalLong / 180 * PI;
  float pLat = pow(sin((startLat - finalLat) / 2), 2);
  float pLong = cos(startLat) * cos(finalLat) * pow(sin((startLong - finalLong) / 2), 2);
  float d = 2 * asin(sqrt(pLat + pLong));

  // get course
  // edgecase on poles
  float course = 0.0;
  if (cos(startLat) < 0.0000001)
  {
    if (startLat > 0)
    {
      course = PI;
    }
    else
      course = 2 * PI;
  }

  // set course
  if (sin(finalLong - startLong) < 0)
  {
    course = acos((sin(finalLat) - sin(startLat) * cos(d)) / (sin(d) * cos(startLat)));
  }
  else
    course = 2 * PI - acos((sin(finalLat) - sin(startLat) * cos(d)) / (sin(d) * cos(startLat)));
  return 360.0 - ((course / PI) * 180);
}



void pidDeltaAngle() // provides angle delta value in range (-180 to 180) between course and heading to PID to correct the heading to delta = 0
{

  int delta = nextPointHeading * 100;

  int iYaw = heading * 100;

  int newYaw = (delta - iYaw) % 36000;
  if (newYaw < 18000)
  {
    newYaw *= -1;
  }
  else
    newYaw = 18000 - (newYaw) % 18000;
  pidInput = (float)(newYaw) / 100;
}



////////////////////////////////-----------------measurements------------------------------///////////////////////

int accelThread() // the function is being called by main loop
{
  // todo: add mutex locks whenever using the data
  // while (true)
  {

    if (mpu.update())
    {
      roll = mpu.getRoll();
      pitch = mpu.getPitch();
      heading = mpu.getYaw() + 180.0; // to convert from (-180, 180) to (0, 360)
      accX = mpu.getAccX();
      accY = mpu.getAccY();
      accZ = mpu.getAccZ();
      fallDetected = detectFall();
      return 1;
    }
    else return 0;
  }
}

void baroThread() // uses the library to calculate the altitude. Added dt to calculate the verical speed. TODO: add exponential averaging
{
  while(true)
  { 
    
    baro.startOneShot();
    while (!baro.conversionComplete()) {} //busy wait on a data on one thread.
    

    //---------------gnssCall
    gnssThread();
    


    baroAltitude = baro.getLastConversionResults(MPL3115A2_ALTITUDE);
    vspeed = exponentialAveraging( vspeed, (baroAltitude - prevAltitude) / (((float)(millis()) - deltaT_baro) / 1000.0), 0.6);
    deltaT_baro = (float)(millis());
    baroTemperature = baro.getTemperature();
    
    prevAltitude =  baroAltitude;
  }
}

void gnssThread() // uses library to get the coordinates in (long, lat) format. Calculates the ground speed. TODO: add veritcal speed calculation and fuse with barometer readings.
{

  
  {
      
      latitude = (float)myGNSS.getLatitude()/ 10000000;
      longitude = (float)myGNSS.getLongitude() / 10000000;
      gnssAltitude = (float)(myGNSS.getAltitude());
      gpsHeading = myGNSS.getHeading() /100000;
      groundSpeed = (float)(myGNSS.getGroundSpeed()) / 1000.0;
      generateTimeStamp();
      

      gnssVspeed = exponentialAveraging(gnssVspeed,(gnssAltitude - prevGnssAltitude) / (((float)(millis()) - deltaT_gnss) / 1000.0), 0.5);

      deltaT_gnss = (float)(millis());

      prevGnssAltitude = gnssAltitude;
      

  }
}







void setup()
{
  // initialize the communication with the sensors.
  Serial.begin(9600);
  Wire2.begin(); //second bus for IMU
  Wire.begin();
  delay(1000);

  //initializes sensors
  sensorsInit();

  baroGnssThr = threads.addThread(baroThread);
  // gnssThr = threads.addThread(gnssThread);
  // call PID settings
  courseCorrection.SetMode(AUTOMATIC);
  state = ASCEND_STATE;

  
}

void loop()
{

  // call measurements (only IMU, I know)
  accelThread();
  

  if(TEST)
  {
    
    printData();
    
  }
  else
    stateAction();
  
}


//--------------------------ACTION FUNCTIONS --------------------


//write data to a telemetry File
void sdWrite()
{

  telemetryFile.print(heading);
  telemetryFile.print(",");
  telemetryFile.print(pitch);
  telemetryFile.print(",");
  telemetryFile.print(roll);
  telemetryFile.print(",");
  telemetryFile.print(gpsHeading);
  telemetryFile.print(",");
  telemetryFile.print(latitude);
  telemetryFile.print(",");
  telemetryFile.print(longitude);
  telemetryFile.print(",");
  telemetryFile.print(groundSpeed);
  telemetryFile.print(",");
  telemetryFile.print(gnssVspeed);
  telemetryFile.print(",");
  telemetryFile.print(baroAltitude);
  telemetryFile.print(",");
  telemetryFile.print(baroTemperature);
  telemetryFile.print(",");
  telemetryFile.println();
}

//takes in a steering input and range of steering coefficient
void controlServo(double steeringInput, double coefficient)
{

  servo.rotate((int)(steeringInput * coefficient), 4); //rotate by input (allowed threshold of rotation is 4 degrees)
  Serial.print("Servo Angle: "); //debug to check servo angle
  //todo write down the true rotation angle of servo

  
  
}



void printData()
{
  //gnss data
  Serial.println(fallDetected);
  Serial.print("Latitude: ");
  Serial.println(latitude, 7);
  Serial.print("Longitude: ");
  Serial.println(longitude, 7);
  Serial.print("GroundSpeed: ");
  Serial.println(groundSpeed);
  Serial.print("GNSS heading: ");
  Serial.println(gpsHeading);
  Serial.println();
  
  // orientation 
  Serial.print("Roll (deg): ");
  Serial.println(roll);
  Serial.print("Pitch (deg): ");
  Serial.println(pitch);
  Serial.print("Yaw (deg): ");
  Serial.println(heading);

  //computations 
  Serial.print("Course to final: ");
  nextPointHeading = setCourse2Points(finLat, finLong);
  Serial.println(nextPointHeading);
  Serial.print("DistanceToFinal: ");
  Serial.println(distance2Points(finLat, finLong));
  pidDeltaAngle();
  Serial.print("Delta angle: ");
  Serial.println(pidInput);
  courseCorrection.Compute();
  Serial.print("PID output: ");
  Serial.println(steeringOutput);

  //altitude/ vertical speed
  Serial.print("Baro Altitude (m): ");
  Serial.println(baroAltitude);
  Serial.print("Baro Temperature (deg C): ");
  Serial.println(baroTemperature);
  Serial.print("Vertical speed (m/s): ");
  Serial.println(vspeed);

  //state machine 
  Serial.print("State: ");
  Serial.println(state);
}



// ------------------------------------ STATE MACHINE -------------------------------------------



void stateAction() //todo finish the state machine, develop descend state machine.
{
  switch(state)
  {
    case BOOTUP_STATE: //initialize sensors, run checks om them
    {
      Serial.println("The system has not finished its bootup sequence, restart the system and check whether sensors are connected.");

      if(sensorsVerified)
      {
        bootupToAscend();
        break;
      }
      
       // loop to notify that sensors were not initialized through yelling buzzer.
      digitalWrite(BUZZER_PIN, HIGH); 
      delay(1000);            
      digitalWrite(BUZZER_PIN, LOW);  
      delay(1000); 
      
    } break;

    case ASCEND_STATE: //fall detection is running. (haven't decided how to enforce constarint)
    {
      //run data recording to visualize. The system does not take action until fall detected and parachute opened
      ascendToDescend();
      sdWrite();
    } break;
    case DESCEND_STATE:
    { 
      // data recording is still running, and system takes actions through steering and estimating the landing in position
      // todo dead reckoning will be added to compensate for low update rates of GNSS to run simulations for high speed systems, location of which cannot be well estimated through only gnss.
      // if servo will be stalling, change the servo to separate thread, which will be run with special frequency

      controlServo(steeringOutput, rangeCoefficient);
      sdWrite();
    }break;
  };
}



//------------------------STATE MACHINE TRANSITION FUNCTIONS ---------------------

//checks whether bootup was successful: all sensors initialized (including servo)
void bootupToAscend()
{
  if(sensorsVerified && state == BOOTUP_STATE)
  {
    state = ASCEND_STATE;
  }
}


// changing state machine from ascend to descend takes is fall detected
void ascendToDescend()
{
  if(fallDetected && state == ASCEND_STATE) 
  {
    state = DESCEND_STATE;
  }
}

//detects fall based on the value of accelerometer, runs synchronously with IMU
bool detectFall()
{
  float a[3] = {accX, accY, accZ};
  float pAcc = sqrt(pow(a[0], 2) + pow(a[1], 2) +pow(a[2], 2));
  if(pAcc < 0.05 && state == ASCEND_STATE) //just a precaution. 
  {
     
    return true;
  }
  return false;
}


//--------------------- INIT PROCEDURE-----------


void sensorsInit()
{
  pinMode(BUZZER_PIN, OUTPUT);

  // set the setting for IMU module
  MPU9250Setting setting;
  setting.accel_fs_sel = ACCEL_FS_SEL::A16G;
  setting.gyro_fs_sel = GYRO_FS_SEL::G2000DPS;
  setting.mag_output_bits = MAG_OUTPUT_BITS::M16BITS;
  setting.fifo_sample_rate = FIFO_SAMPLE_RATE::SMPL_125HZ;
  setting.gyro_fchoice = 0x03;
  setting.gyro_dlpf_cfg = GYRO_DLPF_CFG::DLPF_10HZ;
  setting.accel_fchoice = 0x01;
  setting.accel_dlpf_cfg = ACCEL_DLPF_CFG::DLPF_10HZ;


  if (myGNSS.begin() == true) // initialize GPS the order is chosen, so that the sensor takes time to initialize any connection with any of sattelites to get time data
  {
    Serial.println("GNSS module detected!");
    myGNSS.setMeasurementRate(200); //assumed 100 ms
    
  }
  else
  {
    Serial.println("Unable to detect GNSS module!");
    sensorsVerified = false; // sensor was not validated, thus all system is stuck in bootup
  }

  // intiailize the communication with the sensors

  if (baro.begin() == true) // initialize barometer
  {
    Serial.println("Barometer module detected!");
    baro.setMode(MPL3115A2_ALTIMETER);
  }
  else
  {
    Serial.println("Unable to detect Barometer module!");
    sensorsVerified = false; // sensor was not validated, thus all system is stuck in bootup
  }

  // initialize IMU on 2nd I2C bus
  if (mpu.setup(IMU_ADDRESS, setting, Wire2) == true)
  {
    Serial.println("Accelerometer module detected!");
    // conduct calibration of the compass and accelerometers and gyroscope
    Serial.println("Calibrating compass, please, perform rotations around 3 axis ");
    mpu.calibrateMag();
    Serial.println("Calibrating accelerometer and gyroscope");
    mpu.calibrateAccelGyro();
    Serial.println("Calibration done!");
    mpu.setFilterIterations(4);
  }
  else
  {
    Serial.println("Unable to detect Accelerometer module!");
    sensorsVerified = false; // sensor was not validated, thus all system is stuck in bootup
  }

  //initialize timestamp for a datafile. We assume gnss gets signal to retrieve time based on UTC format.
  Serial.println("Getting a timestamp for datafile to write data...");
  delay(2000);
  generateTimeStamp();
  Serial.print("timestamp generated: " + timestamp); 
  
  char* name = (fileName + timestamp + fileExtension).c_str();  //assemble data to generate full filename
  telemetryFile = SD.open(name, FILE_WRITE); //todo change name of the file based on the timestamp
  //set up servo 
  servo.setServoControl(SERVO_PIN);
  servo.setKp(1.0);

}

void generateTimeStamp() //kept as void, no mean to test the data came through. Assumed the sensor initialized and got connection. Will be called within data retrieve thread, to reduce load on i2c bus 1.
{
  
  char buffer[20];
  int year = myGNSS.getYear();
  int month = myGNSS.getMonth();
  int day = myGNSS.getDay();
  int hour = myGNSS.getHour();
  int minute = myGNSS.getMinute();
  int second = myGNSS.getSecond();
  dtostrf(day, 1, 0, buffer);
  timestamp += buffer;
  timestamp += "-";
  dtostrf(month, 1, 0, buffer);
  timestamp += buffer;
  timestamp += "-";
  dtostrf(year, 1, 0, buffer);
  timestamp += buffer;
  timestamp += "-";
  dtostrf(hour, 1, 0, buffer);
  timestamp += buffer;
  timestamp += "-";
  dtostrf(minute, 1, 0, buffer);
  timestamp += buffer;
  timestamp += "-";
  dtostrf(second, 1, 0, buffer);
  timestamp += buffer;


}



//---------------filters------------
// used to approximate average out rough and "spiky" outputs from sensors
float exponentialAveraging(float a, float b, float k1) 
{
  return a*k1 + b*(1-k1);
}


