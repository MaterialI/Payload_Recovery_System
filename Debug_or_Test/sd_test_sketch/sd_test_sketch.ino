#include <SPI.h>
#include <SD.h>
#include <MPU9250.h> 
#include <Wire.h>  
#include <string.h>
const int chipSelect = BUILTIN_SDCARD;
File myFile;
String filename = "test";
String timestamp = "6969";
String fileExtension = ".txt";
long long count = 0;
float roll = 0.0;
float pitch;
float yaw;
String name;
MPU9250 mpu; 
void setup() {
  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  Wire2.begin();


  MPU9250Setting setting;
  setting.accel_fs_sel = ACCEL_FS_SEL::A16G;
  setting.gyro_fs_sel = GYRO_FS_SEL::G2000DPS;
  setting.mag_output_bits = MAG_OUTPUT_BITS::M16BITS;
  setting.fifo_sample_rate = FIFO_SAMPLE_RATE::SMPL_125HZ;
  setting.gyro_fchoice = 0x03;
  setting.gyro_dlpf_cfg = GYRO_DLPF_CFG::DLPF_10HZ;
  setting.accel_fchoice = 0x01;
  setting.accel_dlpf_cfg = ACCEL_DLPF_CFG::DLPF_10HZ;
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }


  Serial.print("Initializing SD card...");

  if (!SD.begin(chipSelect)) {
    Serial.println("initialization failed!");
    while (1);
  }
  Serial.println("initialization done.");

  if (mpu.setup(0x68, setting, Wire2) == true)
  {
    Serial.println("Accelerometer module detected!");
    // conduct calibration of the compass and accelerometers and gyroscope
    Serial.println("Calibrating compass, please, perform rotations around 3 axis ");
    mpu.calibrateMag();
    Serial.println("Calibrating accelerometer and gyroscope");
    mpu.calibrateAccelGyro();
    Serial.println("Calibration done!");
  }
  else
  {
    Serial.println("Unable to detect Accelerometer module!");
  }
  name = (filename + timestamp + fileExtension);

  myFile = SD.open(name.c_str(), FILE_WRITE);
  myFile.println("VAmooos");
  myFile.write("VAmooos write");

}

int iter = 0;
void loop() {
  iter++;
  if(iter > 100)
  {
    myFile.close();
  }
  // nothing happens after setup
  //do sensor recording and write datafile
  char data[20];
  if (mpu.update())
    {
      roll = mpu.getYaw();
      
      
    }
    
    dtostrf(roll, 4, 3, data);
    
    Serial.println(data);
    myFile.println(roll);

}