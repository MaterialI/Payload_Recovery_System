
#include <MPU9250.h>
#include <quaternionFilters.h>

#include <TeensyThreads.h>
#include <Wire.h>

#include <PID_v1.h> //PID 
// TwoWire I2C2;
MPU9250 mpu;
float yaw = 0.0;
double inp = 0.0;
double goal = 20.69;
double output = 0.0;
//PID coefficients
double cP = 1;
double cI = 0.5;
double cD = 0.5;

PID courseCorrection(&inp, &output, &goal, cP, cI, cD, DIRECT);

void setup() {
  // I2C_Init()
  // I2C2.begin(25, 24, 100000);
  Serial.begin(9600);
  Wire.begin();
  Wire2.begin();
  delay(2000);
 
    //set the setting for IMU module
  MPU9250Setting setting;
  setting.accel_fs_sel = ACCEL_FS_SEL::A16G;
  setting.gyro_fs_sel = GYRO_FS_SEL::G2000DPS;
  setting.mag_output_bits = MAG_OUTPUT_BITS::M16BITS;
  setting.fifo_sample_rate = FIFO_SAMPLE_RATE::SMPL_200HZ;
  setting.gyro_fchoice = 0x03;
  setting.gyro_dlpf_cfg = GYRO_DLPF_CFG::DLPF_41HZ;
  setting.accel_fchoice = 0x01;
  setting.accel_dlpf_cfg = ACCEL_DLPF_CFG::DLPF_45HZ;

  if(mpu.setup(0x68, setting, Wire2) == false)
    Serial.println("Not initialized");
  // if(!Wire.begin(0x68))
  // Serial.println("Not initialized");
  courseCorrection.SetMode(AUTOMATIC);
}
void loop() {
  
  if(mpu.update())
  {
    yaw = mpu.getYaw();
    
  }
  inp = yaw;
  Serial.println(yaw);
  courseCorrection.Compute();
  Serial.print("PID out: ");
  Serial.println(output);
  // update_accel_gyro();
  // Serial.print("Roll (deg): ");
  // Serial.println(a[0]);
  // Serial.print("Pitch (deg): ");
  // Serial.println(a[1]);
  // Serial.print("Yaw (deg): ");
  // Serial.println(a[2]);
  }