#include <Wire.h> //wire library to communicate
#include <FeedBackServo.h>
#include <MPU9250.h> //Gyro, accel


#define FEEDBACK_PIN 7
#define SERVO_PIN 8

MPU9250 mpu; //gyro, accel, compass

FeedBackServo servo = FeedBackServo(FEEDBACK_PIN);

void setup() {
  // put your setup code here, to run once:
   Serial.begin(9600); 
  Wire2.begin();
  Wire.begin(); 
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
  servo.setServoControl(SERVO_PIN);
  servo.setKp(1.0);

  mpu.setup(0x68, setting, Wire2);
}
int yaw = 0.0;
void loop() {
  // put your main code here, to run repeatedly:
  yaw += 1;
  yaw = yaw % 360;
  servo.rotate((int)(yaw),4);
  Serial.print("Servo Angle: ");
  Serial.println(servo.Angle());


}
