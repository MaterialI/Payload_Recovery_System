#include <Wire.h>
#include <MPU9250.h>

MPU9250 mpu;

void setup() {
  Wire.begin();
  Serial.begin(9600);

  // Initialize the MPU9250 sensor
  if (!mpu.setup(0x68)) {
    Serial.println("MPU9250 initialization failed. Check wiring or try resetting the Arduino.");
    while (1);
  }

}

void loop() {
    if (mpu.update()) {
        Serial.print(mpu.getYaw()); Serial.print(", ");
        Serial.print(mpu.getPitch()); Serial.print(", ");
        Serial.println(mpu.getRoll());
    }
}