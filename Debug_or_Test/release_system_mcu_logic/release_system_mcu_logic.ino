// #include <Servo.h>
// const int inputPin = 2;  // Replace with the actual pin number connected to the square wave signal
// const int switchPin = 13; // Replace with the actual pin number connected to your switch

// void setup() {
//   pinMode(inputPin, INPUT);
//   pinMode(switchPin, OUTPUT);
//   digitalWrite(switchPin, LOW);
// }
// bool wave = false;
// void loop() {
//   if (digitalRead(inputPin) == HIGH || wave) 
//   {
//     digitalWrite(switchPin, HIGH); // Activate the switch on the first high pulse
//     wave = true;
//   }
// }
#include <Servo.h>

Servo myServo;  // Create a Servo object

void setup() {
  myServo.attach(A7);  // Attach the servo to pin A7
}

void loop() {
  myServo.write(0);  // Move the servo to the 90-degree position
  delay(10000);        // Wait for 1 second
  myServo.write(90); // Move the servo to the 180-degree position
  delay(10000);        // Wait for 1 second
}