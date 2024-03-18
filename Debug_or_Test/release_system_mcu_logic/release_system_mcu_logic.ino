#include <Servo.h>
const int inputPin =5; // A6 // Replace with the actual pin number connected to the square wave signal
const int servoPin = 3; // Replace with the actual pin number connected to your switch
int pos = 0;
int finPos = 77;
Servo myservo;  // Create a servo object
void setup() {
  
  myservo.attach(servoPin);

  myservo.write(pos);
  delay(22000);
  {
    // digitalWrite(A6_PIN, HIGH); // Set A6_PIN to high (logical 1)
    
    myservo.write(77);
  }
}
bool wave = false;
void loop() {
  
}
