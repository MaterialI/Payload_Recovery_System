const int inputPin = 2;  // Replace with the actual pin number connected to the square wave signal
const int switchPin = 13; // Replace with the actual pin number connected to your switch

void setup() {
  pinMode(inputPin, INPUT);
  pinMode(switchPin, OUTPUT);
  digitalWrite(switchPin, LOW);
}
bool wave = false;
void loop() {
  if (digitalRead(inputPin) == HIGH || wave) 
  {
    digitalWrite(switchPin, HIGH); // Activate the switch on the first high pulse
    wave = true;
  }
}