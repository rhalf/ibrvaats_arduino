const int PIN = 2;

void setup() {
  // initialize digital pin GPIO18 as an output.
  pinMode(PIN, OUTPUT);
}

// the loop function runs over and over again forever
void loop() {
  digitalWrite(PIN, HIGH); // turn the LED on
  delay(500);             // wait for 500 milliseconds
  digitalWrite(PIN, LOW);  // turn the LED off
  delay(500);             // wait for 500 milliseconds
}