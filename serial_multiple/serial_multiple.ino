
#include <Timer.h>

Timer timer0(Timer::MILLIS);


void callback0() {
  Serial2.println("Serial2");
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, 16, 17);
  //Hardware Serial of ESP32


  //Timers
  timer0.begin(Timer::FOREVER, 3000, callback0);

  timer0.start();
}



void loop() {
  timer0.run();
}


void serialEvent() {
  if (Serial.available()) {
    Serial.write(Serial.read());
  }
}
