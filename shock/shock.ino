uint8_t PIN_SHOCK = 14;

volatile bool isShocked = false;

// Setup
void setup() {
  //for debugging
  Serial.begin(9600);

  pinMode(PIN_SHOCK, INPUT);
}

void loop() {
  //empty
  isShocked = digitalRead(PIN_SHOCK);

  Serial.println(isShocked);
  delay(100);
}


07:16:31.152 -> E (24535) task_wdt: Task watchdog got triggered. The following tasks/users did not reset the watchdog in time:
07:16:31.280 -> E (24535) task_wdt:  - loopTask (CPU 1)
07:16:31.312 -> E (24535) task_wdt: Tasks currently running:
07:16:31.344 -> E (24535) task_wdt: CPU 0: Task0
07:16:31.409 -> E (24535) task_wdt: CPU 1: loopTask
07:16:31.441 -> E (24535) task_wdt: Print CPU 1 backtrace

