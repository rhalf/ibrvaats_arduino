#include <Arduino.h>
#include <Wire.h>

#include <ArduinoNmeaParser.h>

#include <U8g2lib.h>
#if defined(ESP32)
#include <WiFi.h>
#include <esp_now.h>
#include "esp_wifi.h"
#include <ESP32Ping.h>
#endif

#include <Firebase_ESP_Client.h>
#include "addons/TokenHelper.h"
#include "addons/RTDBHelper.h"


#define WIFI_SSID "caacbay.net"  //Lativo (Boarding)
#define WIFI_PASSWORD "g98j3Q1BIF2g"    //lemonjuice5

#define FIREBASE_PROJECT_ID "ibrvaats"
#define API_KEY "AIzaSyCu8ajKv3lyXT60rlo-1NOuEd4J7KrOY40"
#define USER_EMAIL "tapicglambert20@gmail.com"
#define USER_PASSWORD "123456"

FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;

//Watchdog
#include <esp_task_wdt.h>


// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
U8G2_SSD1306_128X64_NONAME_1_HW_I2C
u8g2(U8G2_R0, /*reset=*/U8X8_PIN_NONE, /*clock=*/SCL, /*data=*/SDA);

uint8_t GPS_RXD2 = 16;
uint8_t GPS_TXD2 = 17;
uint8_t PIN_SHOCK = 14;
uint8_t PIN_ACC = 27;
const int PIN_BUZZER = 5;

TaskHandle_t Task1;
TaskHandle_t Task2;

volatile double longitude = 0;
volatile double latitude = 0;
volatile double speed = 0;
volatile double course = 0;
volatile int satellite = 0;
volatile bool isShocked = false;
volatile bool isConnected = false;
volatile bool isGpsFixed = false;
volatile bool isIgnition = false;

String date = "2020-01-01T00:00:00Z";
String MAC_ADDRESS = WiFi.macAddress();

String padLeft(int value) {
  if (value < 10) return "0" + String(value);
  else String(value);
}

// GPS Datas
void onRmcUpdate(nmea::RmcData const rmc) {
  if (rmc.is_valid) {
    longitude = rmc.longitude;
    latitude = rmc.latitude;
    speed = rmc.speed;
    course = rmc.course;
    date = String(rmc.date.year) + "-" + String(rmc.date.month) + "-" + String(rmc.date.day) + "T" + String(rmc.time_utc.hour) + ":" + String(rmc.time_utc.minute) + ":" + String(rmc.time_utc.second) + "Z";
  }
}

void onGgaUpdate(nmea::GgaData const gga) {
  if (gga.fix_quality != nmea::FixQuality::Invalid) {
    satellite = gga.num_satellites;
  }
}
ArduinoNmeaParser parser(onRmcUpdate, onGgaUpdate);


// Watchdog
void watchdogInit() {
  esp_task_wdt_config_t config = {
    .timeout_ms = 10000,
    .trigger_panic = false,
  };
  esp_task_wdt_reconfigure(&config);  //enable panic so ESP32 restarts
  esp_task_wdt_add(nullptr);          //add current thread to WDT watch
}

// Wifi
void wifiInit() {
  WiFi.mode(WIFI_STA);

  Serial.println("Wifi starting!");
  while (!WiFi.STA.started()) {
    esp_task_wdt_reset();
    Serial.print(".");
    delay(500);
  }
  Serial.println();

  Serial.println("Connecting to Wi-Fi");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    esp_task_wdt_reset();
    Serial.print(".");
    delay(500);
  }
  Serial.println();

  Serial.print("Connected with IP: ");
  Serial.println(WiFi.localIP());

  bool isPinged = false;
  while (!Ping.ping("www.google.com", 3)) {
    esp_task_wdt_reset();
    Serial.print(".");
    delay(500);
  }
  Serial.println();

  Serial.println("Ping www.google.com successful!");

  MAC_ADDRESS = WiFi.macAddress();
  Serial.println();
}

// Firebase
void firebaseInit() {
  Serial.printf("Firebase Client v%s\n\n", FIREBASE_CLIENT_VERSION);

  config.api_key = API_KEY;
  auth.user.email = USER_EMAIL;
  auth.user.password = USER_PASSWORD;

  config.token_status_callback = tokenStatusCallback;

  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);
}

// Display
void displayInit() {
  u8g2.begin();
  u8g2.enableUTF8Print();
  u8g2.setFont(u8g2_font_profont11_tr);
}

void taskInit() {
  //create a task that will be executed in the Task1code() function, with priority 1 and executed on core 0
  xTaskCreatePinnedToCore(
    Task1code, /* Task function. */
    "Task1",   /* name of task. */
    10000,     /* Stack size of task */
    NULL,      /* parameter of the task */
    1,         /* priority of the task */
    &Task1,    /* Task handle to keep track of created task */
    0);        /* pin task to core 0 */
  delay(100);

  //create a task that will be executed in the Task2code() function, with priority 1 and executed on core 1
  xTaskCreatePinnedToCore(
    Task2code, /* Task function. */
    "Task2",   /* name of task. */
    10000,     /* Stack size of task */
    NULL,      /* parameter of the task */
    1,         /* priority of the task */
    &Task2,    /* Task handle to keep track of created task */
    1);        /* pin task to core 1 */
  delay(100);
}

// Setup
void setup() {
  //for debugging
  Serial.begin(9600);
  Serial2.begin(9600, SERIAL_8N1, GPS_RXD2, GPS_TXD2);
  pinMode(PIN_SHOCK, INPUT);
  pinMode(PIN_ACC, INPUT);
  pinMode(PIN_BUZZER, OUTPUT);

  watchdogInit();
  wifiInit();
  firebaseInit();
  displayInit();
  taskInit();
}

void loop() {
  //empty
}

void Task1code(void* pvParameters) {
  while (true) {
    loopTask1();
  }
}

//Task2code: Read Sensor then Firebase
void Task2code(void* pvParameters) {

  while (true) {
    loopTask2();
  }
}

void buzz() {
  digitalWrite(PIN_BUZZER, HIGH);
  delay(50);

  digitalWrite(PIN_BUZZER, LOW);
  delay(50);
}

void sendData() {
  buzz();
  while (Serial2.available()) {
    parser.encode((char)Serial2.read());
  }
  String documentPath_ID = "datas";
  FirebaseJson content;

  content.set("fields/id/stringValue", MAC_ADDRESS);
  content.set("fields/date/timestampValue", date);
  content.set("fields/longitude/doubleValue", longitude);
  content.set("fields/latitude/doubleValue", latitude);
  content.set("fields/speed/doubleValue", speed);
  content.set("fields/course/doubleValue", course);
  content.set("fields/shock/booleanValue", isShocked);
  content.set("fields/satellite/integerValue", satellite);
  content.set("fields/ignition/booleanValue", isIgnition);
  content.set("fields/gpsFixed/booleanValue", isGpsFixed);
  bool result = Firebase.Firestore.createDocument(&fbdo, FIREBASE_PROJECT_ID, "", documentPath_ID.c_str(), content.raw());

  if (result) {
    Serial.println("Added a new document!");
  } else {
    Serial.println("Create document failed.");
    Serial.println(fbdo.errorReason());
  }

  buzz();
  buzz();

  delay(2000);
}

void loopTask1() {
  isShocked = digitalRead(PIN_SHOCK);
  isIgnition = !digitalRead(PIN_ACC);
}


void loopTask2() {
  while (Serial2.available()) {
    parser.encode((char)Serial2.read());
  }

  u8g2.firstPage();
  do {
    u8g2.setCursor(0, 10);
    u8g2.println(String(MAC_ADDRESS));
    u8g2.setCursor(0, 20);
    u8g2.println("Date: " + String(date));
    u8g2.setCursor(0, 30);
    u8g2.println("Satelllite : " + String(satellite));
    u8g2.setCursor(0, 40);
    u8g2.println("Speed: " + String(speed));
    u8g2.setCursor(0, 50);
    u8g2.println("Shock: " + String(isShocked));
    u8g2.setCursor(0, 60);
    u8g2.println("Ignition: " + String(isIgnition));

  } while (u8g2.nextPage());

  if (isShocked) sendData();
}