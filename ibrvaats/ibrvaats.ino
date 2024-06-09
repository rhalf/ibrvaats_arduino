#include <ArduinoNmeaParser.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <Arduino.h>
#if defined(ESP32)
#include <WiFi.h>
#include <esp_now.h>
#include "esp_wifi.h"
#include <ESP32Ping.h>
#endif
#include <Firebase_ESP_Client.h>
#include "addons/TokenHelper.h"
#include "addons/RTDBHelper.h"
#define WIFI_SSID "caacbay.net"
#define WIFI_PASSWORD "g98j3Q1BIF2g"
#define FIREBASE_PROJECT_ID "ibrvaats"
#define API_KEY "AIzaSyCu8ajKv3lyXT60rlo-1NOuEd4J7KrOY40"
#define USER_EMAIL "tapicglambert20@gmail.com"
#define USER_PASSWORD "123456"

FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;

unsigned long sendDataPrevMillis = 0;
int intValue;
float floatValue;
bool signupOK = false;

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
U8G2_SSD1306_128X64_NONAME_1_HW_I2C
u8g2(U8G2_R0, /*reset=*/U8X8_PIN_NONE, /*clock=*/SCL, /*data=*/SDA);

uint8_t GPS_RXD2 = 17;
uint8_t GPS_TXD2 = 16;
uint8_t PIN_SHOCK = 14;
uint8_t PIN_ACC = 27;
const int PIN_BUZZER = 5;

TaskHandle_t Task1;
TaskHandle_t Task2;

volatile double longitude = 0;
volatile double latitude = 0;
volatile double speed = 0;
volatile double course = 0;
volatile double satellite = 0;
volatile bool isShocked = false;
volatile bool isConnected = false;
volatile bool isGpsFixed = false;
volatile bool isIgnition = false;

String date = "";
String Mac_Address = WiFi.macAddress();

void onRmcUpdate(nmea::RmcData const rmc) {
  if (rmc.is_valid) {
    longitude = rmc.longitude;
    latitude = rmc.latitude;
    speed = rmc.speed;
    course = rmc.course;
    date = String(rmc.date.year) + " " + String(rmc.date.month) + " " + String(rmc.date.day);
  }
}
void onGgaUpdate(nmea::GgaData const gga) {
  if (gga.fix_quality != nmea::FixQuality::Invalid) {
    satellite = gga.num_satellites;
  }
}
ArduinoNmeaParser parser(onRmcUpdate, nullptr);

void setup() {
  //for debugging
  Serial.begin(9600);
  Serial2.begin(9600);
  uint8_t CustomMACaddress[] = { 0xB0, 0xA7, 0x32, 0x12, 0x34, 0x56 };
  WiFi.mode(WIFI_STA);
  esp_wifi_set_mac(WIFI_IF_STA, &CustomMACaddress[0]);  //set custom mac address
  while (!WiFi.STA.started()) {
    delay(100);
  }
  Mac_Address = WiFi.macAddress();


  pinMode(PIN_SHOCK, INPUT);
  pinMode(PIN_ACC, INPUT);
  pinMode(PIN_BUZZER, OUTPUT);

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(300);
  }
  delay(200);
  bool isSuccess = Ping.ping("www.google.com", 3);
  if (!isSuccess) {
    Serial.println("Ping failed");
    return;
  }
  Serial.println("Ping succesful.");
  Serial.println();
  Serial.print("Connected with IP: ");
  Serial.println(WiFi.localIP());
  Serial.println();
  Serial.printf("Firebase Client v%s\n\n", FIREBASE_CLIENT_VERSION);

  config.api_key = API_KEY;
  auth.user.email = USER_EMAIL;
  auth.user.password = USER_PASSWORD;

  config.token_status_callback = tokenStatusCallback;

  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);

  u8g2.begin();
  u8g2.enableUTF8Print();
  u8g2.setFont(u8g2_font_profont11_tr);

  //create a task that will be executed in the Task1code() function, with priority 1 and executed on core 0
  xTaskCreatePinnedToCore(
    Task1code, /* Task function. */
    "Task1",   /* name of task. */
    10000,     /* Stack size of task */
    NULL,      /* parameter of the task */
    1,         /* priority of the task */
    &Task1,    /* Task handle to keep track of created task */
    0);        /* pin task to core 0 */
  delay(500);

  //create a task that will be executed in the Task2code() function, with priority 1 and executed on core 1
  xTaskCreatePinnedToCore(
    Task2code, /* Task function. */
    "Task2",   /* name of task. */
    10000,     /* Stack size of task */
    NULL,      /* parameter of the task */
    1,         /* priority of the task */
    &Task2,    /* Task handle to keep track of created task */
    1);        /* pin task to core 1 */
  delay(500);
}

void Task1code(void* pvParameters) {
  while (true)
    if (isShocked == 1) {
      digitalWrite(PIN_BUZZER, HIGH);
      loopTask1();
    } else {
      digitalWrite(PIN_BUZZER, LOW);
    }
}

//Task2code: Read Sensor then Firebase
void Task2code(void* pvParameters) {
  while (true) loopTask2();
}

void loop() {
  //empty
}

void loopTask1() {
  while (Serial2.available()) {
    parser.encode((char)Serial2.read());
  }

  String documentPath_ID = "datas/"
                           ""
                           + Mac_Address;
  FirebaseJson content;

  content.set("fields/longitude/stringValue", String(longitude, 2));
  content.set("fields/latitude/stringValue", String(latitude, 2));
  content.set("fields/speed/stringValue", String(speed, 2));
  content.set("fields/course/stringValue", String(course, 2));
  content.set("fields/shock/stringValue", String(isShocked ? "true" : "false"));
  content.set("fields/date/stringValue", String(date));
  content.set("fields/satellite/stringValue", String(satellite));
  content.set("fields/ignition/stringValue", String(isIgnition ? "true" : "false"));

  if (Firebase.Firestore.patchDocument(&fbdo, FIREBASE_PROJECT_ID, "", documentPath_ID.c_str(), content.raw(), "longitude")) {
  }
  if (Firebase.Firestore.patchDocument(&fbdo, FIREBASE_PROJECT_ID, "", documentPath_ID.c_str(), content.raw(), "latitude"))
}
if (Firebase.Firestore.patchDocument(&fbdo, FIREBASE_PROJECT_ID, "", documentPath_ID.c_str(), content.raw(), "speed")) {
}
if (Firebase.Firestore.patchDocument(&fbdo, FIREBASE_PROJECT_ID, "", documentPath_ID.c_str(), content.raw(), "course")) {
}
if (Firebase.Firestore.patchDocument(&fbdo, FIREBASE_PROJECT_ID, "", documentPath_ID.c_str(), content.raw(), "shock")) {
}
if (Firebase.Firestore.patchDocument(&fbdo, FIREBASE_PROJECT_ID, "", documentPath_ID.c_str(), content.raw(), "date")) {
}
if (Firebase.Firestore.patchDocument(&fbdo, FIREBASE_PROJECT_ID, "", documentPath_ID.c_str(), content.raw(), "satellite")) {
}
if (Firebase.Firestore.patchDocument(&fbdo, FIREBASE_PROJECT_ID, "", documentPath_ID.c_str(), content.raw(), "ignition")) {
}

delay(10000);
}

void loopTask2() {
  isShocked = digitalRead(PIN_SHOCK);
  isIgnition = digitalRead(PIN_ACC);

  //gps
  while (Serial2.available()) {
    parser.encode((char)Serial2.read());
  }

  u8g2.firstPage();
  do {
    u8g2.setCursor(0, 0);
    u8g2.println("Longitude : " + String(longitude));
    u8g2.setCursor(0, 11);
    u8g2.println("Latitude : " + String(latitude));
    u8g2.setCursor(0, 22);
    u8g2.println("Speed : " + String(speed));
    u8g2.setCursor(0, 33);
    u8g2.println("Course : " + String(course));
    u8g2.setCursor(0, 44);
    u8g2.println(date);
    u8g2.setCursor(0, 55);
    u8g2.println("Shock : " + String(isShocked ? "true" : "false"));
  } while (u8g2.nextPage());
}