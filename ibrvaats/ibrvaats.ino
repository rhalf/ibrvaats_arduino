#include <ArduinoNmeaParser.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#define SCREEN_WIDTH 128  // OLED display width, in pixels
#define SCREEN_HEIGHT 64  // OLED display height, in pixels
#include <Arduino.h>
#if defined(ESP32)
#include <WiFi.h>
#include <ESP32Ping.h>
#elif defined(ESP8266)
#include <ESP8266WiFi.h>
#endif
#include <Firebase_ESP_Client.h>
#include "addons/TokenHelper.h"
#include "addons/RTDBHelper.h"
#define WIFI_SSID "caacbay.net"
#define WIFI_PASSWORD "g98j3Q1BIF2g"
#define FIREBASE_PROJECT_ID "esp32firestore-60045"
#define API_KEY "AIzaSyBDcQ3ZAh0kYynLzRDGT6xC9-Ws6S0cQsU"
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
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

uint8_t GPS_RXD2 = 16;
uint8_t GPS_TXD2 = 17;
uint8_t PIN_SHOCK = 14;

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
    /*
    Serial.print(" | HDOP =  ");
    Serial.print(gga.hdop);
    Serial.print(" m | Altitude ");
    Serial.print(gga.altitude);
    Serial.print(" m | Geoidal Separation ");
    Serial.print(gga.geoidal_separation);
    Serial.print(" m");*/
  }
}

ArduinoNmeaParser parser(onRmcUpdate, nullptr);

void setup() {
  //for debugging
  Serial.begin(9600);
  Serial2.begin(9600);
  pinMode(PIN_SHOCK, INPUT);

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

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {  // Address 0x3D for 128x64
    Serial.println(F("SSD1306 allocation failed"));
    for (;;)
      ;
  }

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

//Task1code: Read GPS and Display
void Task1code(void* pvParameters) {
  while (true) loopTask1();
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

  String documentPath_ID = "ESP32/"
                           ""
                           + Mac_Address;
  FirebaseJson content;

  content.set("fields/longitude/stringValue", String(longitude, 2));
  content.set("fields/latitude/stringValue", String(latitude, 2));
  content.set("fields/speed/stringValue", String(speed, 2));
  content.set("fields/course/stringValue", String(course, 2));
  content.set("fields/shock/stringValue", String(isShocked));
  content.set("fields/date/stringValue", String(date));
  content.set("fields/satellite/stringValue", String(satellite));

  if (Firebase.Firestore.patchDocument(&fbdo, FIREBASE_PROJECT_ID, "", documentPath_ID.c_str(), content.raw(), "longitude")) {
    Serial.printf("ok\n%s\n\n", fbdo.payload().c_str());
  }
  if (Firebase.Firestore.patchDocument(&fbdo, FIREBASE_PROJECT_ID, "", documentPath_ID.c_str(), content.raw(), "latitude")) {
    Serial.printf("ok\n%s\n\n", fbdo.payload().c_str());
  }
  if (Firebase.Firestore.patchDocument(&fbdo, FIREBASE_PROJECT_ID, "", documentPath_ID.c_str(), content.raw(), "speed")) {
    Serial.printf("ok\n%s\n\n", fbdo.payload().c_str());
  }
  if (Firebase.Firestore.patchDocument(&fbdo, FIREBASE_PROJECT_ID, "", documentPath_ID.c_str(), content.raw(), "course")) {
    Serial.printf("ok\n%s\n\n", fbdo.payload().c_str());
  }
  if (Firebase.Firestore.patchDocument(&fbdo, FIREBASE_PROJECT_ID, "", documentPath_ID.c_str(), content.raw(), "shock")) {
    Serial.printf("ok\n%s\n\n", fbdo.payload().c_str());
  }
  if (Firebase.Firestore.patchDocument(&fbdo, FIREBASE_PROJECT_ID, "", documentPath_ID.c_str(), content.raw(), "date")) {
    Serial.printf("ok\n%s\n\n", fbdo.payload().c_str());
  }
  if (Firebase.Firestore.patchDocument(&fbdo, FIREBASE_PROJECT_ID, "", documentPath_ID.c_str(), content.raw(), "satellite")) {
    Serial.printf("ok\n%s\n\n", fbdo.payload().c_str());
  }

  delay(10000);
}

void loopTask2() {
  isShocked = digitalRead(PIN_SHOCK);

  //gps
  while (Serial2.available()) {
    parser.encode((char)Serial2.read());
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  display.println("Longitude : " + String(longitude));
  display.setCursor(0, 11);
  display.println("Latitude : " + String(latitude));
  display.setCursor(0, 22);
  display.println("Speed : " + String(speed));
  display.setCursor(0, 33);
  display.println("Course : " + String(course));
  display.setCursor(0, 44);
  display.println(date);
  display.setCursor(0, 55);
  display.println("Shock : " + String(isShocked));


  // display.setCursor(0, 55);
  //display.println("Time : " +  String(time));


  display.display();
}