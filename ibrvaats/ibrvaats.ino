// #include <Arduino.h>
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


//#define WIFI_SSID "Lativo (Boarding)"
//#define WIFI_PASSWORD "lemonjuice5"
#define WIFI_SSID "caacbay.net"       //Lativo (Boarding)
#define WIFI_PASSWORD "g98j3Q1BIF2g"  //lemonjuice5

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

TaskHandle_t TaskDisplay;
TaskHandle_t TaskSensor;
TaskHandle_t TaskWifi;

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
String orientation = "TOP";

String padLeft(int value) {
  if (value < 10) return "0" + String(value);
  else String(value);
}

// bool ping() {
//   return Ping.ping("www.google.com", 3);
// }

// GPS Datas
void onRmcUpdate(nmea::RmcData const rmc) {
  if (rmc.is_valid) {
    longitude = isnan(rmc.longitude) ? 0 : rmc.longitude;
    ;
    latitude = isnan(rmc.latitude) ? 0 : rmc.latitude;
    speed = isnan(rmc.speed) ? 0 : rmc.speed;
    course = isnan(rmc.course) ? 0 : rmc.course;
    date = String(rmc.date.year) + "-" + String(rmc.date.month) + "-" + String(rmc.date.day) + "T" + String(rmc.time_utc.hour) + ":" + String(rmc.time_utc.minute) + ":" + String(rmc.time_utc.second) + "Z";
  }
}

void onGgaUpdate(nmea::GgaData const gga) {
  if (gga.fix_quality != nmea::FixQuality::Invalid) {
    satellite = gga.num_satellites;
    isGpsFixed = true;
  } else {
    isGpsFixed = false;
    satellite = 0;
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



// Firebase
void firebaseInit() {
  // Serial.printf("Firebase Client v%s\n\n", FIREBASE_CLIENT_VERSION);

  config.api_key = API_KEY;
  auth.user.email = USER_EMAIL;
  auth.user.password = USER_PASSWORD;

  config.token_status_callback = tokenStatusCallback;

  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);

  Serial.printf("Connected to Firebase");
}



void taskInit() {
  xTaskCreate(
    taskDisplay,   /* Task function. */
    "TaskDisplay", /* name of task. */
    10000,         /* Stack size of task */
    NULL,          /* parameter of the task */
    3,             /* priority of the task */
    &TaskDisplay   /* Task handle to keep track of created task */
  );

  xTaskCreate(
    taskSensor,   /* Task function. */
    "TaskSensor", /* name of task. */
    10000,        /* Stack size of task */
    NULL,         /* parameter of the task */
    1,            /* priority of the task */
    &TaskSensor   /* Task handle to keep track of created task */
  );              

  xTaskCreate(
    taskWifi,   /* Task function. */
    "TaskWifi", /* name of task. */
    10000,      /* Stack size of task */
    NULL,       /* parameter of the task */
    1,          /* priority of the task */
    &TaskWifi   /* Task handle to keep track of created task*/
  );          
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

  taskInit();

  firebaseInit();
}

void loop() {
  //empty
}

//taskWifi: connect to wifi
void taskWifi(void* pvParameters) {
  Serial.println("WIFI: starting!");
  WiFi.mode(WIFI_STA);

  for (;;) {
    delay(1000);

    if (!WiFi.STA.started()) delay(500);

    if (WiFi.status() == WL_CONNECTED) continue;

    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    Serial.println("WIFI: Connecting...");
    while (WiFi.status() != WL_CONNECTED) delay(500);

    Serial.print("WIFI: Connected with IP: ");
    Serial.println(WiFi.localIP());

    MAC_ADDRESS = WiFi.macAddress();
  }
}

void taskDisplay(void* pvParameters) {
  u8g2.begin();
  u8g2.enableUTF8Print();
  u8g2.setFont(u8g2_font_profont11_tr);

  for (;;) {
    delay(100);

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
  }
}

//taskSensor: Read Sensor then Firebase
void taskSensor(void* pvParameters) {
  for (;;) {

    if (!isShocked && digitalRead(PIN_SHOCK)) {
      Serial.println("SENSOR: Shock!");
    }
    isShocked = digitalRead(PIN_SHOCK);

    if (!isIgnition && !digitalRead(PIN_ACC)) {
      Serial.println("SENSOR: Acc!");
    }
    isIgnition = !digitalRead(PIN_ACC);
  }
}

void buzz() {
  digitalWrite(PIN_BUZZER, HIGH);
  vTaskDelay(50 / portTICK_PERIOD_MS);

  digitalWrite(PIN_BUZZER, LOW);
  vTaskDelay(50 / portTICK_PERIOD_MS);
}

void sendData() {
  buzz();

  if (WiFi.status() == WL_CONNECTED) return;

  buzz();
  String documentPath_ID = "datas";
  FirebaseJson content;

  content.set("fields/mac/stringValue", MAC_ADDRESS);
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
    Serial.print("########################");
    Serial.println("Added a new document!");
  } else {
    Serial.print("########################");
    Serial.println("Error!");
    Serial.println(fbdo.errorReason());
  }

  buzz();
  buzz();

  delay(10000);
  // esp_task_wdt_reset();
}




// void loopTask1() {
//   while (Serial2.available()) {
//     parser.encode((char)Serial2.read());
//   }



//   if (isShocked) {
//     sendData();
//     isShocked = false;
//   }
// }