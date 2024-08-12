// #include <Arduino.h>
#include <Wire.h>

#include <ArduinoNmeaParser.h>

#include <U8g2lib.h>
#include <WiFi.h>

#include <esp_now.h>
#include "esp_wifi.h"

#include <Firebase_ESP_Client.h>
#include "addons/TokenHelper.h"
#include "addons/RTDBHelper.h"


#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
Adafruit_MPU6050 mpu;

// #include <NTPClient.h>
// #include <WiFiUdp.h>

// WiFiUDP ntpUDP;
// NTPClient timeClient(ntpUDP);

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
uint8_t PIN_FRONT = 33;
uint8_t PIN_REAR = 25;

const int PIN_BUZZER = 5;

TaskHandle_t TaskDisplay;
TaskHandle_t TaskSensor;
TaskHandle_t TaskWifi;
TaskHandle_t TaskFirebase;
TaskHandle_t TaskGps;
TaskHandle_t TaskUpload;
TaskHandle_t TaskMpu6050;

volatile double longitude = 0;
volatile double latitude = 0;
volatile double speed = 0;
volatile double course = 0;
volatile double temperature = 0;

volatile double accelX = 0;
volatile double accelY = 0;
volatile double accelZ = 0;
volatile double gyroX = 0;
volatile double gyroY = 0;
volatile double gyroZ = 0;

volatile int satellite = 0;
volatile bool isShocked = false;
volatile bool isFront = false;
volatile bool isRear = false;
volatile bool isConnected = false;
volatile bool isGpsFixed = false;
volatile bool isIgnition = false;

String gpsTime = "2020-01-01T00:00:00Z";
// String serverTime = "2020-01-01T00:00:00Z";

String MAC_ADDRESS = WiFi.macAddress();
String PLATE_NUMER = "AAA1234";
String orientation = "TOP";

//====================================================================== Functions
void buzz(uint8_t times = 1) {
  for (uint8_t t = 0; t < times; t++) {
    digitalWrite(PIN_BUZZER, HIGH);
    vTaskDelay(50 / portTICK_PERIOD_MS);
    digitalWrite(PIN_BUZZER, LOW);
    vTaskDelay(50 / portTICK_PERIOD_MS);
  }
}

String padLeft(int value) {
  if (value < 10) return "0" + String(value);
  else String(value);
}

String toTimestamp(int year, int month, int day, int hour, int minute, int second) {
  return String(year) + "-" + String(month) + "-" + String(day) + "T" + String(hour) + ":" + String(minute) + ":" + String(second) + "Z";
}


//====================================================================== GPS Datas
void onRmcUpdate(nmea::RmcData const rmc) {
  if (rmc.is_valid) {
    longitude = isnan(rmc.longitude) ? 0 : rmc.longitude;
    latitude = isnan(rmc.latitude) ? 0 : rmc.latitude;
    speed = isnan(rmc.speed) ? 0 : rmc.speed;
    course = isnan(rmc.course) ? 0 : rmc.course;
    gpsTime = toTimestamp(rmc.date.year, rmc.date.month, rmc.date.day, rmc.time_utc.hour, rmc.time_utc.minute, rmc.time_utc.second);
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

//====================================================================== Watchdog
void watchdogInit() {
  esp_task_wdt_config_t config = {
    .timeout_ms = 10000,
    .trigger_panic = false,
  };
  esp_task_wdt_reconfigure(&config);  //enable panic so ESP32 restarts

  // esp_task_wdt_add(TaskDisplay);   //add current thread to WDT watch
  // esp_task_wdt_add(TaskSensor);    //add current thread to WDT watch
  // esp_task_wdt_add(TaskWifi);      //add current thread to WDT watch
  // esp_task_wdt_add(TaskFirebase);  //add current thread to WDT watch
  // esp_task_wdt_add(TaskGps);       //add current thread to WDT watch
  // esp_task_wdt_add(TaskUpload);    //add current thread to WDT watch

  esp_task_wdt_delete(TaskDisplay);   //delete current thread to WDT watch
  esp_task_wdt_delete(TaskSensor);    //delete current thread to WDT watch
  esp_task_wdt_delete(TaskWifi);      //delete current thread to WDT watch
  esp_task_wdt_delete(TaskFirebase);  //delete current thread to WDT watch
  esp_task_wdt_delete(TaskGps);       //delete current thread to WDT watch
  esp_task_wdt_delete(TaskUpload);    //delete current thread to WDT watch
  esp_task_wdt_delete(TaskMpu6050);   //delete current thread to WDT watch
}


//====================================================================== TaskInit
void taskInit() {
  xTaskCreate(
    taskWifi,   /* Task function. */
    "TaskWifi", /* name of task. */
    10000,      /* Stack size of task */
    NULL,       /* parameter of the task */
    1,          /* priority of the task */
    &TaskWifi   /* Task handle to keep track of created task*/
  );

  xTaskCreate(
    taskFirebase,   /* Task function. */
    "taskFirebase", /* name of task. */
    10000,          /* Stack size of task */
    NULL,           /* parameter of the task */
    1,              /* priority of the task */
    &TaskFirebase   /* Task handle to keep track of created task*/
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
    taskGps,   /* Task function. */
    "TaskGps", /* name of task. */
    10000,     /* Stack size of task */
    NULL,      /* parameter of the task */
    2,         /* priority of the task */
    &TaskGps   /* Task handle to keep track of created task*/
  );

  xTaskCreate(
    taskDisplay,   /* Task function. */
    "TaskDisplay", /* name of task. */
    10000,         /* Stack size of task */
    NULL,          /* parameter of the task */
    3,             /* priority of the task */
    &TaskDisplay   /* Task handle to keep track of created task */
  );

  xTaskCreate(
    taskUpload,   /* Task function. */
    "TaskUpload", /* name of task. */
    10000,        /* Stack size of task */
    NULL,         /* parameter of the task */
    3,            /* priority of the task */
    &TaskUpload   /* Task handle to keep track of created task */
  );

  xTaskCreate(
    taskMpu6050,   /* Task function. */
    "TaskMpu6050", /* name of task. */
    10000,         /* Stack size of task */
    NULL,          /* parameter of the task */
    2,             /* priority of the task */
    &TaskMpu6050   /* Task handle to keep track of created task */
  );
}



void taskGps(void* pvParameters) {
  Serial2.begin(9600, SERIAL_8N1, GPS_RXD2, GPS_TXD2);

  for (;;) {
    // esp_task_wdt_reset();
    vTaskDelay(1 / portTICK_PERIOD_MS);

    while (Serial2.available()) {
      parser.encode((char)Serial2.read());
    }
  }
}

//Connect to wifi
void taskWifi(void* pvParameters) {
  Serial.println("WIFI: starting!");
  WiFi.mode(WIFI_STA);

  for (;;) {
    // esp_task_wdt_reset();
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    if (!WiFi.STA.started()) delay(500);

    if (WiFi.status() == WL_CONNECTED) continue;

    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    Serial.println("WIFI: Connecting...");
    while (WiFi.status() != WL_CONNECTED) delay(500);

    Serial.print("WIFI: Connected with IP: ");
    Serial.println(WiFi.localIP());

    MAC_ADDRESS = WiFi.macAddress();

    buzz(1);
  }
}

// Firebase
void taskFirebase(void* pvParameters) {
  // Serial.printf("Firebase Client v%s\n\n", FIREBASE_CLIENT_VERSION);
  uint8_t attempt = 0;
  for (;;) {
    // esp_task_wdt_reset();
    vTaskDelay(1 / portTICK_PERIOD_MS);

    if (WiFi.status() != WL_CONNECTED) continue;

    if (Firebase.ready()) continue;

    if (Firebase.isTokenExpired()) attempt = 0;

    if (attempt > 0) continue;

    config.api_key = API_KEY;
    auth.user.email = USER_EMAIL;
    auth.user.password = USER_PASSWORD;

    config.token_status_callback = tokenStatusCallback;

    Firebase.begin(&config, &auth);
    Firebase.reconnectWiFi(true);

    Serial.println("Connected to Firebase!");
    buzz(1);

    attempt++;
  }
}

void taskDisplay(void* pvParameters) {
  u8g2.begin();
  u8g2.enableUTF8Print();
  u8g2.setFont(u8g2_font_profont11_tr);

  for (;;) {
    // esp_task_wdt_reset();
    vTaskDelay(100 / portTICK_PERIOD_MS);

    u8g2.firstPage();
    do {
      u8g2.setCursor(0, 10);
      u8g2.println("PLATE: " + String(PLATE_NUMER));
      u8g2.setCursor(0, 20);
      u8g2.println(String(gpsTime));
      u8g2.setCursor(0, 30);
      u8g2.println("SAT : " + String(satellite) + ", " + String(isGpsFixed ? "Yes" : "No"));
      u8g2.setCursor(0, 40);
      u8g2.println("SPEED: " + String(speed));
      u8g2.setCursor(0, 50);
      u8g2.println("SHOCK: " + String(isShocked) + ", " + String(isFront) + ":" + String(isRear) + " | " + orientation);
      u8g2.setCursor(0, 60);
      u8g2.println(String(accelX, 2) + " | " + String(accelY, 2) + " | " + String(accelZ, 2));
    } while (u8g2.nextPage());
  }
}

//taskSensor: Read Sensor then Firebase
void taskSensor(void* pvParameters) {
  pinMode(PIN_SHOCK, INPUT);
  pinMode(PIN_ACC, INPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_FRONT, INPUT);
  pinMode(PIN_REAR, INPUT);

  for (;;) {
    // esp_task_wdt_reset();
    vTaskDelay(1 / portTICK_PERIOD_MS);

    if (!isShocked && digitalRead(PIN_SHOCK)) {
      Serial.println("SENSOR: Shock!");
    }
    isShocked = digitalRead(PIN_SHOCK);

    if (!isIgnition && !digitalRead(PIN_ACC)) {
      Serial.println("SENSOR: Acc!");
    }
    isIgnition = !digitalRead(PIN_ACC);

    if (!isFront && !digitalRead(PIN_FRONT)) {
      Serial.println("SENSOR: Front!");
    }
    isFront = !digitalRead(PIN_FRONT);

    if (!isRear && !digitalRead(PIN_REAR)) {
      Serial.println("SENSOR: Rear!");
    }
    isRear = !digitalRead(PIN_REAR);
  }
}

void taskUpload(void* pvParameters) {
  for (;;) {
    // esp_task_wdt_reset();
    vTaskDelay(1 / portTICK_PERIOD_MS);

    if (!isShocked) continue;

    if (!isRear && !isFront) continue;

    if (speed < 0.2) continue;

    if (WiFi.status() != WL_CONNECTED) continue;

    if (!Firebase.ready()) continue;

    buzz(2);

    Serial.println("FIREBASE: Uploading document!");

    String documentPath_ID = "datas";
    FirebaseJson content;

    content.set("fields/mac/stringValue", MAC_ADDRESS);
    content.set("fields/gpsTime/timestampValue", gpsTime);
    content.set("fields/longitude/doubleValue", longitude);
    content.set("fields/latitude/doubleValue", latitude);
    content.set("fields/speed/doubleValue", speed);

    content.set("fields/accelX/doubleValue", accelX);
    content.set("fields/accelY/doubleValue", accelY);
    content.set("fields/accelZ/doubleValue", accelZ);
    content.set("fields/gyroX/doubleValue", gyroX);
    content.set("fields/gyroY/doubleValue", gyroY);
    content.set("fields/gyroZ/doubleValue", gyroZ);
    content.set("fields/temperature/doubleValue", temperature);

    content.set("fields/course/doubleValue", course);
    content.set("fields/shock/booleanValue", isShocked);
    content.set("fields/front/booleanValue", isFront);
    content.set("fields/rear/booleanValue", isRear);
    content.set("fields/satellite/integerValue", satellite);
    content.set("fields/ignition/booleanValue", isIgnition);
    content.set("fields/gpsFixed/booleanValue", isGpsFixed);

    bool result = Firebase.Firestore.createDocument(&fbdo, FIREBASE_PROJECT_ID, "", documentPath_ID.c_str(), content.raw());

    if (result) {
      Serial.println("FIREBASE: Document Added!");
      buzz(2);
    } else {
      Serial.print("FIREBASE: Document Error!");
      Serial.println(fbdo.errorReason());
      buzz(3);
    }


    delay(3000);
  }
}

void taskMpu6050(void* pvParameters) {
  if (!mpu.begin()) {
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);

  for (;;) {
    // esp_task_wdt_reset();
    vTaskDelay(500 / portTICK_PERIOD_MS);

    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    accelX = a.acceleration.x;
    accelY = a.acceleration.y;
    accelZ = a.acceleration.z;

    gyroX = g.gyro.x;
    gyroY = g.gyro.y;
    gyroZ = g.gyro.z;

    temperature = temp.temperature;

    bool top = (accelX > -2.0 && accelX < 2.0) && (accelY > -2.0 && accelY < 2.0) && (accelZ > 8.0 && accelZ < 12.0);
    bool bottom = (accelX > -3.0 && accelX < 3.0) && (accelY > -3.0 && accelY < 3.0) && (accelZ < -6.0 && accelZ > -12.0);
    bool left = (accelX > -3.0 && accelX < 3.0) && (accelY > 6.0 && accelY < 12.0) && (accelZ > -3.0 && accelZ < 3.0);
    bool right = (accelX > -3.0 && accelX < 3.0) && (accelY < -6.0 && accelY > -12.0) && (accelZ > -3.0 && accelZ < 3.0);
    bool front = (accelX > 6.0 && accelX < 12.0) && (accelY > -3.0 && accelY < 3.0) && (accelZ > -3.0 && accelZ < 3.0);
    bool rear = (accelX < -6.0 && accelX > -12.0) && (accelY > -23.0 && accelY < 3.0) && (accelZ > -3.0 && accelZ < 3.0);

    if (top) orientation = "TOP";
    if (bottom) orientation = "BOTTOM";
    if (left) orientation = "LEFT";
    if (right) orientation = "RIGHT";
    if (front) orientation = "FRONT";
    if (rear) orientation = "REAR";

    // Serial.println("ORIENTATION: " + orientation);
  }
}

// Setup
void setup() {
  //for debugging
  Serial.begin(9600);

  watchdogInit();
  taskInit();
}

void loop() {}
