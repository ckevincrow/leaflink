// =============================================================
//  LeafLink + Environmental/Power Monitoring
//  Device ID: wag-R4
//  Board: Arduino UNO R4 WiFi
// =============================================================

#include "Adafruit_SHT31.h"
#include <Adafruit_INA260.h>
#include <SoftwareSerial.h>
#include <WiFiS3.h>
#include <R4HttpClient.h>
#include <ArduinoJson.h>

// ---------- SENSOR OBJECTS ----------
Adafruit_SHT31 sht31 = Adafruit_SHT31();
Adafruit_INA260 ina260 = Adafruit_INA260();

// ---------- ALERT PIN ----------
const int alertPin = 7;

// ---------- SAFETY LIMITS ----------
const float MAX_TEMP = 45.0;   // °C
const float MIN_TEMP = 0.0;    // °C
const float MAX_HUM  = 70.0;   // %
const float MIN_VOLT = 2.0;    // V

// ---------- WiFi SETTINGS ----------
const char* ssid     = "Gentry";
const char* password = "123qweasd";

int skip_wifi = 0;   // set to 1 for offline testing

WiFiSSLClient client;
R4HttpClient http;
String arduino_ip;

// ---------- DEVICE IDENTIFIER ----------
String device = "wag-R4";

// ---------- VARIABLES ----------
float tempC, humRH;
float voltage_V, current_mA, power_mW;

bool unsafeTemp;
bool unsafeCold;
bool unsafeHum;
bool lowVolt;

// =============================================================
// SETUP
// =============================================================
void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  pinMode(alertPin, OUTPUT);
  digitalWrite(alertPin, LOW);

  Serial.println("===== LeafLink Environmental + Power Monitor =====");

  // ---- INIT SHT31 ----
  if (!sht31.begin(0x44)) {
    Serial.println("ERROR: SHT31 not detected!");
    while (1);
  }
  sht31.heater(false);
  Serial.println("SHT31 Ready.");

  // ---- INIT INA260 ----
  if (!ina260.begin()) {
    Serial.println("ERROR: INA260 not detected!");
    while (1);
  }
  Serial.println("INA260 Ready.");

  // ---- WIFI CONNECTION ----
  if (!skip_wifi) {
    Serial.print("Connecting to WiFi ");
    Serial.print(ssid);
    Serial.print("... ");

    WiFi.begin(ssid, password);

    while (WiFi.status() != WL_CONNECTED) {
      delay(2000);
      Serial.print(".");
    }

    arduino_ip = WiFi.localIP().toString();
    Serial.print(" connected: ");
    Serial.println(arduino_ip);
  }
}

// =============================================================
// LOOP
// =============================================================
void loop() {
  readSensors();
  checkConditions();
  displayReadings();
  postReadingsToAWS();
  delay(2000);
}

// =============================================================
// READ SENSORS
// =============================================================
void readSensors() {
  tempC = sht31.readTemperature();
  humRH = sht31.readHumidity();

  current_mA = ina260.readCurrent();
  voltage_V  = ina260.readBusVoltage() / 1000.0;
  power_mW   = ina260.readPower();
}

// =============================================================
// CHECK CONDITIONS
// =============================================================
void checkConditions() {
  unsafeTemp  = tempC > MAX_TEMP;
  unsafeCold = tempC < MIN_TEMP;
  unsafeHum   = humRH > MAX_HUM;
  lowVolt     = voltage_V < MIN_VOLT;

  if (unsafeTemp || unsafeCold || unsafeHum || lowVolt) {
    digitalWrite(alertPin, HIGH);
  } else {
    digitalWrite(alertPin, LOW);
  }
}

// =============================================================
// DISPLAY VALUES
// =============================================================
void displayReadings() {
  Serial.println("\n===== SENSOR READINGS =====");

  if (!isnan(tempC) && !isnan(humRH)) {
    Serial.print("Temperature: "); Serial.print(tempC); Serial.println(" °C");
    Serial.print("Humidity:    "); Serial.print(humRH); Serial.println(" %");
  } else {
    Serial.println("ERROR: Failed to read SHT31.");
  }

  Serial.print("Voltage: "); Serial.print(voltage_V); Serial.println(" V");
  Serial.print("Current: "); Serial.print(current_mA); Serial.println(" mA");
  Serial.print("Power:   "); Serial.print(power_mW); Serial.println(" mW");

  Serial.println("===== SYSTEM STATUS =====");
  Serial.print("Overheating? "); Serial.println(unsafeTemp ? "YES" : "NO");
  Serial.print("Too cold? ");     Serial.println(unsafeCold ? "YES" : "NO");
  Serial.print("Humidity too high? "); Serial.println(unsafeHum ? "YES" : "NO");
  Serial.print("Low voltage? "); Serial.println(lowVolt ? "YES" : "NO");

  Serial.print("Overall System: ");
  Serial.println((unsafeTemp || unsafeCold || unsafeHum || lowVolt) ? "UNSAFE" : "SAFE");
}

// =============================================================
// POST TO AWS DYNAMODB
// =============================================================
void postReadingsToAWS() {
  if (skip_wifi) return;

  StaticJsonDocument<512> doc;

  // Device info
  doc["device"]     = device;
  doc["arduino_ip"] = arduino_ip;

  // Unique timestamp
  doc["datetime"] = String(millis());

  // Sensor data
  doc["tempC"]      = String(tempC, 2);
  doc["humidity"]   = String(humRH, 2);
  doc["voltage_V"]  = String(voltage_V, 4);
  doc["current_mA"] = String(current_mA, 2);
  doc["power_mW"]   = String(power_mW, 2);

  // Safety flags
  doc["unsafeTemp"]  = unsafeTemp  ? "YES" : "NO";
  doc["unsafeCold"] = unsafeCold ? "YES" : "NO";
  doc["unsafeHum"]   = unsafeHum   ? "YES" : "NO";
  doc["lowVolt"]     = lowVolt     ? "YES" : "NO";

  // Overall system status
  doc["systemStatus"] =
    (unsafeTemp || unsafeCold || unsafeHum || lowVolt) ? "UNSAFE" : "SAFE";

  // Optional LeafLink metric
  doc["level"] = String(tempC, 2);

  String body;
  serializeJson(doc, body);

  Serial.println("\nPOST BODY:");
  Serial.println(body);

  http.begin(client, "https://7hcxwqqt37.execute-api.us-east-2.amazonaws.com/leaflink-default/leaflink-lambda-function", 443);
  http.setTimeout(3000);

  int response = http.POST(body);
  if (response > 0) {
    Serial.println("AWS RESPONSE:");
    Serial.println(http.getBody());
  } else {
    Serial.println("HTTP FAILED: " + String(response));
  }

  http.close();
}


