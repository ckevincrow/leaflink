#include <Arduino.h>
#include <Wire.h>
#include "Adafruit_SHT31.h"
#include <Adafruit_BME280.h>
#include <SoftwareSerial.h>
#include <WiFiS3.h>
#include <R4HttpClient.h>
#include <ArduinoJson.h>

// ---------- SENSOR OBJECTS ----------
Adafruit_SHT31 sht31 = Adafruit_SHT31();
Adafruit_BME280 bme; 

// ---------- WiFi SETTINGS ----------
const char* ssid     = "Strickland2.4";    
const char* password = "Strickland@1955"; 

int skip_wifi = 0;   // set to 1 if offline testing

WiFiSSLClient client;
R4HttpClient http;
String arduino_ip;

// ---------- DEVICE IDENTIFIER ----------
String device = "css-R4";

// ---------- VARIABLES ----------
float tempC, humRH, windSpeed_mps, windSpeed_kmh, windSpeed_mph, pressure;

const int anemometerPin = 0;  // analog pin 0 (A0)
const float minVoltage = 0.054;  // Voltage corresponding to 0 m/s
const float maxVoltage = 5;  // Voltage corresponding to 32.4 m/s (max speed)
const float maxWindSpeed = 32.4; // Maximum wind speed in m/s

const float mps_to_kmh = 3.6;   // 1 m/s = 3.6 km/h
const float mps_to_mph = 2.23694; // 1 m/s = 2.23694 mph

// =============================================================
// SETUP
// =============================================================
void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  Serial.println("===== LeafLink Environmental + Power Monitor =====");

  // ---- INIT SHT31 ----
  if (!sht31.begin(0x44)) {
    Serial.println("ERROR: SHT31 not detected!");
    while (1);
  }
  sht31.heater(false);
  Serial.println("SHT31 Ready.");

  // ---- INIT BME280 ----
  if (!bme.begin(0x76)) {
    Serial.println("Could not find BME280 sensor!");
    while (1);
  }

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

  int adcValue = analogRead(anemometerPin);
  
  // Convert ADC value to voltage (Arduino ADC range is 0-5.0V)
  float voltage = (adcValue / 1023.00) * 5.0;
  
  // Ensure the voltage is within the anemometer operating range
  if (voltage < minVoltage) {
    voltage = minVoltage;
  } 
  else if (voltage > maxVoltage) {
    voltage = maxVoltage;
  }
  
  // Map the voltage to wind speed
  float windSpeed_mps = ((voltage - minVoltage) / (maxVoltage - minVoltage)) * maxWindSpeed;

  // Convert wind speed to km/h and mph
  float windSpeed_kmh = windSpeed_mps * mps_to_kmh;
  float windSpeed_mph = windSpeed_mps * mps_to_mph;

  float pressure = bme.readPressure() / 100.0F; // Pa to hPa
}

// =============================================================
// DISPLAY VALUES
// =============================================================
void displayReadings() {
  Serial.println("\n===== SENSOR READINGS =====");

  if (!isnan(tempC) && !isnan(humRH)) {
    Serial.print("Temperature: "); Serial.print(tempC); Serial.println(" °C");
    Serial.print("Humidity:    "); Serial.print(humRH); Serial.println(" %");
  } 
  else {
    Serial.println("ERROR: Failed to read SHT31.");
  }

  Serial.print("Wind Speed: ");
  Serial.print(windSpeed_mps); Serial.print(" m/s, ");
  Serial.print(windSpeed_kmh); Serial.print(" km/h, ");
  Serial.print(windSpeed_mph); Serial.println(" mph");
  delay(2000);

  Serial.print("Pressure: "); Serial.print(pressure); Serial.print("hPa | ");
  delay(2000);
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

  // Unique datetime key
  String datetime = String(millis());
  doc["datetime"] = datetime;

  // Sensor data
  doc["tempC"]      = String(tempC, 2);
  doc["humidity"]   = String(humRH, 2);
  doc["Wind"]       = String(windSpeed_mph, 2);
  doc["Pressure"]   = String(pressure, 2);

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
  } 
  else {
    Serial.println("HTTP FAILED: " + String(response));
  }

  http.close();
}