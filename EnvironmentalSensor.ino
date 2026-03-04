#include <Arduino.h>
#include <Wire.h>
#include "Adafruit_SHT31.h"
#include "Adafruit_BMP3XX.h"
#include <SoftwareSerial.h>
#include <WiFiS3.h>
#include <R4HttpClient.h>
#include <ArduinoJson.h>

// ---------- SENSOR OBJECTS ----------
Adafruit_SHT31 sht31 = Adafruit_SHT31();
Adafruit_BMP3XX bmp;

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
float tempC, humRH, pressure;
const int anemometerPin = A0;

const float minVoltage = 0.054;     // Voltage at 0 m/s
const float maxVoltage = 5.0;       // Voltage at 32.4 m/s
const float maxWindSpeed = 32.4;    // m/s

float windSpeed = 0.0;

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

  // ---- INIT BME388 ----
  if (!bmp.begin_I2C()) {   
    Serial.println("Could not find a valid BMP388 sensor, check wiring!");
    while (1);
  }

  analogReadResolution(12);  // UNO R4 = 12-bit ADC (0-4095)

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
  pressure = bmp.readPressure();

  int adcValue = analogRead(anemometerPin);

  //bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  //bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);

  // Convert ADC value (0–4095) to voltage
  float voltage = (adcValue / 4095.0) * 5.0;

  // Clamp voltage to operating range
  if (voltage < minVoltage) voltage = minVoltage;
  if (voltage > maxVoltage) voltage = maxVoltage;

  // Convert voltage to wind speed (m/s)
  windSpeed = ((voltage - minVoltage) / (maxVoltage - minVoltage)) * maxWindSpeed;
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

  Serial.print("Pressure = ");
  pressure = bmp.pressure / 100.0;
  Serial.print(pressure);
  Serial.println(" hPa");

  Serial.print("Wind Speed:  ");
  Serial.print(windSpeed);
  Serial.println(" m/s");
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
  doc["pressure"]   = String(pressure, 2);
  doc["windSpeed"]  = String(windSpeed, 2);

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
