/*
  Get the high precision geodetic solution for latitude and longitude
  By: Nathan Seidle
  Modified by: Steven Rowland and Paul Clark
  SparkFun Electronics
  Date: April 17th, 2020
  License: MIT. See license file for more information but you can
  basically do whatever you want with this code.

  This example shows how to inspect the accuracy of the high-precision
  positional solution. Please see below for information about the units.

  Feel like supporting open source hardware?
  Buy a board from SparkFun!
  ZED-F9P RTK2: https://www.sparkfun.com/products/15136
  NEO-M8P RTK: https://www.sparkfun.com/products/15005

  Hardware Connections:
  Plug a Qwiic cable into the GNSS and a BlackBoard
  If you don't have a platform with a Qwiic connection use the SparkFun Qwiic Breadboard Jumper (https://www.sparkfun.com/products/14425)
  Open the serial monitor at 115200 baud to see the output
*/

// Combined ZED-F9P (I2C) + MAX-M10S (I2C or UART) demo
// Prints ZED-F9P high-precision lat/lon (HPPOSLLH) and MAX-M10S basic PVT
// Board: Arduino uno wifi R4
// Serial Monitor: 9600 baud

/*
  LeafLink GNSS rubric logger – Option B
  ZED-F9P on I2C (0x42) + MAX-M10S on UART2
  Arduino Mega 2560 – Serial Monitor @ 115200

  Metrics reported:
    (1) Timestamp sync: |Δt| <= 10 ms – running % (target >= 70%)
    (2) Same 5 satellites continuously over 5 s – pass/fail per window
    (3) Dual-frequency present on F9P + horiz error <= 2.5 m – running % (target >= 65%)

  Notes:
    - Set your surveyed reference position below (LAT_REF/LON_REF).
    - We compare L1-only C/N0 lists across receivers for satellite matching.
    - NAV-SAT is used for per-satellite lists; NAV-PVT for time & position.

  Library: SparkFun_u-blox_GNSS_Arduino_Library
*/

/*
  LeafLink UBX logger 
  - Logs NAV-SAT, PVT (POSLLH-equivalent), and MON-RF (if available)
  - Default wiring:
      * ZED-F9P on I2C (SDA/SCL).  Address 0x42.
      * (Optional) MAX-M10S on Serial2 @ 9600 flip USE_UART_M10S
  - Output: CSV-like lines on Serial @ 9600 (timestamp millis + UBX fields)


*/
#include <SoftwareSerial.h>
#include <WiFiS3.h>

//#include "R4WiFi_secrets.h" //to place Wifi's SSID & pw in separate file
#include <R4HttpClient.h>
#include <ArduinoJson.h>

#include <Wire.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>


#define USE_I2C_F9P      1      // 1 = F9P via I2C (0x42). 0 = use UART2 (MAX-M10S)
#define NAV_RATE_HZ      5      // message rate (Hz)
#define ELEV_MASK_DEG    0      // set >0 if you want to filter low elevation sats in print
#define MAX_PRINTED_SATS 64


SFE_UBLOX_GNSS gnss;

#if USE_I2C_F9P
// I2C address for u-blox default is 0x42
const uint8_t UBLOX_I2C_ADDR = 0x42;
#else
// (Mega: RX2=19, TX2=18)
const uint32_t UART_BAUD = 9600;
#endif


void printKV(const char *k, long v)   { Serial.print(k); Serial.print('='); Serial.print(v); }
void printKVf(const char *k, double v){ Serial.print(k); Serial.print('='); Serial.print(v, 7); } // for deg * 1e-7



// Try both NAV-SAT type layouts (header variant tolerance)
static inline bool readNAVSAT_and_print(uint32_t hostTimeMs)
{
  if (!gnss.getNAVSAT()) return false;

    // Compose line
    Serial.print(hostTimeMs);
    Serial.print(", NAV-SAT, ");

      char idx[8];
      Serial.print("gnssId_"); Serial.print(idx); Serial.print('=');
      Serial.print("svId_"); Serial.print(idx); Serial.print('=');
      Serial.print("cno_"); Serial.print(idx); Serial.print('=');
      Serial.print("elev_"); Serial.print(idx); Serial.print('=');
      Serial.print("azim_"); Serial.print(idx); Serial.print('=');
    }
      char idx[8];

//  PVT Position Velocity Time
static inline bool readPVT_and_print(uint32_t hostTimeMs)
{
  if (!gnss.getPVT()) 
  return false;

  Serial.print(hostTimeMs);
  Serial.print(", NAV-POSLLH, ");

  // iTOW
  uint32_t iTOW = gnss.getTimeOfWeek(); // milliseconds-of-week if available
  Serial.print("iTOW="); Serial.print((unsigned long)iTOW); Serial.print(", ");

  // Position (deg, mm)
  int32_t lat_e7 = gnss.getLatitude();
  int32_t lon_e7 = gnss.getLongitude();
  int32_t height_mm = gnss.getAltitude(); // above ellipsoid

  Serial.print("lat=");  Serial.print(lat_e7);  Serial.print(", ");
  Serial.print("lon=");  Serial.print(lon_e7);  Serial.print(", ");
  Serial.print("height="); Serial.print(height_mm);
  Serial.println();

  return true;
}

// MON-RF 
static inline bool readMONRF_and_print(uint32_t hostTimeMs)
{
  // Only compiled if library supports UBX-MON-RF 
  #if defined(SFE_UBLOX_GNSS_LIBRARY_VERSION_MAJOR)
    // Check internal buffer; SparkFun lib pushes into packetUBXMONRF when setAutoMONRF(true)
    if (!gnss.getMONRF()) return false;

    // Some header variants provide arrays; print first two AGC counters if present
    // Note: exact struct names can vary; the library generally exposes agcCnt_* fields.
    UBX_MON_RF_t *rf = gnss.packetUBXMONRF;
    if (rf == nullptr) return false;

    Serial.print(hostTimeMs);
    Serial.print(", MON-RF, ");

    
    // If your header names differ, adjust below two lines to match (e.g., OS / UC).
    Serial.print("OS="); Serial.print(rf->blocks[0].agcCnt);
    Serial.print(", UC=");
    if (rf->numBlocks > 1) Serial.print(rf->blocks[1].agcCnt); else Serial.print(-1);
    Serial.println();
    return true;
  #else
    (void)hostTimeMs;
    return false;
  #endif
}

void setup()
{
  Serial.begin(9600);
  while (!Serial) {}

#if USE_I2C_F9P
  Wire.begin();
  if (!gnss.begin(Wire, UBLOX_I2C_ADDR)) {
    Serial.println(F("ERROR: u-blox not found on I2C (0x42). Check wiring."));
    while (1);
  }
  gnss.setI2COutput(COM_TYPE_UBX); // UBX only (no NMEA noise)
#else
  Serial2.begin(UART_BAUD);
  if (!gnss.begin(Serial2)) {
    Serial.println(F("ERROR: u-blox not found on Serial2. Check wiring/baud."));
    while (1);
  }
  gnss.setUART2Output(COM_TYPE_UBX);
#endif

  // Set rates
  gnss.setNavigationFrequency(NAV_RATE_HZ); // 1..20 Hz depending on model

  // Enable automatic population of buffers we will poll
  gnss.setAutoPVT(true, true);     // auto PVT (POSLLH-equivalent)
  gnss.setAutoNAVSAT(true, true);  // auto NAV-SAT blocks

  Serial.println(F("# LeafLink UBX logger ready: printing NAV-SAT, NAV-POSLLH (PVT), MON-RF (if avail)"));
}

void loop()
{
  // Keep the library’s internal parser fed:
  gnss.checkUblox();

  const uint32_t nowMs = millis();

  // Try each message type; print if new
  bool any = false;
  any |= readNAVSAT_and_print(nowMs);
  any |= readPVT_and_print(nowMs);
  any |= readMONRF_and_print(nowMs);

  if (!any) {
    
    delay(5);
  }
}

// GPS module setup
double lat;
double lon;
 
// Sensor setup
double sensorValue;
 
// WiFi settings, if not using R4WiFi_secrets.h
const char *ssid = "eduroam";
//const char *ssid = "Galaxy Z Fold3 5G C407";
const char *password = "Powerhouse17!";
//const char *password = "CaeraD23";
int skip_wifi = 1; //TBD: for now, change to 1 if wifi is not available
 
//HTTP Post settings
WiFiSSLClient client;
R4HttpClient http;
String arduino_ip;
 
//Identifier for the Arduino executing this code
String device = "ckc-arduino2LL"; //YOUR_INITIALS-arduino#, e.g. for Kevin: ckc-arduino2
 
//////////////////////////////////////////////////////////////////////////
void setup_kevin() 
{
 
  Serial.begin(9600);
  randomSeed(analogRead(A0)); //unused pin, for random number generation
 
  // Connect GPS
 
  //Wifi Connection:
  if (!skip_wifi) {          //TBD
    WiFi.begin(ssid, password);
    Serial.print("Connecting to WiFi ");
    Serial.print(ssid);
    Serial.print("... ");
    while ((WiFi.status() != WL_CONNECTED)) {
      delay(5000);
      Serial.print(".");
    }
    Serial.print("connected (");
    arduino_ip = WiFi.localIP().toString();
    Serial.print(arduino_ip); //TBD: Network
    Serial.println(").");
 
    //HTTP Post request prefers updated WiFi firmware:
    String fv = WiFi.firmwareVersion();
    if (fv < WIFI_FIRMWARE_LATEST_VERSION) {
      Serial.println(F("Please upgrade the Wifi firmware"));
    }
    if (WiFi.status() == WL_NO_MODULE) { //TBD: Unknown connection problem
      Serial.println(F("Communication with WiFi module failed!"));
      while (true);
    }
  }
}
//////////////////////////////////////////////////////////////////////////////
 
void loop2() 
{
  if (skip_wifi==1) { //slow things down so as not to incur charges from AWS
    delay(2000);
  }
  readGPS();
  readSensor();
  //add other sensor reading calls here
 
  String datetime = String(random(0, 100));
  postToDb(datetime, sensorValue, lat, lon, device, arduino_ip);
 
  delay(50); //TBD
}
 
////////////////////////////////////////////////////////////////////////////////
 
 
//Read GPS
void readGPS() 
{
  lat = random(0, 100);
  lon = random(0, 100);
  delay(50);
}
 
//Read Sensor
void readSensor() 
{
  sensorValue = random(0, 100);
  delay(200);
}
 
//Create additional read functions as necessary - in general this will require reading pins.
 
 
void postToDb(String datetime, uint8_t sensorValue, double lat, double lon, String arduino_id, String arduino_ip) 
{
  //Each sensor variable needs a column in the database (e.g. "level") - do not hesitate to turn a number into a string for testing purposes:
  StaticJsonDocument<512> doc;
  doc["datetime"] = datetime;
  doc["level"] = sensorValue;
  doc["device"] = String(device);
  doc["arduino_ip"] = String(arduino_ip);
  doc["lon"] = String(lon, 9);
  doc["lat"] = String(lat, 9);
 
  String requestBody;
  serializeJson(doc, requestBody);
 
  //Print to serial monitor
  //Serial.print("requestBody: "); //TBD: signal for local server to process serial monitor content, when not on wifi
  Serial.println(requestBody); // for serial monitor reading on local laptop
 
  if (skip_wifi != 1) {
    //Post to AWS DynamoDB via AWS Lambda
    http.begin(client, "https://7hcxwqqt37.execute-api.us-east-2.amazonaws.com/leaflink-default/leaflink-lambda-function", 443);
    //http.setDebug(true);
    http.setTimeout(3000); //TBD: 3000 default
    //http.addHeader("User-Agent: Arduino UNO R4 WiFi"); //TBD: nec?
    //http.addHeader("Content-Type: application/json"); //TBD: nec?
 
    int responseNum = http.POST(requestBody);
    if (responseNum > 0) // OR if (responseNum == HTTP_CODE_OK) // 200 is OK, i.e. data posted to DynamoDB
    {
      String responseBody = http.getBody();
      Serial.println(responseBody); //TBD
      //Serial.println("Response code: " + String(responseNum));
    } else {
      Serial.println("HTTP Post Request Failed: " + String(responseNum));
    }
 
    http.close();
  }
}



