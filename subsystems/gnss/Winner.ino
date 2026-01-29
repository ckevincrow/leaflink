////Raygen Yeatman
/// Team Leaf_Link
// GNSS Open Sky 

#include <WiFiS3.h>
#include <R4HttpClient.h>
#include <ArduinoJson.h>

#include <Wire.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>

// ------------ GNSS / logging config ------------
SFE_UBLOX_GNSS gnss;

// Using I2C (Qwiic) to the ZED-F9P
const uint8_t UBLOX_I2C_ADDR = 0x42;
const uint8_t NAV_RATE_HZ    = 1;     // 1 Hz is fine for logging

// How many satellites we are willing to log per epoch
const uint8_t MAX_SATS_LOG = 32;

// Arrays to hold satellite IDs and C/N0 for the last epoch
uint8_t lastNumSV          = 0;
uint8_t lastSvId[MAX_SATS_LOG];
uint8_t lastCn0[MAX_SATS_LOG];
float   lastCn0Avg         = 0.0;

// Basic PVT info
uint32_t lastITOW    = 0;   // ms-of-week
double   lastLat     = 0.0; // deg
double   lastLon     = 0.0; // deg
double   lastHeight  = 0.0; // m
uint32_t lastHAccmm  = 0;   // horizontal accuracy in mm
uint8_t  lastFixType = 0;

bool gnss_ok = false;

// ------------ WiFi / AWS config ------------
const char *ssid     = "";
const char *password = "";

// Your Lambda / API Gateway URL
const char *awsUrl =
  "https://7hcxwqqt37.execute-api.us-east-2.amazonaws.com/"
  "leaflink-default/leaflink-lambda-function";

// 1 = wifi on 
int skip_wifi = 0;  

WiFiSSLClient client;
R4HttpClient  http;

String arduino_ip  = "0.0.0.0";
String device_name = "Open-Sky";  


// Read latest PVT (position / time)
bool readPVT()
{
  if (!gnss.getPVT())
    return false;

  lastITOW    = gnss.getTimeOfWeek();
  lastLat     = gnss.getLatitude()  / 1e7;       // deg
  lastLon     = gnss.getLongitude() / 1e7;       // deg
  lastHeight  = gnss.getAltitudeMSL() / 1000.0;  // mm -> m
  lastHAccmm  = gnss.getHorizontalAccEst();      // mm
  lastFixType = gnss.getFixType();

  return true;
}

// Read NAV-SAT and fill SVID + CN0 arrays + average
bool readNAVSAT()
{
  if (!gnss.getNAVSAT())
    return false;

  if (gnss.packetUBXNAVSAT == nullptr)
    return false;

  UBX_NAV_SAT_data_t *sat = &gnss.packetUBXNAVSAT->data;

  uint8_t n = sat->header.numSvs;   
  if (n > MAX_SATS_LOG) n = MAX_SATS_LOG;

  lastNumSV = n;

  uint16_t sumCn0 = 0;
  for (uint8_t i = 0; i < n; i++)
  {
    lastSvId[i] = sat->blocks[i].svId;
    lastCn0[i]  = sat->blocks[i].cno;
    sumCn0 += lastCn0[i];
  }

  lastCn0Avg = (n > 0) ? (float(sumCn0) / float(n)) : 0.0f;
  return (n > 0);
}

// svId and C/N0
void buildSatelliteStrings(String &svIdStr, String &cn0Str)
{
  svIdStr = "";
  cn0Str  = "";

  for (uint8_t i = 0; i < lastNumSV; i++)
  {
    if (i > 0)
    {
      svIdStr += ",";
      cn0Str  += ",";
    }
    svIdStr += String(lastSvId[i]);
    cn0Str  += String(lastCn0[i]);
  }
}

// Send JSON record to AWS
void postToDb(uint32_t hostMs)
{
  StaticJsonDocument<1024> doc;

  doc["datetime"]   = String(lastITOW); // GNSS time-of-week
  doc["device"]     = device_name;
  doc["arduino_ip"] = arduino_ip;
  doc["hostMs"]     = String(hostMs);

  doc["lat"]        = String(lastLat, 7);
  doc["lon"]        = String(lastLon, 7);
  doc["height_m"]   = String(lastHeight, 3);
  doc["hAcc_mm"]    = String(lastHAccmm);
  doc["fixType"]    = lastFixType;
  doc["numSV"]      = lastNumSV;

  String svIdStr, cn0Str;
  buildSatelliteStrings(svIdStr, cn0Str);

  doc["cn0Avg_dBHz"] = String(lastCn0Avg, 2);
  doc["svIds"]       = svIdStr;
  doc["cn0s"]        = cn0Str;

  String requestBody;
  serializeJson(doc, requestBody);

  Serial.println(requestBody);

  if (skip_wifi == 1) return;

  http.begin(client, awsUrl, 443);
  http.setTimeout(3000);

  int responseNum = http.POST(requestBody);
  if (responseNum > 0)
  {
    String responseBody = http.getBody();
    Serial.print("HTTP response: ");
    Serial.println(responseBody);
  }
  else
  {
    Serial.print("HTTP Post failed: ");
    Serial.println(responseNum);
  }

  http.close();
}

void setupWiFi()
{
  if (skip_wifi) return;

  Serial.print("Connecting to WiFi SSID: ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print(".");
    delay(500);
    if (millis() - t0 > 20000) // 20s timeout
    {
      Serial.println("\nWiFi connect TIMEOUT. Continuing offline.");
      skip_wifi = 1;
      return;
    }
  }

  Serial.println("\nWiFi connected.");
  arduino_ip = WiFi.localIP().toString();
  Serial.print("IP: ");
  Serial.println(arduino_ip);

  Serial.print("WiFi FW: ");
  Serial.println(WiFi.firmwareVersion());
}

void setup()
{
  
  Serial.begin(115200);

  // Don't block forever waiting for Serial
  uint32_t t0 = millis();
  while (!Serial && (millis() - t0 < 3000)) { }

  delay(200); 
  Serial.println("\n=== BOOT: LeafLink logger ===");

  Serial.println("Step 1: Wire.begin()");
  Wire.begin();
  Serial.println("Step 1 OK");

  Serial.println("Step 2: gnss.begin()");
  gnss_ok = gnss.begin(Wire, UBLOX_I2C_ADDR);

  if (!gnss_ok)
  {
    Serial.println("ERROR: ZED-F9P not found on I2C @0x42.");
    Serial.println("Continuing without GNSS (will print heartbeat).");
  }
  else
  {
    Serial.println("GNSS OK");

    Serial.println("Config: UBX only");
    gnss.setI2COutput(COM_TYPE_UBX);

    Serial.print("Config: Navigation frequency = ");
    Serial.print(NAV_RATE_HZ);
    Serial.println(" Hz");
    gnss.setNavigationFrequency(NAV_RATE_HZ);

    Serial.println("Config: Auto PVT + Auto NAVSAT enabled");
    gnss.setAutoPVT(true, true);
    gnss.setAutoNAVSAT(true, true);
  }

  Serial.println("Step 3: setupWiFi()");
  setupWiFi();
  Serial.println("Setup complete.");
}

void loop()
{
  static uint32_t lastHeartbeat = 0;
  uint32_t nowMs = millis();

  if (!gnss_ok)
  {
    // If GNSS wasn't detected, prove the MCU is alive
    if (nowMs - lastHeartbeat >= 1000)
    {
      lastHeartbeat = nowMs;
      Serial.println("Heartbeat: running (GNSS not detected).");
    }
    delay(10);
    return;
  }

  gnss.checkUblox();

  static uint32_t lastSendMs = 0;

  bool havePVT = readPVT();
  bool haveNAV = readNAVSAT();

  if (havePVT && haveNAV)
  {
    if (nowMs - lastSendMs >= 5000)
    {
      lastSendMs = nowMs;
      postToDb(nowMs);
    }
  }
  else
  {
    // Shows occasional status so you know it's working
    if (nowMs - lastHeartbeat >= 1000)
    {
      lastHeartbeat = nowMs;
      Serial.print("Waiting GNSS... PVT=");
      Serial.print(havePVT ? "1" : "0");
      Serial.print(" NAVSAT=");
      Serial.println(haveNAV ? "1" : "0");
    }
  }

  delay(10);
}

