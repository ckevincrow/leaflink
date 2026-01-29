/*
  LeafLink – Single C12880MA – PAR + Lux + WiFi + DynamoDB
  Board: Arduino UNO R4 WiFi

  Features:
  - Single Hamamatsu C12880MA spectrometer on A2
  - DS3231 RTC for UTC timestamps
  - Per-pixel dark subtraction (dark frame)
  - Computes:
      * PAR (400–700 nm) in µmol·m⁻²·s⁻¹
      * Lux using CIE V(λ) weighting (true photopic lux)
  - WiFi + HTTPS POST to AWS API Gateway / Lambda (LeafLink DynamoDB)
  - All sensor outputs stored in variables before DynamoDB send

  Usage:
  - On startup: auto dark calibration (cover sensor!)
  - At any time: type 'd' in Serial Monitor and press Enter to redo dark cal
*/

#include <Arduino.h>
#include <Wire.h>
#include <RTClib.h>
#include <WiFiS3.h>
#include <R4HttpClient.h>
#include <ArduinoJson.h>
#include <math.h>

// ---------------- RTC ----------------
RTC_DS3231 rtc;

// ---------------- PINS ----------------
const uint8_t PIN_ST      = 7;
const uint8_t PIN_CLK     = 8;
const uint8_t PIN_VIDEO   = A2;

// ---------------- SPECTROMETER ----------------
const int   N_PIXELS    = 288;
const int   INTEG_CLKS  = 8;
const int   DUMMY_CLKS  = 16;
const uint8_t T_HIGH_US = 1;
const uint8_t T_LOW_US  = 1;

// ---------------- WAVELENGTH MAPPING ----------------
const float L_MIN    = 340.0f;
const float L_MAX    = 850.0f;
const float DELTA_NM = (L_MAX - L_MIN) / (N_PIXELS - 1);

// ---------------- STATS STRUCT ----------------
struct Stats {
  uint16_t minv;
  uint16_t maxv;
  float    mean;
  int      peakPix;
  float    peakNm;
};

// ---------------- CALIBRATION CONSTANTS ----------------
float K_E = 1.0e-6f;

#define RAW_LUX_MID_THRESH   100000.0f
#define RAW_LUX_HIGH_THRESH  200000.0f

const float LUX_GAIN_LOW  = 0.001720f;
const float LUX_GAIN_MID  = 0.001800f;
const float LUX_GAIN_HIGH = 0.009600f;

const float RAW_LUX_OFFSET = 97.1f;

// ---------------- BUFFERS ----------------
uint16_t spec[N_PIXELS];
uint16_t darkSpec[N_PIXELS] = {0};
uint16_t currentSpec[N_PIXELS];

bool darkCalDone = false;

// ---------------- OUTPUT VARIABLES ----------------
// Sensor variables (all integers - DynamoDB doesn't support floats)
int32_t sensor_counts_min;
int32_t sensor_counts_max;
int32_t sensor_counts_mean;
int32_t sensor_peak_pixel;
int32_t sensor_peak_nm;
int32_t sensor_PAR_milli;   // PAR * 1000 (milli-units)
int32_t sensor_Lux_milli;   // Lux * 1000 (milli-units)
int32_t sensor_adc_400nm;
int32_t sensor_adc_425nm;
int32_t sensor_adc_450nm;
int32_t sensor_adc_475nm;
int32_t sensor_adc_500nm;
int32_t sensor_adc_525nm;
int32_t sensor_adc_550nm;
int32_t sensor_adc_575nm;
int32_t sensor_adc_600nm;
int32_t sensor_adc_625nm;
int32_t sensor_adc_650nm;
int32_t sensor_adc_675nm;
int32_t sensor_adc_700nm;

// Meta variables
String timestamp_utc;

// ---------------- WIFI / DB CONFIG ----------------
const char *ssid     = "TP-Link_1130";
const char *password = "abcd1234";
int skip_wifi = 0;

WiFiSSLClient client;
R4HttpClient  http;
String        arduino_ip = "";
String        device     = "dac/R4";

// ---------------- HELPERS ----------------
inline void clkHigh() { digitalWrite(PIN_CLK, HIGH); delayMicroseconds(T_HIGH_US); }
inline void clkLow()  { digitalWrite(PIN_CLK, LOW);  delayMicroseconds(T_LOW_US);  }

float pixelToNm(int p) {
  return L_MIN + DELTA_NM * (float)p;
}

float Vlambda(float nm) {
  if (nm < 380.0f || nm > 780.0f) return 0.0f;
  float x = nm - 555.0f;
  float sigma = (nm < 555.0f) ? 35.0f : 45.0f;
  return expf(-0.5f * x * x / (sigma * sigma));
}

// ---------------- SPECTRUM ACQUISITION ----------------
void getSpectrum(uint16_t *data) {
  digitalWrite(PIN_ST, LOW);
  digitalWrite(PIN_CLK, LOW);
  delayMicroseconds(2);

  digitalWrite(PIN_ST, HIGH);
  for (int i = 0; i < INTEG_CLKS; i++) {
    clkHigh();
    clkLow();
  }
  digitalWrite(PIN_ST, LOW);

  for (int i = 0; i < DUMMY_CLKS; i++) {
    clkHigh();
    clkLow();
  }

  for (int i = 0; i < N_PIXELS; i++) {
    clkHigh();
    data[i] = analogRead(PIN_VIDEO);
    clkLow();
  }
}

// ---------------- PAR & LUX COMPUTATION ----------------
float luxFromRaw(float raw) {
  raw -= RAW_LUX_OFFSET;
  if (raw < 0) raw = 0;

  if (raw < RAW_LUX_MID_THRESH) {
    return raw * LUX_GAIN_LOW;
  } 
  else if (raw < RAW_LUX_HIGH_THRESH) {
    return raw * LUX_GAIN_MID;
  } 
  else {
    return raw * LUX_GAIN_HIGH;
  }
}

float luxIntegralRaw(const uint16_t *s) {
  float acc = 0.0f;
  for (int i = 0; i < N_PIXELS; i++) {
    float nm = pixelToNm(i);
    float v  = Vlambda(nm);
    if (v <= 0.0f) continue;
    acc += s[i] * v * DELTA_NM;
  }
  return acc;
}

float computePAR(const uint16_t *s, float K_E_val) {
  const float h  = 6.626e-34f;
  const float c  = 2.998e8f;
  const float NA = 6.022e23f;

  double photons = 0.0;

  for (int i = 0; i < N_PIXELS; i++) {
    float nm = pixelToNm(i);
    if (nm < 400.0f || nm > 700.0f) continue;

    float E  = s[i] * K_E_val * DELTA_NM;
    float Ep = (h * c) / (nm * 1e-9f);
    photons += E / Ep;
  }

  return (photons / NA) * 1e6f;
}

void measureAveraged(int samples,
                     float &lux,
                     float &par,
                     float &rawLuxInt) {
  uint32_t accum[N_PIXELS] = {0};

  for (int s = 0; s < samples; s++) {
    getSpectrum(spec);

    for (int i = 0; i < N_PIXELS; i++) {
      int32_t v = spec[i] - (darkCalDone ? darkSpec[i] : 0);
      if (v < 0) v = 0;
      accum[i] += (uint32_t)v;
    }
    delay(5);
  }

  for (int i = 0; i < N_PIXELS; i++) {
    currentSpec[i] = accum[i] / samples;
  }

  rawLuxInt = luxIntegralRaw(currentSpec);
  lux = luxFromRaw(rawLuxInt);
  par = computePAR(currentSpec, K_E);
}

// ---------------- DARK CALIBRATION ----------------
void doDarkCalibration() {
  Serial.println(F("\n=== DARK CALIBRATION – COVER SENSOR NOW ==="));
  delay(3000);

  const int N = 50;
  uint32_t accum[N_PIXELS] = {0};

  for (int n = 0; n < N; n++) {
    getSpectrum(spec);
    for (int i = 0; i < N_PIXELS; i++) {
      accum[i] += spec[i];
    }
    delay(10);
  }

  float mean = 0.0f;

  for (int i = 0; i < N_PIXELS; i++) {
    darkSpec[i] = accum[i] / N;
    mean += darkSpec[i];
  }
  mean /= N_PIXELS;

  darkCalDone = true;

  Serial.print(F("Dark mean = ")); Serial.println(mean, 1);
  Serial.println(F("=== DARK CAL COMPLETE ===\n"));
}

// ---------------- STATS / DEBUG ----------------
Stats computeStats(const uint16_t *s) {
  uint32_t sum = 0;
  uint16_t mn = 65535, mx = 0;
  int pk = 0;

  for (int i = 0; i < N_PIXELS; i++) {
    uint16_t v = s[i];
    sum += v;
    if (v < mn) mn = v;
    if (v > mx) { mx = v; pk = i; }
  }

  Stats st;
  st.minv    = mn;
  st.maxv    = mx;
  st.mean    = sum / (float)N_PIXELS;
  st.peakPix = pk;
  st.peakNm  = pixelToNm(pk);
  return st;
}

uint16_t countsAtNm(const uint16_t *s, float targetNm) {
  int   bestIdx  = 0;
  float bestDiff = 1e9f;
  for (int i = 0; i < N_PIXELS; i++) {
    float nm = pixelToNm(i);
    float d  = fabsf(nm - targetNm);
    if (d < bestDiff) {
      bestDiff = d;
      bestIdx  = i;
    }
  }
  return s[bestIdx];
}

void printSensor(const char *label,
                 const uint16_t *spectrum,
                 float lux, float par,
                 float rawLuxInt) {
  Stats st = computeStats(spectrum);

  Serial.println(label);
  
  if (st.maxv >= 4095) {
     Serial.println(F("!!! [SATURATION DETECTED] !!! Data clipped at 4095."));
  }

  Serial.print(F("  Counts min=")); Serial.print(st.minv);
  Serial.print(F(" max="));         Serial.print(st.maxv);
  Serial.print(F(" mean="));        Serial.println(st.mean, 1);
  Serial.print(F("  Peak pixel=")); Serial.print(st.peakPix);
  Serial.print(F(" (~"));           Serial.print(st.peakNm, 0);
  Serial.println(F(" nm)"));

  Serial.println(F("  ADC counts (400–700 nm):"));
  for (float nm = 400.0f; nm <= 700.0f; nm += 25.0f) {
    Serial.print(F("    "));
    Serial.print((int)nm);
    Serial.print(F(" nm : "));
    Serial.println(countsAtNm(spectrum, nm));
  }

  Serial.print(F("  PAR = ")); Serial.print(par, 3); Serial.println(F(" µmol·m⁻²·s⁻¹"));
  Serial.print(F("  RAW LuxIntegral = ")); Serial.println(rawLuxInt, 3);
  Serial.print(F("  Lux = ")); Serial.print(lux, 2); Serial.println(F(" lux\n"));
}

// ---------------- TIME / META ----------------
String getUTCTime() {
  DateTime now = rtc.now();
  char buf[32];
  sprintf(buf, "%04d-%02d-%02d %02d:%02d:%02d UTC",
          now.year(), now.month(), now.day(),
          now.hour(), now.minute(), now.second());
  return String(buf);
}

// ---------------- POPULATE OUTPUT VARIABLES ----------------
void populateOutputVariables(float lux, float par) {
  Stats st = computeStats(currentSpec);

  sensor_counts_min  = (int32_t)st.minv;
  sensor_counts_max  = (int32_t)st.maxv;
  sensor_counts_mean = (int32_t)lround(st.mean);
  sensor_peak_pixel  = (int32_t)st.peakPix;
  sensor_peak_nm     = (int32_t)lround(st.peakNm);
  sensor_PAR_milli   = (int32_t)lround(par * 1000.0f);  // Convert to milli-units
  sensor_Lux_milli   = (int32_t)lround(lux * 1000.0f);  // Convert to milli-units
  
  sensor_adc_400nm = (int32_t)lround(countsAtNm(currentSpec, 400.0f));
  sensor_adc_425nm = (int32_t)lround(countsAtNm(currentSpec, 425.0f));
  sensor_adc_450nm = (int32_t)lround(countsAtNm(currentSpec, 450.0f));
  sensor_adc_475nm = (int32_t)lround(countsAtNm(currentSpec, 475.0f));
  sensor_adc_500nm = (int32_t)lround(countsAtNm(currentSpec, 500.0f));
  sensor_adc_525nm = (int32_t)lround(countsAtNm(currentSpec, 525.0f));
  sensor_adc_550nm = (int32_t)lround(countsAtNm(currentSpec, 550.0f));
  sensor_adc_575nm = (int32_t)lround(countsAtNm(currentSpec, 575.0f));
  sensor_adc_600nm = (int32_t)lround(countsAtNm(currentSpec, 600.0f));
  sensor_adc_625nm = (int32_t)lround(countsAtNm(currentSpec, 625.0f));
  sensor_adc_650nm = (int32_t)lround(countsAtNm(currentSpec, 650.0f));
  sensor_adc_675nm = (int32_t)lround(countsAtNm(currentSpec, 675.0f));
  sensor_adc_700nm = (int32_t)lround(countsAtNm(currentSpec, 700.0f));
}

// ---------------- POST TO DYNAMODB ----------------
void postToDb() {
  StaticJsonDocument<2048> doc;

  // Use prefixed field names to control alphabetical sort order in DynamoDB
  // Format: "01_fieldname", "02_fieldname", etc.
  
  // 01-02: Timestamps and metadata
  doc["device"]   = device;
  doc["datetime"] = timestamp_utc;  // Primary key (required)
  doc["01_timestamp"] = timestamp_utc;
  doc["02_arduino_ip"] = arduino_ip;
  
  // 03-07: Statistics
  doc["03_counts_min"] = sensor_counts_min;
  doc["04_counts_max"] = sensor_counts_max;
  doc["05_counts_mean"] = sensor_counts_mean;
  doc["06_peak_pixel"] = sensor_peak_pixel;
  doc["07_peak_nm"] = sensor_peak_nm;
  
  // 08-20: ADC counts at specific wavelengths (400-700nm)
  doc["08_adc_400nm"] = sensor_adc_400nm;
  doc["09_adc_425nm"] = sensor_adc_425nm;
  doc["10_adc_450nm"] = sensor_adc_450nm;
  doc["11_adc_475nm"] = sensor_adc_475nm;
  doc["12_adc_500nm"] = sensor_adc_500nm;
  doc["13_adc_525nm"] = sensor_adc_525nm;
  doc["14_adc_550nm"] = sensor_adc_550nm;
  doc["15_adc_575nm"] = sensor_adc_575nm;
  doc["16_adc_600nm"] = sensor_adc_600nm;
  doc["17_adc_625nm"] = sensor_adc_625nm;
  doc["18_adc_650nm"] = sensor_adc_650nm;
  doc["19_adc_675nm"] = sensor_adc_675nm;
  doc["20_adc_700nm"] = sensor_adc_700nm;
  
  // 21-22: Final measurements
  doc["21_PAR_umol_m2_s"] = sensor_PAR_milli;
  doc["22_lux"] = sensor_Lux_milli;

  String payload;
  serializeJson(doc, payload);

  Serial.println(F("JSON to send:"));
  Serial.println(payload);
  Serial.print(F("Payload size: "));
  Serial.print(payload.length());
  Serial.println(F(" bytes"));

  if (skip_wifi || WiFi.status() != WL_CONNECTED) return;

  http.begin(
    client,
    "https://7hcxwqqt37.execute-api.us-east-2.amazonaws.com/leaflink-default/leaflink-lambda-function",
    443
  );
  http.setTimeout(8000);
  http.addHeader("Content-Type: application/json");
  
  int code = http.POST(payload);

  Serial.print(F("HTTP POST: "));
  Serial.println(code);
  if (code > 0) {
    Serial.println(http.getBody());
  }
  http.close();
}

// ---------------- SETUP ----------------
void setup() {
  Serial.begin(9600);
  while (!Serial) {;}

  pinMode(PIN_ST,  OUTPUT);
  pinMode(PIN_CLK, OUTPUT);
  digitalWrite(PIN_ST, LOW);
  digitalWrite(PIN_CLK, LOW);

  Wire.begin();

  if (!rtc.begin()) {
    Serial.println(F("RTC not found!"));
  } else {
    Serial.println(F("RTC found."));

    // Uncomment to set RTC time (then re-comment and re-upload)
    /*
    DateTime local(F(__DATE__), F(__TIME__));
    const int UTC_OFFSET = 6;
    uint32_t utcUnix = local.unixtime() + UTC_OFFSET * 3600UL;
    rtc.adjust(DateTime(utcUnix));
    Serial.println(F("RTC SET TO TRUE UTC"));
    */
  }

  analogReadResolution(12);

  Serial.println(F("\nSingle C12880MA – PAR + Lux + WiFi"));
  Serial.println(F("Type 'd' in Serial Monitor for dark calibration.\n"));

  if (!skip_wifi) {
    WiFi.begin(ssid, password);
    Serial.print(F("Connecting to WiFi "));
    Serial.print(ssid);
    while (WiFi.status() != WL_CONNECTED) {
      delay(1000);
      Serial.print(F("."));
    }
    arduino_ip = WiFi.localIP().toString();
    Serial.print(F("\nWiFi connected, IP = "));
    Serial.println(arduino_ip);
  }

  delay(2000);
  doDarkCalibration();
}

// ---------------- LOOP ----------------
void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'd' || c == 'D') {
      doDarkCalibration();
    }
  }

  float lux, par;
  float rawInt;

  // Measure and compute sensor data
  measureAveraged(40, lux, par, rawInt);

  // Get timestamp
  timestamp_utc = getUTCTime();

  // Print to Serial
  Serial.println(F("============================================================"));
  Serial.print(F("Time: ")); Serial.println(timestamp_utc);
  printSensor("Sensor", currentSpec, lux, par, rawInt);

  Serial.print(F("[CAL DEBUG] RAW LuxInt="));
  Serial.println(rawInt, 3);

  // Populate all output variables
  populateOutputVariables(lux, par);

  // Send to DynamoDB
  postToDb();

  delay(60000);
}