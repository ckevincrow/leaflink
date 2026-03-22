#include <Wire.h>
#include <Arduino_RouterBridge.h>
#include <SparkFun_u-blox_GNSS_v3.h>

bool has_GNSS = false;
bool has_C12880MA = false;

SFE_UBLOX_GNSS myGNSS;
int32_t cachedLat = 0, cachedLon = 0, cachedAlt = 0;
uint32_t cachedHAcc = 0, cachedVAcc = 0;
uint8_t cachedNumSVs = 0;
uint16_t cachedAgc0 = 0, cachedAgc1 = 0;
uint8_t cachedJam0 = 0, cachedJam1 = 0;
String cachedSvs = "", cachedCno = "", cachedGns = "";

const uint8_t PIN_ST = 7;
const uint8_t PIN_CLK = 8;
const uint8_t PIN_VIDEO = A2;
const int N_PIXELS = 288;
const int INTEG_CLKS = 20;
const int DUMMY_CLKS = 16;
const uint8_t T_HIGH_US = 1;
const uint8_t T_LOW_US = 1;

uint16_t spec[N_PIXELS];
uint16_t darkSpec[N_PIXELS] = {0};
uint16_t currentSpec[N_PIXELS];
bool darkCalDone = false;
String cachedSpecStr = "[]";

inline void clkHigh() { digitalWrite(PIN_CLK, HIGH); delayMicroseconds(T_HIGH_US); }
inline void clkLow() { digitalWrite(PIN_CLK, LOW); delayMicroseconds(T_LOW_US); }

void getSpectrum(uint16_t *data) {
  digitalWrite(PIN_ST, LOW);
  digitalWrite(PIN_CLK, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_ST, HIGH);
  for (int i = 0; i < INTEG_CLKS; i++) { clkHigh(); clkLow(); }
  digitalWrite(PIN_ST, LOW);
  for (int i = 0; i < DUMMY_CLKS; i++) { clkHigh(); clkLow(); }
  for (int i = 0; i < N_PIXELS; i++) {
    clkHigh();
    data[i] = analogRead(PIN_VIDEO);
    clkLow();
  }
}

void doDarkCalibration() {
  const int N = 50;
  uint32_t accum[N_PIXELS] = {0};
  for (int n = 0; n < N; n++) {
    getSpectrum(spec);
    for (int i = 0; i < N_PIXELS; i++) accum[i] += spec[i];
    delay(10);
  }
  for (int i = 0; i < N_PIXELS; i++) darkSpec[i] = accum[i] / N;
  darkCalDone = true;
}

// Hardware executes 40-sample temporal averaging to mitigate thermal and shot noise.
// Floating-point math is deferred to the host to circumvent Zephyr linker constraints.
void pollC12880MA() {
  uint32_t accum[N_PIXELS] = {0};
  int samples = 40;
  for (int s = 0; s < samples; s++) {
    getSpectrum(spec);
    for (int i = 0; i < N_PIXELS; i++) {
      int32_t v = spec[i] - (darkCalDone ? darkSpec[i] : 0);
      accum[i] += (v < 0) ? 0 : v;
    }
    delay(5);
  }

  cachedSpecStr = "[";
  for (int i = 0; i < N_PIXELS; i++) {
    currentSpec[i] = accum[i] / samples;
    cachedSpecStr += String(currentSpec[i]);
    if (i < N_PIXELS - 1) cachedSpecStr += ",";
  }
  cachedSpecStr += "]";
}

void auditHardware() {
  Wire1.begin();
  Wire1.setClock(400000);
  
  if (myGNSS.begin(Wire1)) {
    has_GNSS = true;
    myGNSS.setI2COutput(COM_TYPE_UBX);
    myGNSS.newCfgValset(); 
    myGNSS.addCfgValset8(UBLOX_CFG_SIGNAL_GLO_ENA, 0); 
    myGNSS.addCfgValset8(UBLOX_CFG_SIGNAL_BDS_ENA, 0); 
    myGNSS.sendCfgValset();
    myGNSS.setAutoPVT(true);
    myGNSS.setAutoNAVSAT(true);
  }

  pinMode(PIN_ST, OUTPUT);
  pinMode(PIN_CLK, OUTPUT);
  digitalWrite(PIN_ST, LOW);
  digitalWrite(PIN_CLK, LOW);
  analogReadResolution(12);
  has_C12880MA = true; 
  cachedSpecStr.reserve(2048);
  doDarkCalibration();
}

void pollGNSS() {
  myGNSS.checkUblox(); 
  if (myGNSS.getPVT()) {
    cachedLat = myGNSS.getLatitude();
    cachedLon = myGNSS.getLongitude();
    cachedAlt = myGNSS.getAltitudeMSL();
    cachedHAcc = myGNSS.getHorizontalAccEst();
    cachedVAcc = myGNSS.getVerticalAccEst();
  }
  if (myGNSS.getNAVSAT()) {
    cachedNumSVs = myGNSS.packetUBXNAVSAT->data.header.numSvs;
    cachedSvs = ""; cachedCno = ""; cachedGns = "";
    uint8_t limit = (cachedNumSVs > 6) ? 6 : cachedNumSVs; 
    for (uint8_t i = 0; i < limit; i++) {
      if (i > 0) { cachedSvs += ","; cachedCno += ","; cachedGns += ","; }
      cachedSvs += String(myGNSS.packetUBXNAVSAT->data.blocks[i].svId);
      cachedCno += String(myGNSS.packetUBXNAVSAT->data.blocks[i].cno);
      cachedGns += String(myGNSS.packetUBXNAVSAT->data.blocks[i].gnssId);
    }
  }
  UBX_MON_RF_data_t rfData;
  if (myGNSS.getRFinformation(&rfData)) {
    cachedAgc0 = rfData.blocks[0].agcCnt;
    cachedJam0 = rfData.blocks[0].jamInd;
    if (rfData.header.nBlocks > 1) {
      cachedAgc1 = rfData.blocks[1].agcCnt;
      cachedJam1 = rfData.blocks[1].jamInd;
    }
  }
}

void setup() {
  Bridge.begin();
  auditHardware();
}

void dispatchTelemetry() {
  static uint32_t last_send = 0;
  if (millis() - last_send > 10000) { 
    String json = "{";
    bool first = true;

    if (has_GNSS) {
      json += "\"latitude\":" + String(cachedLat);
      json += ",\"longitude\":" + String(cachedLon);
      json += ",\"altitude\":" + String(cachedAlt);
      json += ",\"hAcc\":" + String(cachedHAcc);
      json += ",\"vAcc\":" + String(cachedVAcc);
      json += ",\"numSVs\":" + String(cachedNumSVs);
      json += ",\"agcCnt0\":" + String(cachedAgc0) + ",\"jamInd0\":" + String(cachedJam0);
      json += ",\"agcCnt1\":" + String(cachedAgc1) + ",\"jamInd1\":" + String(cachedJam1);
      json += ",\"svID\":\"" + cachedSvs + "\",\"cno\":\"" + cachedCno + "\",\"gnssID\":\"" + cachedGns + "\"";
      first = false;
    }

    if (has_C12880MA) {
      if (!first) json += ",";
      json += "\"spectrum\":" + cachedSpecStr;
      json += ",\"integ_clks\":" + String(INTEG_CLKS);
    }
    
    json += "}";
    Bridge.notify("telemetry", json);
    last_send = millis();
  }
}

void loop() {
  Bridge.update();
  if (has_GNSS) pollGNSS();
  if (has_C12880MA) pollC12880MA();
  dispatchTelemetry();
}