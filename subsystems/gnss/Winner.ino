#define ENABLE_DEBUG 1

#include <Wire.h>
#include <Arduino_RouterBridge.h>
#include <SparkFun_u-blox_GNSS_v3.h>

const uint8_t I2C_ADDR_GNSS = 0x42;

SFE_UBLOX_GNSS myGNSS;
bool has_GNSS = false;

// Cached GNSS values
int32_t cachedLat = 0;
int32_t cachedLon = 0;
int32_t cachedAlt = 0;      // mm
uint8_t cachedFixType = 0;
uint32_t cachedHAcc = 0;    // mm
uint32_t cachedITOW = 0;    // ms
uint8_t cachedNumSVs = 0;

String cachedSvIDs = "";

void logDebug(const String &msg) {
#if ENABLE_DEBUG
  Bridge.notify("debug", msg);
  Bridge.update();
  delay(25);
#endif
}

bool probeI2C(uint8_t address, TwoWire &port) {
  port.beginTransmission(address);
  return (port.endTransmission() == 0);
}

void initGNSS() {
  logDebug("Starting GNSS init...");

  Wire1.begin();
  delay(1500);
  Wire1.setClock(100000);
  delay(1500);

  if (!probeI2C(I2C_ADDR_GNSS, Wire1)) {
    logDebug("GNSS not detected at 0x42");
    return;
  }

  logDebug("GNSS detected at 0x42");

  bool beginOK = false;
  for (int attempt = 1; attempt <= 5; attempt++) {
    logDebug("GNSS begin attempt " + String(attempt));
    delay(750);

    if (myGNSS.begin(Wire1, I2C_ADDR_GNSS)) {
      beginOK = true;
      break;
    }
  }

  if (!beginOK) {
    logDebug("myGNSS.begin(Wire1) failed");
    return;
  }

  has_GNSS = true;
  logDebug("myGNSS.begin(Wire1) succeeded");

  delay(250);
  myGNSS.setI2COutput(COM_TYPE_UBX);
  logDebug("setI2COutput(COM_TYPE_UBX) done");

  delay(150);
  myGNSS.setAutoPVT(true);
  logDebug("setAutoPVT(true) done");

  delay(150);
  myGNSS.setAutoNAVSAT(true);
  logDebug("setAutoNAVSAT(true) done");

  logDebug("GNSS init complete");
}

void pollGNSS() {
  static uint32_t lastNavSatDebug = 0;

  myGNSS.checkUblox();
  myGNSS.checkCallbacks();

  if (myGNSS.getPVT()) {
    cachedFixType = myGNSS.getFixType();
    cachedLat = myGNSS.getLatitude();
    cachedLon = myGNSS.getLongitude();
    cachedAlt = myGNSS.getAltitudeMSL();
    cachedHAcc = myGNSS.getHorizontalAccEst();
    cachedITOW = myGNSS.getTimeOfWeek();
    cachedNumSVs = myGNSS.getSIV();
  }

  bool gotNavSat = myGNSS.getNAVSAT();

  if (gotNavSat) {
    uint8_t navsatNum = myGNSS.packetUBXNAVSAT->data.header.numSvs;

    String newSvIDs = "";

    for (uint8_t i = 0; i < navsatNum; i++) {
      uint8_t svID = myGNSS.packetUBXNAVSAT->data.blocks[i].svId;

      if (i > 0) {
        newSvIDs += ",";
      }
      newSvIDs += String(svID);
    }

    if (newSvIDs.length() > 0) {
      cachedSvIDs = newSvIDs;
    }

    if (millis() - lastNavSatDebug > 5000) {
      logDebug("NAVSAT ok, header numSvs = " + String(navsatNum) + ", svIds = " + newSvIDs);
      lastNavSatDebug = millis();
    }
  } else {
    if (millis() - lastNavSatDebug > 5000) {
      logDebug("NAVSAT not updated");
      lastNavSatDebug = millis();
    }
  }
}

void sendPayload() {
  static uint32_t lastSend = 0;
  if (millis() - lastSend < 2000) return;

  if (!has_GNSS) {
    logDebug("Skipping payload because GNSS init failed");
    lastSend = millis();
    return;
  }

  String json = "{";
  json += "\"latitude\":" + String(cachedLat);
  json += ",\"longitude\":" + String(cachedLon);
  json += ",\"altitude\":" + String(cachedAlt);
  json += ",\"fixType\":" + String(cachedFixType);
  json += ",\"hAcc\":" + String(cachedHAcc);
  json += ",\"iTOW_ms\":" + String(cachedITOW);
  json += ",\"numSVs\":" + String(cachedNumSVs);
  json += ",\"svID\":\"" + cachedSvIDs + "\"";
  json += "}";

  Bridge.notify("cpp_to_python", json);
  logDebug("GNSS payload sent");
  lastSend = millis();
}

void setup() {
  Bridge.begin();
  delay(3000);
  logDebug("Firmware init start");
  initGNSS();
}

void loop() {
  Bridge.update();

  if (has_GNSS) {
    pollGNSS();
    sendPayload();
  } else {
    static uint32_t lastFailMsg = 0;
    if (millis() - lastFailMsg > 5000) {
      logDebug("GNSS not initialized; no payload sent");
      lastFailMsg = millis();
    }
  }

  delay(100);
}
