#include <Wire.h>
#include <SparkFun_u-blox_GNSS_v3.h>

SFE_UBLOX_GNSS myGNSS;

unsigned long lastRFTime = 0; // Timer for RF status updates

void printLLH() {
// get lat / lon / height from myGNSS functions
  if (myGNSS.getPVT() == true)
  {
    int32_t latitude = myGNSS.getLatitude();
    Serial.print(F("Lat: "));
    Serial.print(latitude);

    int32_t longitude = myGNSS.getLongitude();
    Serial.print(F(" Long: "));
    Serial.print(longitude);
    Serial.print(F(" (degrees * 10^-7)"));

    int32_t altitude = myGNSS.getAltitudeMSL(); // Altitude above Mean Sea Level
    Serial.print(F(" Alt: "));
    Serial.print(altitude);
    Serial.print(F(" (mm)"));

    Serial.println();
  }
}

void printSatelliteInfo() {
  // get CN0, svID, and gnssID from data
  // 1. Poll the module for the NAV-SAT packet

  myGNSS.checkUblox(); // Keep the library processing incoming data

  if (myGNSS.getNAVSAT() == true) {    
    // 2. Get the number of satellites detected
    // In v3 library, we access the header from the packetNAVSAT pointer
    uint8_t numSVs = myGNSS.packetUBXNAVSAT->data.header.numSvs;

    Serial.print(F("Satellites found: "));
    Serial.println(numSVs);

    // 3. Loop through each satellite and pull data
    // We only print the first few to avoid flooding the Serial Monitor
    for (uint8_t i = 0; i < numSVs; i++) {
      uint8_t svID = myGNSS.packetUBXNAVSAT->data.blocks[i].svId;
      uint8_t cno = myGNSS.packetUBXNAVSAT->data.blocks[i].cno; // Signal strength
      uint8_t gnssID = myGNSS.packetUBXNAVSAT->data.blocks[i].gnssId; // 0=GPS, 1=SBAS, 2=Galileo, 3=Beidou, 5=QZSS, 6=GLONASS

      Serial.print(F("  SV: ")); Serial.print(svID);
      Serial.print(F(" | GNSS: ")); Serial.print(gnssID);
      Serial.print(F(" | Strength (C/N0): ")); Serial.println(cno);
      
      if (i >= 5) { // Stop after 6 satellites for brevity
        Serial.println(F("  ... (rest omitted)"));
        break;
      }
    }
  } else {
    Serial.println(F("NAV-SAT poll failed."));
  }

  // IMPORTANT: Tell the library we are done with this packet
    //myGNSS.packetUBXNAVSAT->isValid = false;
}

void printRFMon() {
  // 2. Poll MON-RF every second
  // TO-DO: Check that this actually does what it says on the tin...

  // Create a timer so we don't spam the I2C bus
  static unsigned long lastRF = 0;
  if (millis() - lastRF > 2000) {
    lastRF = millis();

    // 1. Create a local structure to hold the RF data
    UBX_MON_RF_data_t rfData;

    // 2. Call the function using the address of our structure
    if (myGNSS.getRFinformation(&rfData)) {
      
      // 3. Access the data from the blocks
      // nBlocks tells us how many RF bands the chip is reporting (usually 2 on ZED-F9P)
      for (int i = 0; i < rfData.header.nBlocks; i++) {
        uint16_t agc = rfData.blocks[i].agcCnt;
        uint8_t jam = rfData.blocks[i].jamInd;

        Serial.print(F("RF Block "));
        Serial.print(i);
        Serial.print(F(": AGC="));
        Serial.print(agc);
        Serial.print(F(", Jamming="));
        Serial.println(jam);
      }
    } else {
      Serial.println(F("Failed to get RF Information."));
    }
  }

}

void noGLONASSBeidou() {
  // --- SHRINK PACKET SIZE FOR UNO R4 ---
  // the Arduino Uno R4 cannot handle packet sizes greater than 300-bytes
  // and unfortunately there is about 500 bytes worth of data. For now, lets 
  // disable non-American GNSS constellations like GLONASS and BeiDou
  Serial.println(F("Disabling GLONASS and BeiDou..."));

  // 1. Clear any previous configuration in the buffer
  myGNSS.newCfgValset(); 

  // 2. Add the commands to turn off the extra constellations
  // 0 = False/Disable
  myGNSS.addCfgValset8(UBLOX_CFG_SIGNAL_GLO_ENA, 0); 
  myGNSS.addCfgValset8(UBLOX_CFG_SIGNAL_BDS_ENA, 0); 

  // 3. Send the configuration to the module (using VALSET_LAYER_RAM)
  if (myGNSS.sendCfgValset() == false) {
    Serial.println(F("Failed to update constellation config."));
  } else {
    Serial.println(F("Constellations disabled. Packet size should now be < 300 bytes."));
  }

}

void setup() {
  Serial.begin(115200);
  while (!Serial); 
  
  // Use the standard Wire bus for header pins
  //Wire.begin();
  //Wire.setClock(400000); // ZED-F9P supports Fast Mode (400kHz)
  Wire1.begin();
  Wire1.setClock(400000);
  // myGNSS.enableDebugging(); // Keep this on to see the "ACK" logs

  if (myGNSS.begin(Wire1) == false) // Defaults to Wire and 0x42
  {
    Serial.println(F("u-blox GNSS not detected on Wire headers."));
    while (1);
  }
  
  myGNSS.setI2COutput(COM_TYPE_UBX);
  noGLONASSBeidou();
}

void loop() {

  
  // 1. Standard PVT Data
  printLLH();

  //2. get satellite info
  printSatelliteInfo();

  //3. Try to get automatic gain contr
  printRFMon();
}