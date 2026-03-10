# 😀 GNSS
Raygen Yeatman
2-13-2026

### Description
# LeafLink GNSS → AWS Uploader (Arduino UNO R4 WiFi + u-blox ZED-F9P)

This project reads live GNSS position data from a u-blox **ZED-F9P** (via I2C) using the SparkFun GNSS library, then uploads each valid 3D fix to an **AWS API Gateway** endpoint over HTTPS. Each upload matches the payload format (`datetime`, `device`, `lat`, `lon`, `height_m`).

---

## What this sketch does

On a repeating interval (default: every 2 seconds), the Arduino:

1. Talks to the ZED-F9P over **I2C**
2. Waits for a **valid 3D GNSS fix**
3. Pulls:
   - Latitude
   - Longitude
   - Height/altitude
   - GNSS UTC date/time
4. Builds a JSON payload:

```json
{
  "datetime": "2026-02-18T16:12:34.567Z",
  "device": "Under-Canopy",
  "lat": "33.1234567",
  "lon": "-90.1234567",
  "height_m": "123.400"
}





