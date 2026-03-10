import requests
from decimal import Decimal
from time import time, sleep
from datetime import datetime, timezone
import json
import os

# -------- ADDED: GNSS over I2C (Qwiic) ----------
from smbus2 import SMBus
from pyubx2 import UBXReader, UBX_PROTOCOL
# -----------------------------------------------

def make_time_stamp():
    dt_object = datetime.now(timezone.utc)

    # Convert to ISO format string with milliseconds precision
    # The default output uses a +00:00 offset for UTC
    iso_ts = dt_object.isoformat(timespec="milliseconds")

    # Replace the offset with 'Z'
    date_time_string = iso_ts.replace("+00:00", "Z")
    return date_time_string


# this is constant for dynamodb / AWS handling
url = "https://7hcxwqqt37.execute-api.us-east-2.amazonaws.com/leaflink-default/leaflink-lambda-function"

# select your device
device = "Open-Sky"

# -------- ADDED: u-blox I2C stream wrapper ----------
# ZED-F9P default I2C 7-bit address is commonly 0x42.
# The u-blox I2C stream uses:
#   0xFD/0xFE = bytes available (LSB/MSB)
#   0xFF      = stream data register
class UbloxI2CStream:
    def __init__(self, bus_num=1, addr=0x42, timeout_s=2.0):
        self.bus = SMBus(bus_num)
        self.addr = addr
        self.timeout_s = timeout_s
        self.buf = bytearray()

    def close(self):
        try:
            self.bus.close()
        except Exception:
            pass

    def _bytes_available(self):
        lsb = self.bus.read_byte_data(self.addr, 0xFD)
        msb = self.bus.read_byte_data(self.addr, 0xFE)
        return (msb << 8) | lsb

    def _fill(self, want_at_least):
        deadline = time() + self.timeout_s
        while len(self.buf) < want_at_least and time() < deadline:
            avail = self._bytes_available()
            if avail <= 0:
                sleep(0.01)
                continue

            # SMBus block reads are often capped at 32 bytes
            to_read = min(avail, 32)
            chunk = self.bus.read_i2c_block_data(self.addr, 0xFF, to_read)
            self.buf.extend(bytes(chunk))

    def read(self, n):
        if n <= 0:
            return b""
        self._fill(n)
        if not self.buf:
            return b""
        out = self.buf[:n]
        del self.buf[:n]
        return bytes(out)
# ----------------------------------------------------

# -------- ADDED: Open I2C GNSS stream + UBX reader ----
I2C_BUS  = int(os.getenv("GNSS_I2C_BUS", "1"))
I2C_ADDR = int(os.getenv("GNSS_I2C_ADDR", "0x42"), 0)

stream = UbloxI2CStream(bus_num=I2C_BUS, addr=I2C_ADDR, timeout_s=2.0)
ubr = UBXReader(stream, protfilter=UBX_PROTOCOL)  # UBX only
# ------------------------------------------------------

# -------- ADDED: Loop forever pulling GNSS + uploading --
while True:
    raw, msg = ubr.read()
    if msg is None:
        continue

    ident = getattr(msg, "identity", "")

    # Pull position from ZED-F9P
    # Prefer NAV-PVT (includes fix quality), fallback NAV-POSLLH
    if ident == "NAV-PVT":
        fix_type = int(getattr(msg, "fixType", 0))
        if fix_type < 3:
            continue  # require 3D fix

        lat_val = getattr(msg, "lat", None)  # usually 1e-7 degrees
        lon_val = getattr(msg, "lon", None)
        # prefer hMSL (mm) if present, else height
        h_val = getattr(msg, "hMSL", getattr(msg, "height", None))

        if lat_val is None or lon_val is None or h_val is None:
            continue

        lat_f = float(lat_val) / 1e7 if abs(float(lat_val)) > 180 else float(lat_val)
        lon_f = float(lon_val) / 1e7 if abs(float(lon_val)) > 180 else float(lon_val)
        height_f = float(h_val) / 1000.0 if abs(float(h_val)) > 10000 else float(h_val)

    elif ident == "NAV-POSLLH":
        lat_val = getattr(msg, "lat", None)     # usually 1e-7 degrees
        lon_val = getattr(msg, "lon", None)
        h_val   = getattr(msg, "height", None)  # usually mm

        if lat_val is None or lon_val is None or h_val is None:
            continue

        lat_f = float(lat_val) / 1e7 if abs(float(lat_val)) > 180 else float(lat_val)
        lon_f = float(lon_val) / 1e7 if abs(float(lon_val)) > 180 else float(lon_val)
        height_f = float(h_val) / 1000.0 if abs(float(h_val)) > 10000 else float(h_val)

    else:
        continue  # ignore other UBX messages

    # Your existing timestamp + payload formatting
    date_time_string = make_time_stamp()  # this is necessary for uploading

    lat    = str(f"{lat_f:.7f}")
    lon    = str(f"{lon_f:.7f}")
    height = str(f"{height_f:.3f}")

    # make json / dic to upload
    payload = {
        "datetime": date_time_string,
        "device": device,
        "lat": lat,
        "lon": lon,
        "height_m": height
    }

    # upload data (enabled)
    try:
        response = requests.post(url, json=payload, timeout=10)
        print(response.status_code, response.text)
    except Exception as e:
        print("UPLOAD ERROR:", e)

    with open("example_data_dump.txt", 'a') as f:
        json.dump(payload, f)
        f.write(os.linesep)  # Add a newline character for the next record

    # pacing (adjust if you want faster)
    sleep(1)
# --------------------------------------------------------

