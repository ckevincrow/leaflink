import json
import os
import time
from datetime import datetime, timezone

import requests
from smbus2 import SMBus
from pyubx2 import UBXReader, UBX_PROTOCOL

# =========================
# USER SETTINGS
# =========================
AWS_URL = "https://7hcxwqqt37.execute-api.us-east-2.amazonaws.com/leaflink-default/leaflink-lambda-function"
DEVICE = "Open-Sky"   # change to "Under-Canopy" on the other unit

I2C_BUS = 2           # Uno Q Qwiic bus from your earlier checks
I2C_ADDR = 0x42       # ZED-F9P default I2C address
STREAM_REG = 0xFF     # u-blox DDC stream register

UPLOAD_EVERY_S = 10.0
LOG_FILE = "example_data_dump.txt"

# =========================
# HELPERS
# =========================
def utc_ts() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds").replace("+00:00", "Z")

def debug(msg: str) -> None:
    print(msg, flush=True)

def bytes_available(bus: SMBus) -> int:
    lsb = bus.read_byte_data(I2C_ADDR, 0xFD, force=True)
    msb = bus.read_byte_data(I2C_ADDR, 0xFE, force=True)
    return (msb << 8) | lsb

def ubx_checksum(data: bytes) -> bytes:
    ck_a = 0
    ck_b = 0
    for b in data:
        ck_a = (ck_a + b) & 0xFF
        ck_b = (ck_b + ck_a) & 0xFF
    return bytes([ck_a, ck_b])

def build_ubx(msg_class: int, msg_id: int, payload: bytes) -> bytes:
    length = len(payload)
    hdr = bytes([0xB5, 0x62, msg_class & 0xFF, msg_id & 0xFF, length & 0xFF, (length >> 8) & 0xFF])
    chk = ubx_checksum(hdr[2:] + payload)
    return hdr + payload + chk

def write_stream(bus: SMBus, packet: bytes) -> None:
    for i in range(0, len(packet), 32):
        chunk = packet[i:i+32]
        bus.write_i2c_block_data(I2C_ADDR, STREAM_REG, list(chunk), force=True)
        time.sleep(0.01)

def cfg_msg_enable(msg_class: int, msg_id: int, rate_ddc: int = 1) -> bytes:
    # UBX-CFG-MSG legacy payload:
    # msgClass,msgID,rateDDC,rateUART1,rateUART2,rateUSB,rateSPI,reserved
    payload = bytes([msg_class, msg_id, rate_ddc, 0, 0, 0, 0, 0])
    return build_ubx(0x06, 0x01, payload)

def enable_i2c_messages(bus: SMBus) -> None:
    debug("Enabling NAV-PVT, NAV-SAT, MON-RF on I2C...")
    # NAV-PVT
    write_stream(bus, cfg_msg_enable(0x01, 0x07, 1))
    # NAV-SAT
    write_stream(bus, cfg_msg_enable(0x01, 0x35, 1))
    # MON-RF
    write_stream(bus, cfg_msg_enable(0x0A, 0x38, 1))

def parse_csv_ints(csv_str: str) -> list[int]:
    if not csv_str:
        return []
    vals = []
    for x in csv_str.split(","):
        x = x.strip()
        if not x:
            continue
        try:
            vals.append(int(x))
        except ValueError:
            pass
    return vals

# =========================
# CUSTOM I2C STREAM FOR pyubx2
# =========================
class UbloxI2CStream:
    def __init__(self, bus: SMBus):
        self.bus = bus
        self.buf = bytearray()

    def _fill(self, want: int) -> None:
        deadline = time.time() + 1.0
        while len(self.buf) < want and time.time() < deadline:
            try:
                avail = bytes_available(self.bus)
            except OSError as e:
                debug(f"bytes_available error: {e}")
                time.sleep(0.05)
                continue

            if avail <= 0:
                time.sleep(0.02)
                continue

            try:
                to_read = min(avail, 32)
                chunk = self.bus.read_i2c_block_data(I2C_ADDR, STREAM_REG, to_read, force=True)
                self.buf.extend(bytes(chunk))
            except OSError as e:
                debug(f"read_i2c_block_data error: {e}")
                time.sleep(0.05)

    def read(self, n: int) -> bytes:
        self._fill(n)
        if not self.buf:
            return b""
        out = self.buf[:n]
        del self.buf[:n]
        return bytes(out)

# =========================
# MAIN
# =========================
def main() -> None:
    debug(f"Opening I2C bus {I2C_BUS} at address 0x{I2C_ADDR:02X}")

    last_upload = 0.0
    last_status = 0.0

    # Cached GNSS values
    lat = None
    lon = None
    height_m = None
    fix_type = 0
    hacc_mm = 0
    vacc_mm = 0
    num_sv = 0
    sv_ids = ""
    cn0s = ""
    cn0_avg = "0.0"
    itow_ms = 0
    agc0 = 0
    jam0 = 0
    agc1 = 0
    jam1 = 0

    with SMBus(I2C_BUS) as bus:
        enable_i2c_messages(bus)
        time.sleep(0.5)

        try:
            debug(f"bytes_available after config = {bytes_available(bus)}")
        except Exception as e:
            debug(f"Initial bytes_available failed: {e}")

        stream = UbloxI2CStream(bus)
        ubr = UBXReader(stream, protfilter=UBX_PROTOCOL)

        while True:
            now = time.time()
            if now - last_status >= 2.0:
                last_status = now
                try:
                    debug(f"alive bytes_available = {bytes_available(bus)}")
                except Exception as e:
                    debug(f"alive probe failed: {e}")

            try:
                raw, msg = ubr.read()
            except Exception as e:
                debug(f"UBXReader error: {e}")
                time.sleep(0.1)
                continue

            if msg:
                debug(f"msg: {msg.identity}")

                if msg.identity == "NAV-PVT":
                    fix_type = int(getattr(msg, "fixType", 0))
                    itow_ms = int(getattr(msg, "iTOW", 0))

                    lat_raw = getattr(msg, "lat", None)
                    lon_raw = getattr(msg, "lon", None)
                    hmsl_mm = getattr(msg, "hMSL", None)

                    if lat_raw is not None and lon_raw is not None:
                        lat = float(lat_raw) / 1e7
                        lon = float(lon_raw) / 1e7

                    if hmsl_mm is not None:
                        height_m = float(hmsl_mm) / 1000.0

                    if hasattr(msg, "hAcc"):
                        hacc_mm = int(getattr(msg, "hAcc"))
                    if hasattr(msg, "vAcc"):
                        vacc_mm = int(getattr(msg, "vAcc"))

                elif msg.identity == "NAV-SAT":
                    num_sv = int(getattr(msg, "numSvs", 0) or 0)
                    md = msg.__dict__

                    sv_list = []
                    cno_list = []

                    for i in range(1, num_sv + 1):
                        idx = f"{i:02d}"
                        sv = md.get(f"svId_{idx}", None)
                        cno = md.get(f"cno_{idx}", None)

                        if sv is not None and cno is not None:
                            sv_list.append(str(int(sv)))
                            cno_list.append(str(int(cno)))

                    sv_ids = ",".join(sv_list)
                    cn0s = ",".join(cno_list)

                    ints = parse_csv_ints(cn0s)
                    cn0_avg = str(round(sum(ints) / len(ints), 2)) if ints else "0.0"

                elif msg.identity == "MON-RF":
                    d = msg.__dict__
                    agc0 = int(d.get("agcCnt_01", 0) or 0)
                    jam0 = int(d.get("jamInd_01", 0) or 0)
                    agc1 = int(d.get("agcCnt_02", 0) or 0)
                    jam1 = int(d.get("jamInd_02", 0) or 0)

            now = time.time()
            if (now - last_upload) >= UPLOAD_EVERY_S:
                last_upload = now

                # Only upload valid-looking GNSS data
                if lat is None or lon is None or height_m is None or fix_type < 2:
                    debug("Waiting for valid GNSS fix...")
                    continue

                payload = {
                    "datetime": utc_ts(),
                    "fixType": fix_type,
                    "lon": str(round(lon, 7)),
                    "height_m": str(round(height_m, 2)),
                    "hAcc_mm": int(hacc_mm),
                    "numSV": int(num_sv),
                    "cn0s": cn0s,
                    "svIds": sv_ids,
                    "lat": str(round(lat, 7)),
                    "timestamp_lambda": utc_ts(),
                    "hostMs": str(int(time.time() * 1000)),
                    "cn0Avg_dBHz": cn0_avg,
                }

                debug(f"Uploading: {payload}")

                try:
                    r = requests.post(AWS_URL, json=payload, timeout=10)
                    debug(f"AWS {r.status_code} {r.text[:200]}")
                except Exception as e:
                    debug(f"AWS post failed: {e}")

                with open(LOG_FILE, "a") as f:
                    json.dump(payload, f)
                    f.write("\n")


if __name__ == "__main__":
    main()
