import json
import urllib.request
from time import sleep
from datetime import datetime, timezone
from arduino.app_utils import App, Bridge

ENABLE_DEBUG_PRINT = True
ENABLE_JSON_PRINT = True

url = "https://7hcxwqqt37.execute-api.us-east-2.amazonaws.com/leaflink-default/leaflink-lambda-function"

device = "Open-SkyQrgy111"
# device = "Under-CanopyQ"

state = {}
idle_cycles = 0

def on_cpp_payload(payload):
    try:
        state.update(json.loads(payload))
    except Exception as e:
        if ENABLE_DEBUG_PRINT:
            print(f"PYTHON HOST JSON ERROR: {e}", flush=True)

def on_debug_payload(payload):
    if ENABLE_DEBUG_PRINT:
        print(f"C++ DEBUG: {payload}", flush=True)

Bridge.provide("cpp_to_python", on_cpp_payload)
Bridge.provide("debug", on_debug_payload)

def loop():
    global idle_cycles

    if not state:
        idle_cycles += 1
        if idle_cycles % 100 == 0 and ENABLE_DEBUG_PRINT:
            print("PYTHON HOST: Awaiting C++ payload...", flush=True)
        sleep(0.1)
        return

    idle_cycles = 0

    now_iso = datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")

    lat_deg = state.get("latitude", 0) / 1e7
    lon_deg = state.get("longitude", 0) / 1e7
    height_m = state.get("altitude", 0) / 1000.0

    payload = {
        "datetime": now_iso,
        "timestamp_lambda": now_iso,
        "device": device,
        "hostMs": str(int(datetime.now().timestamp() * 1000)),
        "fixType": int(state.get("fixType", 0)),
        "lon": str(round(lon_deg, 7)),
        "height_m": str(round(height_m, 2)),
        "hAcc_mm": int(state.get("hAcc", 0)),
        "numSV": int(state.get("numSVs", 0)),
        "svIds": state.get("svID", "") or "",
        "lat": str(round(lat_deg, 7)),
        "iTOW_ms": int(state.get("iTOW_ms", 0)),
    }

    if ENABLE_JSON_PRINT:
        print(f"PYTHON HOST: Dispatching JSON -> {json.dumps(payload)}", flush=True)

    try:
        req = urllib.request.Request(url, data=json.dumps(payload).encode("utf-8"))
        req.add_header("Content-Type", "application/json")
        with urllib.request.urlopen(req, timeout=5) as r:
            if ENABLE_JSON_PRINT:
                print(f"PYTHON HOST: AWS Response -> {r.read().decode('utf-8')}", flush=True)
    except Exception as e:
        if ENABLE_DEBUG_PRINT:
            print(f"PYTHON HOST FAULT: {e}", flush=True)

    sleep(2)

if __name__ == "__main__":
    App.run(user_loop=loop)
