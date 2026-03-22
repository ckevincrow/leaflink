# View URL: http://ec2-18-224-219-60.us-east-2.compute.amazonaws.com/scan_simple2.php
import json
import urllib.request
import math
from time import sleep
from datetime import datetime, timezone
from arduino.app_utils import App, Bridge

url = "https://7hcxwqqt37.execute-api.us-east-2.amazonaws.com/leaflink-default/leaflink-lambda-function"
device = "cletus"
state = {}

N_PIXELS = 288
L_MIN = 340.0
L_MAX = 850.0
DELTA_NM = (L_MAX - L_MIN) / (N_PIXELS - 1)
K_E = .005

def pixel_to_nm(p):
    return L_MIN + DELTA_NM * float(p)

def v_lambda(nm):
    if nm < 380.0 or nm > 780.0: return 0.0
    
    x = nm - 555.0
    sigma = 35.0 if nm < 555.0 else 45.0
    return math.exp(-0.5 * x * x / (sigma * sigma))

# Optical integrals (PAR/Lux) and algebraic reductions execute on the host
# to avoid firmware linker faults associated with missing math libraries (libm).
def compute_optical_metrics(spec, integ_clks = 20): # Ensure this matches your Arduino!
    if not spec or len(spec) != N_PIXELS: return {}
    
    # 1. Normalize by integration time to stop 'clock drift'
    norm_spec = [float(s) / float(integ_clks) for s in spec]
    
    mn = min(norm_spec)
    mx = max(norm_spec)
    mean_val = sum(norm_spec) / N_PIXELS
    pk_idx = norm_spec.index(mx)
    pk_nm = pixel_to_nm(pk_idx)
    
    raw_int = 0.0
    photons = 0.0
    h = 6.626e-34
    c = 2.998e8
    NA = 6.022e23
    
    for i in range(N_PIXELS):
        nm = pixel_to_nm(i)
        v = v_lambda(nm)
        
        # USE NORM_SPEC HERE
        if v > 0.0:
            raw_int += norm_spec[i] * v * DELTA_NM
            
        if 400.0 <= nm <= 700.0:
            # USE NORM_SPEC HERE
            E = norm_spec[i] * K_E * DELTA_NM
            Ep = (h * c) / (nm * 1e-9)
            photons += E / Ep
            
    # Calculate PAR (Final result in umol/m2/s)
    par = (photons / NA) * 1e6
    
    # Calculate Lux (Applying a calibrated multiplier for ~700 Lux)
    # 0.005 is a standard starting point for this sensor
    lux = raw_int * 4.7 

    metrics = {
        "03_counts_min": int(mn * integ_clks), 
        "04_counts_max": int(mx * integ_clks),
        "05_counts_mean": round(mean_val * integ_clks),
        "06_peak_pixel": int(pk_idx),
        "07_peak_nm": round(pk_nm),
        "21_PAR_umol_m2_s": round(par *1.5), 
        "22_lux": round(lux)           # NO MORE * 1000
    }
    
    # Populate the ADC wavelength bins
    for i in range(13):
        target_nm = 400.0 + (i * 25.0)
        best_idx = min(range(N_PIXELS), key=lambda j: abs(pixel_to_nm(j) - target_nm))
        metrics[f"{i+8:02d}_adc_{int(target_nm)}nm"] = int(spec[best_idx])
        
    return metrics

def on_telemetry(payload):
    try:
        data = json.loads(payload)
        state.update(data)
    except:
        pass

Bridge.provide("telemetry", on_telemetry)

def loop():
    if not state: return
    
    timestamp = datetime.now(timezone.utc).isoformat()
    aws_payload = {
        "datetime": timestamp,
        "device": device,
        "01_timestamp": timestamp,
        "hostMs": str(int(datetime.now().timestamp() * 1000))
    }
    
    if "latitude" in state:
        aws_payload.update({
            "latitude": state.get("latitude"),
            "longitude": state.get("longitude"),
            "altitude": state.get("altitude"),
            "hAcc_mm": state.get("hAcc"),
            "vAcc_mm": state.get("vAcc"),
            "numSVs": state.get("numSVs"),
            "agc_cnt_0": state.get("agcCnt0"),
            "jam_ind_0": state.get("jamInd0"),
            "agc_cnt_1": state.get("agcCnt1"),
            "jam_ind_1": state.get("jamInd1"),
            "svIDs": state.get("svID"),
            "cn0s": state.get("cno"),
            "gnssIDs": state.get("gnssID")
        })

    if "spectrum" in state:
        current_clks = state.get("integ_clks", 20)
        optics = compute_optical_metrics(state.get("spectrum"))
        aws_payload.update(optics)

    print(json.dumps(aws_payload))
    
    try:
        req = urllib.request.Request(url, data=json.dumps(aws_payload).encode('utf-8'))
        req.add_header('Content-Type', 'application/json')
        with urllib.request.urlopen(req, timeout=5) as r: pass
    except: pass
    
    sleep(10)

if __name__ == "__main__":
    App.run(user_loop=loop)