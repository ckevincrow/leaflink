# -*- coding: utf-8 -*-
"""
Created on Sat Nov 22 20:53:48 2025

@author: db1950
"""

import serial
# import pynmea2
from pyubx2 import UBXReader
import time
import serial.tools.list_ports

def list_usb_ports():
    ports = serial.tools.list_ports.comports()
    for port in ports:
        print(f"{port.device} - {port.description}")
# int_f2 = lambda a: f"{a:02d}"

###############################################################################
# Inputs
###############################################################################
PORT     = "COM9"
BAUDRATE = 9600
fn1_out = "nav-sat.txt"
fn2_out = "mon-rf.txt"
fn3_out = "nav-posllh.txt"

# message keeping
timeout = 1 # how long to wait for the serial connection before giving up [s]
protfilter = 2 # only read ublox messages / ignore nmea # https://github.com/semuconsulting/pyubx2

iterations = 100
cit = 0


###############################################################################
with serial.Serial(PORT, BAUDRATE, timeout=timeout) as ser, open(fn1_out, "a") as nav_sat_file, open(fn2_out, "a") as mon_rf_file, open(fn3_out, "a") as nav_pos_file:
    ubr = UBXReader(ser, protfilter=protfilter)
    while cit < iterations:
        cit += 1
        try:
            (raw, parsed) = ubr.read()
            if parsed:
                # print(parsed)
                pd = parsed.__dict__
                
                if parsed.identity == "NAV-SAT":
                    numSvs = parsed.numSvs
                    # clear / initialize lists
                    vals = ["gnssId_", "svId_", "cno_", "elev_", "azim_"]
                    ns_str = f"{time.time()}, "
                    for a in range(numSvs):
                        cur_sv = f"{a+1:02d}"
                        for val in vals: 
                            fval = val + cur_sv
                            ns_str += f"{fval}={pd[fval]}, "
                    ns_str += f"iTOW={pd['iTOW']} "
                    nav_sat_file.write(f"{ns_str}\n")
                    nav_sat_file.flush()
                    
                elif parsed.identity == "NAV-POSLLH":
                    nav_pos_file.write(f"{time.time()}, iTOW={parsed.iTOW}, lat={parsed.lat}, lon={parsed.lon}, height={parsed.height} \n")
                    nav_pos_file.flush()
                    
                elif parsed.identity == "MON-RF":
                    mon_rf_file.write(f"{time.time()}, ACG1_Gain={parsed.agcCnt_01}, ACG2_Gain={parsed.agcCnt_02}\n")
                    mon_rf_file.flush()
                        
        except KeyboardInterrupt:
            break
        except Exception as e:
            print("UBX error:", e)