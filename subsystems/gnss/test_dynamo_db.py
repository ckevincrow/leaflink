import requests

from decimal import Decimal

from time import time

from datetime import datetime, timezone

import json

import os



def list_usb_ports():

    ports = serial.tools.list_ports.comports()

    for port in ports:

        print(f"{port.device} - {port.description}")

# int_f2 = lambda a: f"{a:02d}"



PORT     = "COM17"

BAUDRATE = 9600

fn1_out = "nav-sat.txt"                                                                                                              fn2_out = "mon-rf.txt"                                                                                                               fn3_out = "nav-posllh.txt"                                                                                                                                                                                                                                                with serial.Serial(PORT, BAUDRATE, timeout=timeout) as ser, open(fn1_out, "a") as nav_sat_file, open(fn2_out, "a") as mon_rf_file, open(fn3_out, "a") as nav_pos_file:                                                                                                        ubr = UBXReader(ser, protfilter=protfilter)                                                                                          while cit < iterations:                                                                                                                  cit += 1                                                                                                                             try:                                                                                                                                     (raw, parsed) = ubr.read()                                                                                                           if parsed:                                                                                                                               # print(parsed)                                                                                                                      pd = parsed.__dict__                                                                                                                                                                                                                                                      if parsed.identity == "NAV-SAT":                                                                                                         numSvs = parsed.numSvs          
