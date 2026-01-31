import requests
from decimal import Decimal
from time import time
from datetime import datetime, timezone
import json
import os

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
device = "Under-Canopy"

# Random Data
lat    = str("37.4219983")
lon    = str("-122.084")
height = str("5.2")
date_time_string = make_time_stamp() # this is necessary for uploading

# make json / dic to upload
payload = { 
        "datetime" : date_time_string, 
        "device" : device,
        "lat" : lat,
        "lon": lon,
        "height_m": height
}

# # upload data 
# response = requests.post(url, json=payload)
# print(response.status_code, response.text)


with open("example_data_dump.txt", 'a') as f:
        json.dump(payload, f)
        f.write(os.linesep) # Add a newline character for the next record
