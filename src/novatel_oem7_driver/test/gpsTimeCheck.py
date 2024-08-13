import datetime
import numpy as np
import ruamel.yaml

from gps_time.core import GPSTime
from gps_time.datetime import datetime2tow
# pip install gps-time


## Some sanity check on conversion from GPS time to UTC time
# gps_time1 = GPSTime(2322, 309298.776)
# # Adjust for leap seconds, 18 seconds since 2016-12-31
# # The next possible date is June 30, 2025.
# utc_time = gps_time1 - 18
# # print(gps_time1)
# print(f"Datetime: {gps_time1.to_datetime()}")
# print(f"Datetime: {utc_time.to_datetime()}")


# UTC this is not corrected
now = datetime.datetime.now().timestamp()
nowdt = datetime.datetime.now()
print(f"UTC Not corrected Datetime: {nowdt}")

gps_time = GPSTime.from_datetime(nowdt) + 18
print(f"GPS Corrected Datetime: {gps_time.to_datetime()}")

# Calculate the current GPS week and seconds since the start of GPS epoch (January 6, 1980 00:00:00 UTC)
gps_epoch_start = datetime.datetime(1980, 1, 6, 0, 0, 0)
delta_seconds = now - datetime.datetime.timestamp(gps_epoch_start) + 18 - 3600
gps_weeks = int(delta_seconds / 604800)  # 604800 seconds in a GPS week
gps_seconds = delta_seconds % 604800.0  # 18 leap seconds since 1980
print(f"GPS Week: {gps_weeks}, GPS Seconds: {gps_seconds}")

gps_time_reverse = GPSTime(gps_weeks, gps_seconds)
print(f"Jonas correction, GPS Corrected Datetime: {gps_time_reverse.to_datetime()}")
