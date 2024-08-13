#pragma once
#include "novatel_oem7_driver/replayer.hpp"

namespace novatel_oem7_driver {

uint64_t getCurrentTime();
uint64_t getMonotonicTime();
uint64_t gps2Utc(uint64_t gps_week, double gps_time);
uint64_t gps2UtcWithMiilli(uint64_t gps_time);
double gps2UtcWithMilliDouble(double gps_time);

}
