#pragma once
#include "novatel_oem7_driver/replayer_utils.hpp"
#include <sys/time.h>

#include <iostream>
#include <sstream>
#include <cstring>

namespace novatel_oem7_driver {

#define SECS_PER_WEEK (60L*60*24*7)
#define LEAP_SECOND 18 // since 2016
uint64_t gps2Utc(uint64_t gps_week, double gps_time) {
    // int gps_week;               // weeks from 1980-1-6 GPS time epoch
    // double gps_time;            // seconds within the week as double
    return uint64_t(SECS_PER_WEEK * gps_week * 1000000ULL + (gps_time * 1000000ULL) + 315964800000000ULL - LEAP_SECOND * 1000000ULL);
}

uint64_t gps2UtcWithMiilli(uint64_t gps_time) {
    // gps_time in milliseconds
    return uint64_t(gps_time + 315964800000ULL - LEAP_SECOND * 1000ULL);
}

double gps2UtcWithMilliDouble(double gps_time) {
    // gps_time in milliseconds as type double
    return gps_time + 315964800000.0 - 18.0 * 1000.0;
}

}
