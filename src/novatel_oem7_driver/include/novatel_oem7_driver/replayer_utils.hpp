#pragma once
#include "novatel_oem7_driver/replayer.hpp"
#include <ros/ros.h>
#include <thread>


namespace novatel_oem7_driver {

void setThreadPriority(std::thread* thread, int priority);
void setSelfThreadPriority(int priority);
void setSelfThreadAffinity(int core_id);
uint64_t getCurrentTime();
uint64_t getMonotonicTime();
uint64_t gps2Utc(uint64_t gps_week, double gps_time);
uint64_t gps2UtcWithMiilli(uint64_t gps_time);
double gps2UtcWithMilliDouble(double gps_time);
int getGPSweek(const uint64_t &stamp);
double getGPSsecond(const uint64_t &stamp);
int setSystemTime(uint64_t time);
// void createDirectory(std::string path);
// std::string splitFilename(const std::string& str);
// void backtrace_handle_init(void);


template <
    class result_t   = std::chrono::milliseconds,
    class clock_t    = std::chrono::steady_clock,
    class duration_t = std::chrono::milliseconds
>
auto since(std::chrono::time_point<clock_t, duration_t> const& start)
{
    return std::chrono::duration_cast<result_t>(clock_t::now() - start);
}

}
