#pragma once
#include <novatel_oem7_driver/oem7_receiver_if.hpp>
#include <ros/ros.h>
#include <filesystem>
#include <rosbag/view.h>
#include <rosbag/bag.h>
#include <novatel_oem7_driver/oem7_ros_messages.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <novatel_oem7_driver/oem7_messages.h>
#include <novatel_oem7_driver/oem7_imu.hpp>
#include "sensor_msgs/Imu.h"
#include "novatel_oem7_msgs/CORRIMU.h"
#include "novatel_oem7_msgs/IMURATECORRIMU.h"
#include "novatel_oem7_msgs/INSSTDEV.h"
#include "novatel_oem7_msgs/INSCONFIG.h"
#include "novatel_oem7_msgs/TIME.h"
#include "novatel_oem7_msgs/HEADING2.h"
#include "novatel_oem7_msgs/BESTGNSSPOS.h"
#include "novatel_oem7_msgs/BESTPOS.h"
#include "novatel_oem7_msgs/BESTVEL.h"
#include "novatel_oem7_msgs/BESTUTM.h"
#include "novatel_oem7_msgs/PPPPOS.h"
#include "novatel_oem7_msgs/TERRASTARINFO.h"
#include "novatel_oem7_msgs/TERRASTARSTATUS.h"
#include "novatel_oem7_msgs/INSPVA.h"
#include "novatel_oem7_msgs/INSPVAX.h"
#include "novatel_oem7_msgs/RXSTATUS.h"
#include <stdint.h>
#include "novatel_oem7_msgs/Oem7Header.h"
#include <novatel_oem7_driver/oem7_message_util.hpp>
#include <oem7_driver_util.hpp>
#include "nav_msgs/Odometry.h"
#include "gps_common/GPSFix.h"
#include "sensor_msgs/NavSatFix.h"
#include "geometry_msgs/Point.h"
#include <boost/scoped_ptr.hpp>
#include <oem7_ros_publisher.hpp>
#include <math.h>
#include <map>
#include <chrono>

namespace novatel_oem7_driver {

class RosbagRangeDataProcessorRos{
 public:
  RosbagRangeDataProcessorRos(ros::NodeHandlePtr nh);
  ~RosbagRangeDataProcessorRos() = default;

  void initialize();
  void initCommonRosStuff();

  bool processIMURosbag();
  bool processGNSSRosbag();
  bool processOdometryRosbag();
  bool associateAndWriteOdometryMsgs();
  bool associateAndWriteIMUmsgs();
  bool associateAndWriteGPSmsgs();
  bool createOutputDirectory();
  bool validateTopicsInRosbag(const rosbag::Bag& bag, const std::vector<std::string>& mandatoryTopics);

inline double gpsToRosTime(int week, int milliseconds) {
  // Step 1: Calculate total GPS time in seconds since the GPS epoch
  double gps_time_seconds = week * SECONDS_IN_WEEK + milliseconds / 1000.0;

  // Step 2: Adjust GPS time to UTC by subtracting the leap seconds
  double utc_time_seconds = gps_time_seconds - LEAP_SECONDS;

  // Step 3: Convert UTC time to ROS (Unix) time by adding the difference between Unix and GPS epochs
  double ros_time_seconds = utc_time_seconds + GPS_EPOCH_UNIX;

  return ros_time_seconds;
}


  inline double elapsedMilliseconds()
  {
    return std::chrono::duration_cast<std::chrono::milliseconds>(endTime_ - startTime_).count();
  }
  
  inline double elapsedSeconds()
  {
      return elapsedMilliseconds() / 1000.0;
  }

 private:

  const int SECONDS_IN_WEEK = 604800;   // 7 * 24 * 60 * 60 seconds in a week
  const int GPS_EPOCH_UNIX = 315964800; // Unix time of GPS epoch (January 6, 1980)
  const int LEAP_SECONDS = 18;          // Difference between GPS time and UTC in seconds


  std::string inputRosbagBasePath_;
  std::string inputRosbagName_;
  std::string outputRosbagName_;
  std::string rosbagFullname_ ;
  std::string rosbagOutFullname_ ;
  std::queue<sensor_msgs::Imu> rosIMUQueue_;
  std::queue<novatel_oem7_msgs::CORRIMU> corrIMUquque_;

  std::queue<sensor_msgs::NavSatFix> rosFixQueue_;
  std::queue<gps_common::GPSFix> gpsCommonQuque_;

  std::queue<novatel_oem7_msgs::INSPVA> INSPVAQueue_;
  std::queue<nav_msgs::Odometry> odometryQuque_;
  int64_t lastOdometryTime_ = 0;
  int64_t lastInspvaTime_ = 0;
  bool skipOdometry_ = false;
  bool skipInspva_ = false;

  ros::Time tracker;
  int64_t totalMsec_prev = 0;
  int64_t totalMsec = 0;
  double rosCompatibleTime_prev = 0;

  rosbag::Bag outBag;
  // rosbag::Bag inputBag;
  ros::NodeHandlePtr nh_;

  std::chrono::time_point<std::chrono::steady_clock> startTime_;
  std::chrono::time_point<std::chrono::steady_clock> endTime_;

};

}
