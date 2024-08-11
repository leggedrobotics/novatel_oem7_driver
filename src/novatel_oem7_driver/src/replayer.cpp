/*
 * RosbagRangeDataProcessorRos.cpp
 *
 *  Created on: Apr 21, 2022
 *      Author: jelavice
 */

#include "novatel_oem7_driver/replayer.hpp"
#include <ros/ros.h>
#include <cstdlib>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include "novatel_oem7_driver/replayer_utils.hpp"

namespace novatel_oem7_driver {

RosbagRangeDataProcessorRos::RosbagRangeDataProcessorRos(ros::NodeHandlePtr nh) : nh_(nh) {}

void RosbagRangeDataProcessorRos::initialize() {
  initCommonRosStuff();

  // Remove exsisting rosbag
  rosbagFullname_ = inputRosbagBasePath_ + inputRosbagName_;
  rosbagOutFullname_ = inputRosbagBasePath_ + outputRosbagName_;
  std::remove(rosbagOutFullname_.c_str());

  if (!createOutputDirectory()) {
    ROS_ERROR("Failed to create output directory. Exiting.");
    return;
  }

  // We are ready to initiatate the outbag
  outBag.open(rosbagOutFullname_, rosbag::bagmode::Write);
  if (!processIMURosbag())
  {
    ROS_ERROR("IMU Rosbag processing failed. Exiting.");
    outBag.close();
    return;
  }

  if (!processGNSSRosbag())
  {
    ROS_ERROR("GNSS Rosbag processing failed. Exiting.");
    outBag.close();
    return;
  }

  if (!processOdometryRosbag())
  {
    ROS_ERROR("Odometry Rosbag processing failed. Exiting.");
      outBag.close();
    return;
  }

  outBag.close();


}

bool RosbagRangeDataProcessorRos::createOutputDirectory() {
  // TODO(TT) This folder is currently not used.

  // Check if the output folder exists.
  if (std::filesystem::is_directory(inputRosbagBasePath_)) {
    return true;
  }

  // If the folder doesn't exist, create it.
  try {
    return std::filesystem::create_directories(inputRosbagBasePath_);
  } catch (const std::exception& exception) {
    ROS_ERROR_STREAM("Caught an exception trying to create output folder: " << exception.what());
  }

  return false;
}

void RosbagRangeDataProcessorRos::initCommonRosStuff() {
  inputRosbagBasePath_ = nh_->param<std::string>("inputRosbagBasePath", "");
  inputRosbagName_ = nh_->param<std::string>("inputBagName", "");
  outputRosbagName_ = nh_->param<std::string>("outputBagName", "");

  // Printout the parameters.
  ROS_INFO_STREAM("inputRosbagBasePath_: " << inputRosbagBasePath_);
  ROS_INFO_STREAM("inputRosbagName_: " << inputRosbagName_);
  ROS_INFO_STREAM("outputRosbagName_: " << outputRosbagName_);
  
}

bool RosbagRangeDataProcessorRos::validateTopicsInRosbag(const rosbag::Bag& bag, const std::vector<std::string>& mandatoryTopics) {
  // Get a view on the data and check if all mandatory topics are present.
  bool areMandatoryTopicsInRosbag{true};

  // Iterate over the mandatory topics and check if they are in the bag.
  for (const auto& topic : mandatoryTopics) {
    rosbag::View topicView(bag, rosbag::TopicQuery(topic));
    if (topicView.size() == 0u) {
      ROS_ERROR_STREAM("Topic '" << topic << "' is not in the rosbag.");
      areMandatoryTopicsInRosbag = false;
    } else {
      ROS_INFO_STREAM("Topic '" << topic << "' is in the rosbag.");
    }      // if
  }        // for

  if (!areMandatoryTopicsInRosbag) {
    ROS_ERROR("All required topics are not within the rosbag. Replay operations are terminated. Waiting user to terminate.");
    return false;
  }

  return true;
}

bool RosbagRangeDataProcessorRos::processOdometryRosbag() {
  std::vector<std::string> odomTopics;

  odomTopics.push_back("/gt_box/cpt7/odom");
  odomTopics.push_back("/gt_box/cpt7/inspva");

  // Create me a high resolution clock timer from std chrono
  // Start the timer.
  m_StartTime = std::chrono::steady_clock::now();

  // Open ROS bag.
  rosbag::Bag bag;
  try {
    bag.open(rosbagFullname_, rosbag::bagmode::Read);
  } catch (const rosbag::BagIOException& e) {
    ROS_ERROR_STREAM("Error opening ROS bag: '" << rosbagFullname_ << "'");
    return false;
  }
  ROS_INFO_STREAM("ROS bag '" << rosbagFullname_ << "' open.");

  if (!validateTopicsInRosbag(bag, odomTopics)) {
    bag.close();
    return false;
  }

  ROS_INFO_STREAM("\033[92m"
                  << " Here we go... "
                  << "\033[0m");
  const ros::WallTime first{ros::WallTime::now() + ros::WallDuration(1.0)};
  ros::WallTime::sleepUntil(first);

  // // The bag view we iterate over.
  rosbag::View view(bag, rosbag::TopicQuery(odomTopics));

  ros::Time stampLastIteration{view.getBeginTime()};
  ros::WallTime wallStampLastIteration{ros::WallTime::now()};
  const ros::WallTime wallStampStartSequentialRun{ros::WallTime::now()};

  for (const auto& messageInstance : view) {
    // If the node is shutdown, stop processing and do early return.
    if (!ros::ok()) {
      return false;
    }

    bool isInvalidMessageInBag = false;

    // Update time registers.
    if ((ros::Time::now() - stampLastIteration) >= ros::Duration(1.0)) {
      stampLastIteration = ros::Time::now();
      wallStampLastIteration = ros::WallTime::now();
    }

    if (messageInstance.getTopic() == "/gt_box/cpt7/odom") {
      nav_msgs::Odometry::ConstPtr message = messageInstance.instantiate<nav_msgs::Odometry>();
      if (message != nullptr) {

        tracker = message->header.stamp;

        // add to the queue
        odometryQuque_.push(*message);

      } else {
        isInvalidMessageInBag = true;
        ROS_WARN("Invalid message found in ROS bag.");
      }
    }

    if (messageInstance.getTopic() == "/gt_box/cpt7/inspva") {
      novatel_oem7_msgs::INSPVA::ConstPtr message = messageInstance.instantiate<novatel_oem7_msgs::INSPVA>();
      if (message != nullptr) {

        INSPVAQueue_.push(*message);


      } else {
        isInvalidMessageInBag = true;
        ROS_WARN("Invalid message found in ROS bag.");
      }
    }

    const ros::WallDuration processingWallDurationActual{ros::WallTime::now() - wallStampLastIteration};
    const ros::Duration processingDurationActual{tracker - stampLastIteration};
    const auto rosWallTimeRatio{processingDurationActual.toSec() / processingWallDurationActual.toSec()};
    // Print walltime ratio
    if (rosWallTimeRatio > 1.1) {
      ROS_WARN_STREAM("Walltime ratio is: " << rosWallTimeRatio);
    }

  }

  associateAndWriteOdometryMsgs();

  ROS_INFO("Finished running through the bag for Odom msgs.");
  const ros::Time bag_begin_time = view.getBeginTime();
  const ros::Time bag_end_time = view.getEndTime();
  m_EndTime = std::chrono::steady_clock::now();

  std::cout << "Rosbag processing finished. Rosbag duration: " << (bag_end_time - bag_begin_time).toSec() << " sec."
            << " Time elapsed for processing: " << elapsedSeconds() << " sec. \n \n";

  bag.close();
  return true;
}

bool RosbagRangeDataProcessorRos::processGNSSRosbag() {
  std::vector<std::string> GPStopics;

  GPStopics.push_back("/gt_box/cpt7/gps/fix");
  GPStopics.push_back("/gt_box/cpt7/gps/gps");

  // Create me a high resolution clock timer from std chrono
  // Start the timer.
  m_StartTime = std::chrono::steady_clock::now();

  // Open ROS bag.
  rosbag::Bag bag;
  try {
    bag.open(rosbagFullname_, rosbag::bagmode::Read);
  } catch (const rosbag::BagIOException& e) {
    ROS_ERROR_STREAM("Error opening ROS bag: '" << rosbagFullname_ << "'");
    return false;
  }
  ROS_INFO_STREAM("ROS bag '" << rosbagFullname_ << "' open.");

  if (!validateTopicsInRosbag(bag, GPStopics)) {
    bag.close();
    return false;
  }

  ROS_INFO_STREAM("\033[92m"
                  << " Here we go... "
                  << "\033[0m");
  const ros::WallTime first{ros::WallTime::now() + ros::WallDuration(1.0)};
  ros::WallTime::sleepUntil(first);

  // // The bag view we iterate over.
  rosbag::View view(bag, rosbag::TopicQuery(GPStopics));

  ros::Time stampLastIteration{view.getBeginTime()};
  ros::WallTime wallStampLastIteration{ros::WallTime::now()};
  const ros::WallTime wallStampStartSequentialRun{ros::WallTime::now()};

  for (const auto& messageInstance : view) {
    // If the node is shutdown, stop processing and do early return.
    if (!ros::ok()) {
      return false;
    }

    bool isInvalidMessageInBag = false;

    // Update time registers.
    if ((ros::Time::now() - stampLastIteration) >= ros::Duration(1.0)) {
      stampLastIteration = ros::Time::now();
      wallStampLastIteration = ros::WallTime::now();
    }

    if (messageInstance.getTopic() == "/gt_box/cpt7/gps/fix") {
      //"/gt_box/cpt7/corrimu"
      sensor_msgs::NavSatFix::ConstPtr message = messageInstance.instantiate<sensor_msgs::NavSatFix>();
      if (message != nullptr) {

        tracker = message->header.stamp;

        // add to the queue
        rosFixQueue_.push(*message);


      } else {
        isInvalidMessageInBag = true;
        ROS_WARN("Invalid message found in ROS bag.");
      }
    }

    if (messageInstance.getTopic() == "/gt_box/cpt7/gps/gps") {
      //"/gt_box/cpt7/gps/imu"
      gps_common::GPSFix::ConstPtr message = messageInstance.instantiate<gps_common::GPSFix>();
      if (message != nullptr) {
        // Re-write the message with the new timestamp.
        // sensor_msgs::Imu messageOut = *message;
        gpsCommonQuque_.push(*message);

        //

        
      } else {
        isInvalidMessageInBag = true;
        ROS_WARN("Invalid message found in ROS bag.");
      }
    }

    const ros::WallDuration processingWallDurationActual{ros::WallTime::now() - wallStampLastIteration};
    const ros::Duration processingDurationActual{tracker - stampLastIteration};
    const auto rosWallTimeRatio{processingDurationActual.toSec() / processingWallDurationActual.toSec()};
    // Print walltime ratio
    if (rosWallTimeRatio > 1.1) {
      ROS_WARN_STREAM("Walltime ratio is: " << rosWallTimeRatio);
    }

  }

  associateAndWriteGPSmsgs();

  ROS_INFO("Finished running through the bag for GPS msgs.");
  const ros::Time bag_begin_time = view.getBeginTime();
  const ros::Time bag_end_time = view.getEndTime();
  m_EndTime = std::chrono::steady_clock::now();

  std::cout << "Rosbag processing finished. Rosbag duration: " << (bag_end_time - bag_begin_time).toSec() << " sec."
            << " Time elapsed for processing: " << elapsedSeconds() << " sec. \n \n";

  bag.close();
  return true;
}

bool RosbagRangeDataProcessorRos::associateAndWriteOdometryMsgs() {
  
  // If the queue is empty, return false.
  if (odometryQuque_.empty() || INSPVAQueue_.empty()) {
    ROS_WARN("Queue empty for GPS.");
    return false;
  }

  // if the queue sizes are not equal, return false. Write warning.
  if (odometryQuque_.size() != INSPVAQueue_.size()) {
    ROS_WARN("IMU Queue sizes are not equal. Will associate messages.");
    // Write the number of msgs
    ROS_WARN_STREAM("odometryQuque_ size: " << odometryQuque_.size());
    ROS_WARN_STREAM("INSPVAQueue_ size: " << INSPVAQueue_.size());
    // return false;
  }
  
  // Iterate through the queues and associate the messages.
  while (!odometryQuque_.empty() && !INSPVAQueue_.empty()) {

    // These are ordered.
    nav_msgs::Odometry odomMsg = odometryQuque_.front();
    novatel_oem7_msgs::INSPVA inspvaMsg = INSPVAQueue_.front();

    // double expectedmillisecondDiff = 20;

    uint64_t inspvaMsgtoNSEC = inspvaMsg.header.stamp.toNSec();
    uint64_t odomMsgtoNSEC = odomMsg.header.stamp.toNSec();

    // Calculate the difference between the last msgs and the current msgs header timestamps.
    
    ////////////////////////////////////////////////////////////////////////////
    std::chrono::time_point<std::chrono::steady_clock, std::chrono::nanoseconds> receivedTimeinspvaMsg(std::chrono::nanoseconds{inspvaMsgtoNSEC});
    auto receivedTimeinspvaMsgMilliseconds = std::chrono::time_point_cast<std::chrono::milliseconds>(receivedTimeinspvaMsg).time_since_epoch();

    std::chrono::time_point<std::chrono::steady_clock, std::chrono::nanoseconds> receivedTimeodomMsg(std::chrono::nanoseconds{odomMsgtoNSEC});
    auto receivedTimeodomMsgMilliseconds = std::chrono::time_point_cast<std::chrono::milliseconds>(receivedTimeodomMsg).time_since_epoch();

    if ( std::fabs( receivedTimeinspvaMsgMilliseconds.count() - receivedTimeodomMsgMilliseconds.count()) > 8) {
      // ROS_WARN("Time difference between the two messages is too large. Checking association.");
      // The time difference is this
      std::cout << std::setprecision(15) << "TimeDiff: " << std::fabs( receivedTimeinspvaMsgMilliseconds.count() - receivedTimeodomMsgMilliseconds.count()) << " ms." << std::endl;
      // std::cout << std::setprecision(15) << "inspvaMsg.header.stamp: " << inspvaMsg.header.stamp << std::endl;

      ROS_WARN_STREAM("receivedTimeinspvaMsgMilliseconds: " << receivedTimeinspvaMsgMilliseconds.count());
      ROS_WARN_STREAM("receivedTimeodomMsgMilliseconds: " << receivedTimeodomMsgMilliseconds.count());

      if (receivedTimeodomMsgMilliseconds.count() > receivedTimeinspvaMsgMilliseconds.count()) {

        // Odom msg is ahead of inspva msg. We need to pop the inspva msg.
        // INSPVAQueue_.pop();
        if ( ((odomMsgtoNSEC - lastOdometryTime_ / 1000000.0) > 30) ) {
          std::cout << "Odom msg is ahead of inspva msg. We need to pop the inspva msg. Time Diff: " << std::setprecision(15)<< (odomMsgtoNSEC - lastOdometryTime_ / 1000000.0) << std::endl;
          skipOdometry_ = true;
        }
      }

      if (receivedTimeinspvaMsgMilliseconds.count() > receivedTimeodomMsgMilliseconds.count()) {


        if ( ((inspvaMsgtoNSEC - lastInspvaTime_ / 1000000.0) > 30) ) {
          // INSPVA msg is ahead of odom msg. We need to pop the odom msg.
          std::cout << "inspva msg is ahead of odom msg. We need to pop the odom msg." << std::endl;


          skipInspva_ = true;
        }
      }

      // return false;
    }
    ////////////////////////////////////////////////////////////////////////////
        
    lastOdometryTime_ = odomMsgtoNSEC;
    lastInspvaTime_ = inspvaMsgtoNSEC;
    const int64_t totalMsec= GPSTimeToMsec(inspvaMsg.nov_header);
    uint64_t gpsTimeMilli = gps2UtcWithMiilli(totalMsec);

    std::chrono::time_point<std::chrono::steady_clock, std::chrono::milliseconds> timePoint(std::chrono::milliseconds{gpsTimeMilli});
    auto seconds = std::chrono::time_point_cast<std::chrono::seconds>(timePoint).time_since_epoch();
    auto nanoseconds = std::chrono::time_point_cast<std::chrono::nanoseconds>(timePoint).time_since_epoch() - seconds;

    ros::Time ros_utc_time;
    ros_utc_time.sec = seconds.count();
    ros_utc_time.nsec = nanoseconds.count();

    odomMsg.header.stamp = ros_utc_time;
    inspvaMsg.header.stamp = ros_utc_time;

    // Pop the front of the queues.
    if (skipOdometry_)
    {
      //
      
    }else{
      outBag.write("/gt_box/cpt7/odom", ros_utc_time, odomMsg);
      odometryQuque_.pop();
    }

    if (skipInspva_)
    {
      
      }else{
      outBag.write("/gt_box/cpt7/inspva", ros_utc_time, inspvaMsg);

      INSPVAQueue_.pop();
    }

    skipOdometry_ = false;
    skipInspva_ = false;

  }

  return true;
}

bool RosbagRangeDataProcessorRos::associateAndWriteGPSmsgs() {
  
  // If the queue is empty, return false.
  if (rosFixQueue_.empty() || gpsCommonQuque_.empty()) {
    ROS_WARN("Queue empty for GPS.");
    return false;
  }

  // if the queue sizes are not equal, return false. Write warning.
  if (rosFixQueue_.size() != gpsCommonQuque_.size()) {
    ROS_WARN("IMU Queue sizes are not equal. Will associate messages.");
    // Write the number of msgs
    ROS_WARN_STREAM("rosFixQueue_ size: " << rosFixQueue_.size());
    ROS_WARN_STREAM("gpsCommonQuque_ size: " << gpsCommonQuque_.size());
    // return false;
  }
  
  // Iterate through the queues and associate the messages.
  while (!rosFixQueue_.empty() && !gpsCommonQuque_.empty()) {

    // These are ordered.
    sensor_msgs::NavSatFix navsatFixMsg = rosFixQueue_.front();
    gps_common::GPSFix gpsCommonMsg = gpsCommonQuque_.front();

    uint64_t gpsCommonMsgtoNSEC = gpsCommonMsg.header.stamp.toNSec();
    uint64_t navsatFixMsgtoNSEC = navsatFixMsg.header.stamp.toNSec();

    std::chrono::time_point<std::chrono::steady_clock, std::chrono::nanoseconds> receivedTimegpsCommonMsg(std::chrono::nanoseconds{gpsCommonMsgtoNSEC});
    auto receivedTimegpsCommonMsgMilliseconds = std::chrono::time_point_cast<std::chrono::milliseconds>(receivedTimegpsCommonMsg).time_since_epoch();

    std::chrono::time_point<std::chrono::steady_clock, std::chrono::nanoseconds> receivedTimenavsatFixMsg(std::chrono::nanoseconds{navsatFixMsgtoNSEC});
    auto receivedTimenavsatFixMsgMilliseconds = std::chrono::time_point_cast<std::chrono::milliseconds>(receivedTimenavsatFixMsg).time_since_epoch();

    if ( std::fabs( receivedTimegpsCommonMsgMilliseconds.count() - receivedTimenavsatFixMsgMilliseconds.count()) > 10) {
      ROS_WARN("Time difference between the two messages is too large. Cannot associate messages.");
      // The time difference is this
      std::cout << std::setprecision(15) << "Diff: " << std::fabs( receivedTimegpsCommonMsgMilliseconds.count() - receivedTimenavsatFixMsgMilliseconds.count()) << std::endl;

      ROS_WARN_STREAM("receivedTimegpsCommonMsgMilliseconds: " << receivedTimegpsCommonMsgMilliseconds.count());
      ROS_WARN_STREAM("receivedTimenavsatFixMsgMilliseconds: " << receivedTimenavsatFixMsgMilliseconds.count());
      // return false;
    }

    // Differently gps_commoin provides gps time as float64 seconds.
    double gpsTimeAsMilliSeconds = gpsCommonMsg.time * 1000.0;
    double gpsTimeMilli = gps2UtcWithMilliDouble(gpsTimeAsMilliSeconds);

    // Make it nanoseconds with uint64_t
    uint64_t gpsTimeMilliNSEC = gpsTimeMilli * 1000000ULL;

    std::chrono::time_point<std::chrono::steady_clock, std::chrono::nanoseconds> timePoint(std::chrono::nanoseconds{gpsTimeMilliNSEC});
    auto seconds = std::chrono::time_point_cast<std::chrono::seconds>(timePoint).time_since_epoch();
    auto nanoseconds = std::chrono::time_point_cast<std::chrono::nanoseconds>(timePoint).time_since_epoch() - seconds;

    ros::Time ros_utc_time;
    ros_utc_time.sec = seconds.count();
    ros_utc_time.nsec = nanoseconds.count();

    navsatFixMsg.header.stamp = ros_utc_time;
    gpsCommonMsg.header.stamp = ros_utc_time;
    outBag.write("/gt_box/cpt7/gps/fix", ros_utc_time, navsatFixMsg);
    outBag.write("/gt_box/cpt7/gps/gps", ros_utc_time, gpsCommonMsg);

    // Pop the front of the queues.
    rosFixQueue_.pop();
    gpsCommonQuque_.pop();
  }

  return true;
}

bool RosbagRangeDataProcessorRos::associateAndWriteIMUmsgs() {
  
  // If the queue is empty, return false.
  if (corrIMUquque_.empty() || rosIMUQueue_.empty()) {
    ROS_WARN("Queue empty for IMU.");
    return false;
  }

  // if the queue sizes are not equal, return false. Write warning.
  if (corrIMUquque_.size() != rosIMUQueue_.size()) {
    ROS_WARN("IMU Queue sizes are not equal. Cannot associate messages.");
    // Write the number of msgs
    ROS_WARN_STREAM("corrIMUquque_ size: " << corrIMUquque_.size());
    ROS_WARN_STREAM("rosIMUQueue_ size: " << rosIMUQueue_.size());
    // return false;
  }
  
  // Iterate through the queues and associate the messages.
  while (!corrIMUquque_.empty() || !rosIMUQueue_.empty()) {

    novatel_oem7_msgs::CORRIMU corrIMU = corrIMUquque_.front();
    sensor_msgs::Imu rosIMU = rosIMUQueue_.front();

    // auto weekNum = message->nov_header.gps_week_number;
    // auto milliSeconds = message->nov_header.gps_week_milliseconds;
    // // ROS_INFO_STREAM("Original GPS time: ");
    // std::cout << std::setprecision(10) << "Week: " << weekNum << std::endl;
    // std::cout << std::setprecision(10) << "milliSeconds in Week: " << milliSeconds << std::endl;
    // std::cout << std::setprecision(10) << "Seconds in Week: " << (double)milliSeconds / 1000.0 << std::endl;

    ///////////////////////////////////////////////////////////////////
    // GPS is ahead of UTC by 18 seconds, due to leap seconds.
    ///////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////
    const int64_t totalMsec= GPSTimeToMsec(corrIMU.nov_header);
    uint64_t gpsTimeMilli = gps2UtcWithMiilli(totalMsec);

    std::chrono::time_point<std::chrono::steady_clock, std::chrono::milliseconds> timePoint(std::chrono::milliseconds{gpsTimeMilli});
    auto seconds = std::chrono::time_point_cast<std::chrono::seconds>(timePoint).time_since_epoch();
    auto nanoseconds = std::chrono::time_point_cast<std::chrono::nanoseconds>(timePoint).time_since_epoch() - seconds;

    ros::Time ros_utc_time;
    ros_utc_time.sec = seconds.count();
    ros_utc_time.nsec = nanoseconds.count();


    uint64_t rosIMUtoNSEC = rosIMU.header.stamp.toNSec();
    uint64_t corrIMUtoNSEC = corrIMU.header.stamp.toNSec();

    // // Print the uint64_t values. With std::cout, we can print the values. With high precision.
    // std::cout << std::setprecision(15) << "rosIMUtoNSEC: " << rosIMUtoNSEC << std::endl;
    // std::cout << std::setprecision(15) << "corrIMUtoNSEC: " << corrIMUtoNSEC << std::endl;
    

    std::chrono::time_point<std::chrono::steady_clock, std::chrono::nanoseconds> receivedTimeRosImu(std::chrono::nanoseconds{rosIMUtoNSEC});
    auto receivedTimeRosImuMilliseconds = std::chrono::time_point_cast<std::chrono::milliseconds>(receivedTimeRosImu).time_since_epoch();

    std::chrono::time_point<std::chrono::steady_clock, std::chrono::nanoseconds> receivedTimeCorrIMU(std::chrono::nanoseconds{corrIMUtoNSEC});
    auto receivedTimeCorrIMUMilliseconds = std::chrono::time_point_cast<std::chrono::milliseconds>(receivedTimeCorrIMU).time_since_epoch();


    if ( std::fabs( receivedTimeRosImuMilliseconds.count() - receivedTimeCorrIMUMilliseconds.count()) > 5) {
      ROS_WARN("Time difference between the two messages is too large. Cannot associate messages.");
      // The time difference is this
      std::cout << std::setprecision(15) << "Diff: " << std::fabs( receivedTimeRosImuMilliseconds.count() - receivedTimeCorrIMUMilliseconds.count()) << std::endl;

      ROS_WARN_STREAM("receivedTimeRosImuMilliseconds: " << receivedTimeRosImuMilliseconds.count());
      ROS_WARN_STREAM("receivedTimeCorrIMUMilliseconds: " << receivedTimeCorrIMUMilliseconds.count());
      // return false;
    }

    corrIMU.header.stamp = ros_utc_time;
    rosIMU.header.stamp = ros_utc_time;
    outBag.write("/gt_box/cpt7/corrimu", ros_utc_time, corrIMU);
    outBag.write("/gt_box/cpt7/gps/imu", ros_utc_time, rosIMU);

    // Pop the front of the queues.
    corrIMUquque_.pop();
    rosIMUQueue_.pop();
  }

  return true;
}

bool RosbagRangeDataProcessorRos::processIMURosbag() {
  std::vector<std::string> IMUtopics;

  IMUtopics.push_back("/gt_box/cpt7/corrimu");
  IMUtopics.push_back("/gt_box/cpt7/gps/imu");

  // Create me a high resolution clock timer from std chrono
  // Start the timer.
  m_StartTime = std::chrono::steady_clock::now();

  // Open ROS bag.
  rosbag::Bag bag;
  try {
    bag.open(rosbagFullname_, rosbag::bagmode::Read);
  } catch (const rosbag::BagIOException& e) {
    ROS_ERROR_STREAM("Error opening ROS bag: '" << rosbagFullname_ << "'");
    return false;
  }
  ROS_INFO_STREAM("ROS bag '" << rosbagFullname_ << "' open.");

  if (!validateTopicsInRosbag(bag, IMUtopics)) {
    bag.close();
    return false;
  }

  ROS_INFO_STREAM("\033[92m"
                  << " Here we go... "
                  << "\033[0m");
  const ros::WallTime first{ros::WallTime::now() + ros::WallDuration(1.0)};
  ros::WallTime::sleepUntil(first);

  // // The bag view we iterate over.
  rosbag::View view(bag, rosbag::TopicQuery(IMUtopics));

  ros::Time stampLastIteration{view.getBeginTime()};
  ros::WallTime wallStampLastIteration{ros::WallTime::now()};
  const ros::WallTime wallStampStartSequentialRun{ros::WallTime::now()};

  for (const auto& messageInstance : view) {
    // If the node is shutdown, stop processing and do early return.
    if (!ros::ok()) {
      return false;
    }

    bool isInvalidMessageInBag = false;

    // Update time registers.
    if ((ros::Time::now() - stampLastIteration) >= ros::Duration(1.0)) {
      stampLastIteration = ros::Time::now();
      wallStampLastIteration = ros::WallTime::now();
    }

    // Tf static.
    if (messageInstance.getTopic() == "/gt_box/cpt7/corrimu") {
      //"/gt_box/cpt7/corrimu"
      novatel_oem7_msgs::CORRIMU::ConstPtr message = messageInstance.instantiate<novatel_oem7_msgs::CORRIMU>();
      if (message != nullptr) {

        tracker = message->header.stamp;

        // add to the queue
        corrIMUquque_.push(*message);


      } else {
        isInvalidMessageInBag = true;
        ROS_WARN("Invalid message found in ROS bag.");
      }
    }

    if (messageInstance.getTopic() == "/gt_box/cpt7/gps/imu") {
      //"/gt_box/cpt7/gps/imu"
      sensor_msgs::Imu::ConstPtr message = messageInstance.instantiate<sensor_msgs::Imu>();
      if (message != nullptr) {
        // Re-write the message with the new timestamp.
        // sensor_msgs::Imu messageOut = *message;
        rosIMUQueue_.push(*message);

        
      } else {
        isInvalidMessageInBag = true;
        ROS_WARN("Invalid message found in ROS bag.");
      }
    }

    const ros::WallDuration processingWallDurationActual{ros::WallTime::now() - wallStampLastIteration};
    const ros::Duration processingDurationActual{tracker - stampLastIteration};
    const auto rosWallTimeRatio{processingDurationActual.toSec() / processingWallDurationActual.toSec()};
    // Print walltime ratio
    if (rosWallTimeRatio > 1.1) {
      ROS_WARN_STREAM("Walltime ratio is: " << rosWallTimeRatio);
    }

  }

  associateAndWriteIMUmsgs();

  ROS_INFO("Finished running through the bag for IMU msgs.");
  const ros::Time bag_begin_time = view.getBeginTime();
  const ros::Time bag_end_time = view.getEndTime();
  m_EndTime = std::chrono::steady_clock::now();

  std::cout << "Rosbag processing finished. Rosbag duration: " << (bag_end_time - bag_begin_time).toSec() << " sec."
            << " Time elapsed for processing: " << elapsedSeconds() << " sec. \n \n";

  bag.close();
  return true;
}

}
