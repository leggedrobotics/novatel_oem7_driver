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

  processRosbag();

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


bool RosbagRangeDataProcessorRos::processRosbag() {
  std::vector<std::string> topics;

  // // Currently investigating if we really need clock or can we live without it.
  topics.push_back("/gt_box/cpt7/corrimu");
  // topics.push_back(cloudTopic_);
  // topics.push_back(tfStaticTopic_);

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

  if (!validateTopicsInRosbag(bag, topics)) {
    bag.close();
    return false;
  }

  // We are ready to initiatate the outbag
  outBag.open(rosbagOutFullname_, rosbag::bagmode::Write);

  ROS_INFO_STREAM("\033[92m"
                  << " Here wo go... "
                  << "\033[0m");
  const ros::WallTime first{ros::WallTime::now() + ros::WallDuration(2.0)};
  ros::WallTime::sleepUntil(first);

  // // The bag view we iterate over.
  rosbag::View view(bag, rosbag::TopicQuery(topics));

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

        auto weekNum = message->nov_header.gps_week_number;
        auto milliSeconds = message->nov_header.gps_week_milliseconds;

        // ROS_INFO_STREAM("Original GPS time: ");
        std::cout << std::setprecision(10) << "Week: " << weekNum << std::endl;
        std::cout << std::setprecision(10) << "milliSeconds in Week: " << milliSeconds << std::endl;
        std::cout << std::setprecision(10) << "Seconds in Week: " << (double)milliSeconds / 1000.0 << std::endl;

        // double rosCompatibleTime = gpsToRosTime(weekNum, milliSeconds);
        // // ROS_INFO_STREAM("ROS compatible time: ");
        // // std::cout << std::setprecision(10) << rosCompatibleTime << std::endl;

        // // double difference = rosCompatibleTime - rosCompatibleTime_prev;
        // // rosCompatibleTime_prev = rosCompatibleTime;
        // // ROS_INFO_STREAM("Difference: ");
        // // std::cout << std::setprecision(10) << difference* 1e3 << std::endl;

        // // Extract the integer part of the seconds
        // uint32_t sec = static_cast<uint32_t>(rosCompatibleTime);
        
        // // Extract the fractional part and convert it to nanoseconds
        // uint32_t nsec = static_cast<uint32_t>((rosCompatibleTime - sec) * 1e9);
        
        // // Construct the ros::Time object
        // ros::Time ros_time;
        // ros_time.sec = sec;
        // ros_time.nsec = nsec;


        //Print tracker
        // ROS_INFO_STREAM("Tracker: " << tracker);
        //Use std cout and std preciison to write the time in extreme precision
        // std::cout << std::setprecision(20) << rosCompatibleTime << std::endl;

        int64_t totalMsec= GPSTimeToMsec(message->nov_header);
        // ROS_INFO_STREAM("Calculated Total Msec: ");
        // std::cout << std::setprecision(10) << "totalMsec: " << totalMsec << std::endl;
        // std::cout << std::setprecision(10) << "milliSeconds in Week: " << milliSeconds << std::endl;


        // int gps_week = getGPSweek(totalMsec);
        // double gps_time = getGPSsecond(totalMsec);
        // ROS_INFO_STREAM("Recovered Week: ");
        // std::cout << std::setprecision(10) << "Recovered Week: " << gps_week << std::endl;
        // std::cout << std::setprecision(10) << "Seconds in Week: " << gps_time << std::endl;

        // uint64_t hostTime = getCurrentTime();
        // THIS IS IN MILLISECONDS
        // uint64_t gpsTime = gps2Utc(weekNum, milliSeconds* (uint64_t)1000.0);

        ///////////////////////////////////////////////////////////////////
        // GPS is ahead of UTC by 18 seconds, due to leap seconds.
        ///////////////////////////////////////////////////////////////////
        ///////////////////////////////////////////////////////////////////

        uint64_t gpsTimeMilli = gps2UtcWithMiilli(totalMsec);
        // uint64_t gpsTimeNanoSeconds = gpsTimeMilli * (uint64_t)1000000.0;

        std::chrono::time_point<std::chrono::steady_clock, std::chrono::milliseconds> timePoint(std::chrono::milliseconds{gpsTimeMilli});
        auto seconds = std::chrono::time_point_cast<std::chrono::seconds>(timePoint).time_since_epoch();
        auto nanoseconds = std::chrono::time_point_cast<std::chrono::nanoseconds>(timePoint).time_since_epoch() - seconds;
        
        // std::cout << "Seconds: " << seconds.count() << "s\n";
        // std::cout << "Nanoseconds: " << nanoseconds.count() << "ms\n";

        // auto dv = std::div(gpsTime * 1LL, 1000LL);
        // double remaning = dv.rem;
        // double nanoRemoved = gpsTime - remaning;
        // ROS_INFO_STREAM("gpsTimeNanoSeconds: ");
        // std::cout << std::setprecision(15) << gpsTimeNanoSeconds << std::endl;

        // double pctimediffMSec = fabs(gpsTime / 1000.0 - hostTime / 1000.0);
        // ROS_INFO_STREAM("pctimediffMSec");
        // std::cout << std::setprecision(10) << pctimediffMSec << std::endl;

        // ROS_INFO_STREAM("Converted UTC time: ");
        // double gpsTimeSecondsDouble_raw = static_cast<double>(gpsTime);
        // double gpsTimeSecondsDouble = gpsTimeSecondsDouble_raw / 1e6;
        // std::cout << std::setprecision(10) << gpsTimeSecondsDouble << std::endl;

        ros::Time ros_utc_time;
        // uint64_t sec_utc = static_cast<uint64_t>(gpsTimeSecondsDouble);
        ros_utc_time.sec = seconds.count();
        // double nsec_utc = static_cast<uint64_t>((gpsTimeSecondsDouble - sec_utc) * 1e9);
        ros_utc_time.nsec = nanoseconds.count();

        ROS_INFO_STREAM("Calculated ROS time: ");
        std::cout << std::setprecision(15) << ros_utc_time << std::endl;

        ros::Duration diff = ros_utc_time - message->header.stamp;

        ROS_INFO_STREAM("Received ROS time: ");
        std::cout << std::setprecision(15) << message->header.stamp << std::endl;

        ROS_INFO_STREAM("ROS time diff: ");
        std::cout << std::setprecision(15) << diff << std::endl;

        novatel_oem7_msgs::CORRIMU messageOut = *message;

        messageOut.header.stamp = ros_utc_time;
        outBag.write("/gt_box/cpt7/corrimu", ros_utc_time, messageOut);



        // int64_t diff = totalMsec - totalMsec_prev;
        // totalMsec_prev = totalMsec;
        
        // ROS_INFO_STREAM("Total milliseconds: ");
        // std::cout << std::setprecision(10) << diff << std::endl;

        // Print
        // ROS_INFO_STREAM("Week number: " << weekNum << " Milliseconds: " << milliSeconds);





      } else {
        isInvalidMessageInBag = true;
        ROS_WARN("Invalid message found in ROS bag.");
      }
    }

    // // Tf.
    // if (messageInstance.getTopic() == tfTopic_) {
    //   tf2_msgs::TFMessage::ConstPtr message = messageInstance.instantiate<tf2_msgs::TFMessage>();
    //   if (message != nullptr) {
    //     transformBroadcaster_.sendTransform(message->transforms);

    //   } else {
    //     isInvalidMessageInBag = true;
    //     ROS_WARN("Invalid message found in ROS bag.");
    //   }
    // }


    const ros::WallDuration processingWallDurationActual{ros::WallTime::now() - wallStampLastIteration};
    const ros::Duration processingDurationActual{tracker - stampLastIteration};
    const auto rosWallTimeRatio{processingDurationActual.toSec() / processingWallDurationActual.toSec()};
    // Print walltime ratio
    if (rosWallTimeRatio > 1.1) {
      ROS_WARN_STREAM("Walltime ratio is: " << rosWallTimeRatio);
    }

  }

  ROS_INFO("Finished running through the bag.");
  const ros::Time bag_begin_time = view.getBeginTime();
  const ros::Time bag_end_time = view.getEndTime();
  m_EndTime = std::chrono::steady_clock::now();

  std::cout << "Rosbag processing finished. Rosbag duration: " << (bag_end_time - bag_begin_time).toSec()
            << " Time elapsed for processing: " << elapsedSeconds() << " sec. \n \n";

  bag.close();
  outBag.close();

  return true;
}

}
