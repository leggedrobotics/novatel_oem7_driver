#include "novatel_oem7_driver/replayer.hpp"
#include <ros/ros.h>
#include <cstdlib>
#include <rosbag/view.h>
#include <rosbag/bag.h> 
#include <csignal>
#include <tf/tf.h>
#include <tf2_msgs/TFMessage.h>
#include <filesystem>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include "novatel_oem7_driver/replayer_utils.hpp"

namespace novatel_oem7_driver {

RosbagRangeDataProcessorRos::RosbagRangeDataProcessorRos(ros::NodeHandlePtr nh) : nh_(nh) {}

void RosbagRangeDataProcessorRos::initialize() {
  initCommonRosStuff();

  // Remove exsisting rosbag
  rosbagFullname_ = inputRosbagBasePath_ + inputRosbagName_;
  if (!std::filesystem::exists(rosbagFullname_))
			{
        ROS_ERROR("Input Rosbag does not exist. Exiting.");
        return;
      }
  // rosbagOutFullname_ = inputRosbagBasePath_ + outputRosbagName_;
  std::remove(rosbagOutFullname_.c_str());

  if (!createOutputDirectory()) {
    ROS_ERROR("Failed to create output directory. Exiting.");
    return;
  }

  // Run the processing.
  run();

  return;
}

void RosbagRangeDataProcessorRos::run() {

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

  ROS_INFO_STREAM("\033[92m"
                << " SUCCESSFULLY COMPLETED REPLAYING. TERMINATING MYSELF. "
                << "\033[0m");

  const ros::WallTime first{ros::WallTime::now() + ros::WallDuration(2)};
  ros::WallTime::sleepUntil(first);

  return;
}

bool RosbagRangeDataProcessorRos::createOutputDirectory() {

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
  // outputRosbagName_ = nh_->param<std::string>("outputBagName", "");
  outputRosbagName_ = inputRosbagName_;
  
  outputRosbagName_.erase(outputRosbagName_.end() - 4, outputRosbagName_.end());
  rosbagOutFullname_= inputRosbagBasePath_ + outputRosbagName_+ "_post_processed.bag";

  // Printout the parameters.
  ROS_INFO_STREAM("inputRosbagBasePath_: " << inputRosbagBasePath_);
  ROS_INFO_STREAM("inputRosbagName_: " << inputRosbagName_);
  ROS_INFO_STREAM("rosbagOutFullname_: " << rosbagOutFullname_);
  
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
  startTime_ = std::chrono::steady_clock::now();

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
                  << " Here we go for Odometry Rosbag... "
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

    // const ros::WallDuration processingWallDurationActual{ros::WallTime::now() - wallStampLastIteration};
    // const ros::Duration processingDurationActual{tracker - stampLastIteration};
    // const auto rosWallTimeRatio{processingDurationActual.toSec() / processingWallDurationActual.toSec()};
    // // Print walltime ratio
    // if (rosWallTimeRatio > 1.1) {
    //   ROS_WARN_STREAM("Walltime ratio is: " << rosWallTimeRatio);
    // }

  }

  associateAndWriteOdometryMsgs();

  ROS_INFO("Finished running through the bag for Odom msgs.");
  const ros::Time bag_begin_time = view.getBeginTime();
  const ros::Time bag_end_time = view.getEndTime();
  endTime_ = std::chrono::steady_clock::now();

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
  startTime_ = std::chrono::steady_clock::now();

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
                  << " Here we go for GPS Rosbag "
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
      gps_common::GPSFix::ConstPtr message = messageInstance.instantiate<gps_common::GPSFix>();
      if (message != nullptr) {
        gpsCommonQuque_.push(*message);
        
      } else {
        isInvalidMessageInBag = true;
        ROS_WARN("Invalid message found in ROS bag.");
      }
    }

    // const ros::WallDuration processingWallDurationActual{ros::WallTime::now() - wallStampLastIteration};
    // const ros::Duration processingDurationActual{tracker - stampLastIteration};
    // const auto rosWallTimeRatio{processingDurationActual.toSec() / processingWallDurationActual.toSec()};
    // // Print walltime ratio
    // if (rosWallTimeRatio > 1.1) {
    //   ROS_WARN_STREAM("Walltime ratio is: " << rosWallTimeRatio);
    // }

  }

  associateAndWriteGPSmsgs();

  ROS_INFO("Finished running through the bag for GPS msgs.");
  const ros::Time bag_begin_time = view.getBeginTime();
  const ros::Time bag_end_time = view.getEndTime();
  endTime_ = std::chrono::steady_clock::now();

  std::cout << "Rosbag processing finished. Rosbag duration: " << (bag_end_time - bag_begin_time).toSec() << " sec."
            << " Time elapsed for processing: " << elapsedSeconds() << " sec. \n \n";

  bag.close();
  return true;
}

bool RosbagRangeDataProcessorRos::associateAndWriteOdometryMsgs() {
  
  uint64_t allowedReceivedTimeDifference = 10; // 10 ms
  uint64_t allowedTimeDifferenceBetweenConsecutiveMsgs = 30; // 30 ms

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

    uint64_t seqOdomMsg = odomMsg.header.seq;//   +1;
    uint64_t seqINSVPAMsg = inspvaMsg.header.seq;

    while(seqOdomMsg != seqINSVPAMsg) {

      ROS_WARN_STREAM("Seq Numbers are not equal. seqOdomMsg: " << seqOdomMsg << " AND seqINSVPAMsg: " << seqINSVPAMsg);

      if (seqOdomMsg < prevseqOdomMsg_)
      {
        ROS_ERROR_STREAM("ROS odomMsg Seq number going back. Previous " << prevseqOdomMsg_ << " AND seqOdomMsg: " << seqOdomMsg);
        odometryQuque_.pop();
        odomMsg = odometryQuque_.front();
        seqOdomMsg = odomMsg.header.seq;
        continue;
      }
      
      if (seqINSVPAMsg < prevseqINSVPAMsg_)
      {
        ROS_ERROR_STREAM("ROS INSPVAQueue_ Seq number going back. Previous " << prevseqINSVPAMsg_ << " AND seqINSVPAMsg: " << seqINSVPAMsg);
        INSPVAQueue_.pop();
        inspvaMsg = INSPVAQueue_.front();
        seqINSVPAMsg = inspvaMsg.header.seq;
        continue;
      }

      if (seqOdomMsg > seqINSVPAMsg) {
        // ROS_WARN_STREAM("ROS IMU Seq number is bigger. seqOdomMsg " << seqOdomMsg << " AND seqINSVPAMsg: " << seqINSVPAMsg);
        INSPVAQueue_.pop();
        inspvaMsg = INSPVAQueue_.front();
        seqINSVPAMsg = inspvaMsg.header.seq;
        continue;
      } else {
        // ROS_WARN("CORR IMU is ahead of ROS IMU. Will pop the ROS IMU.");
        // ROS_WARN_STREAM("seqINSVPAMsg number is bigger. seqINSVPAMsg " << seqINSVPAMsg << " AND seqOdomMsg: " << seqOdomMsg);
        odometryQuque_.pop();
        odomMsg = odometryQuque_.front();
        seqOdomMsg = odomMsg.header.seq;
        continue;
      }
    }


    ros::Duration receivedRosTimeDifference = odomMsg.header.stamp - inspvaMsg.header.stamp;

    uint64_t inspvaMsgtoNSEC = inspvaMsg.header.stamp.toNSec();
    uint64_t odomMsgtoNSEC = odomMsg.header.stamp.toNSec();

    // Calculate the difference between the last msgs and the current msgs header timestamps.
    ////////////////////////////////////////////////////////////////////////////
    std::chrono::time_point<std::chrono::steady_clock, std::chrono::nanoseconds> receivedTimeinspvaMsg(std::chrono::nanoseconds{inspvaMsgtoNSEC});
    auto receivedTimeinspvaMsgMilliseconds = std::chrono::time_point_cast<std::chrono::milliseconds>(receivedTimeinspvaMsg).time_since_epoch();

    std::chrono::time_point<std::chrono::steady_clock, std::chrono::nanoseconds> receivedTimeodomMsg(std::chrono::nanoseconds{odomMsgtoNSEC});
    auto receivedTimeodomMsgMilliseconds = std::chrono::time_point_cast<std::chrono::milliseconds>(receivedTimeodomMsg).time_since_epoch();

    // Magic Number fix TT
    if ( std::fabs(receivedRosTimeDifference.toSec()*1000) > allowedReceivedTimeDifference) {
      ROS_WARN("Time difference between the two messages is too large. Checking association.");
      std::cout << std::setprecision(15) << "TimeDiff: " << std::fabs(receivedRosTimeDifference.toSec()*1000) << " ms." << std::endl;
      ROS_WARN_STREAM("receivedTimeinspvaMsgMilliseconds: " << receivedTimeinspvaMsgMilliseconds.count());
      ROS_WARN_STREAM("receivedTimeodomMsgMilliseconds: " << receivedTimeodomMsgMilliseconds.count());

      if (receivedTimeodomMsgMilliseconds.count() > receivedTimeinspvaMsgMilliseconds.count()) {

        // Odom msg is ahead of inspva msg. We need to pop the inspva msg.
        // INSPVAQueue_.pop();
        // This means some msgs were skipped. (not recorded)
        ros::Duration differenceBetweenConsecutiveRosIMUMsgs = odomMsg.header.stamp - lastOdometryTime_;
        if ( ( differenceBetweenConsecutiveRosIMUMsgs.toSec()*1000  > allowedTimeDifferenceBetweenConsecutiveMsgs) ) {
          std::cout << "Odom msg is ahead of inspva msg. We need to pop the inspva msg. Time Diff: " << std::setprecision(15)<< differenceBetweenConsecutiveRosIMUMsgs.toSec()*1000 << std::endl;
          skipOdometry_ = true;
        }
      }

      if (receivedTimeinspvaMsgMilliseconds.count() > receivedTimeodomMsgMilliseconds.count()) {
        // Magic Number fix TT
        ros::Duration differenceBetweenConsecutiveRosCorrIMUMsgs = inspvaMsg.header.stamp - lastInspvaTime_;
        if ( (differenceBetweenConsecutiveRosCorrIMUMsgs.toSec()*1000 > allowedTimeDifferenceBetweenConsecutiveMsgs) ) {
          // INSPVA msg is ahead of odom msg. We need to pop the odom msg.
          std::cout << "inspva msg is ahead of odom msg. We need to pop the odom msg." << std::endl;

          skipInspva_ = true;
        }
      }
    }
    ////////////////////////////////////////////////////////////////////////////
        
    lastOdometryTime_ = odomMsg.header.stamp;
    lastInspvaTime_ = inspvaMsg.header.stamp;

    const int64_t totalMsec= GPSTimeToMsec(inspvaMsg.nov_header);
    uint64_t gpsTimeMilli = gps2UtcWithMiilli(totalMsec);

    std::chrono::time_point<std::chrono::steady_clock, std::chrono::milliseconds> timePoint(std::chrono::milliseconds{gpsTimeMilli});
    auto seconds = std::chrono::time_point_cast<std::chrono::seconds>(timePoint).time_since_epoch();
    auto nanoseconds = std::chrono::time_point_cast<std::chrono::nanoseconds>(timePoint).time_since_epoch() - seconds;

    ros::Time ros_utc_time;
    ros_utc_time.sec = seconds.count();
    ros_utc_time.nsec = nanoseconds.count();

    ros::Duration conversionDiff = ros_utc_time - inspvaMsg.header.stamp;
    
    if (conversionDiff.toSec() > 0.1)
    {
      ROS_ERROR_STREAM("BIG DIFFERENCE BETWEEN CONVERTED ROS TIME AND ORIGINAL RECEIVED TIME. INSVPA TIME CONVERSION MIGHT BE FAULTY : " << conversionDiff.toSec()*1000);
    }

    odomMsg.header.stamp = ros_utc_time;
    inspvaMsg.header.stamp = ros_utc_time;

    // Pop the front of the queues.
    if (!skipOdometry_)
    {

      // Convert the odomMsg to tf transform and write to the bag.
      geometry_msgs::TransformStamped transformStamped;
      transformStamped.header.stamp = ros_utc_time;
      transformStamped.header.frame_id = "box_base";
      transformStamped.child_frame_id = "cpt7_odom";

      tf2::Vector3 translation;
      translation.setX(odomMsg.pose.pose.position.x);
      translation.setY(odomMsg.pose.pose.position.y);
      translation.setZ(odomMsg.pose.pose.position.z);

      tf2::Quaternion quat;
      quat.setX(odomMsg.pose.pose.orientation.x);
      quat.setY(odomMsg.pose.pose.orientation.y);
      quat.setZ(odomMsg.pose.pose.orientation.z);
      quat.setW(odomMsg.pose.pose.orientation.w);

      transformStamped.transform.translation.x = translation.x();
      transformStamped.transform.translation.y = translation.y();
      transformStamped.transform.translation.z = translation.z();
      transformStamped.transform.rotation.x = quat.x();
      transformStamped.transform.rotation.y = quat.y();
      transformStamped.transform.rotation.z = quat.z();
      transformStamped.transform.rotation.w = quat.w();

      tf::Transform transform;
      tf::transformMsgToTF(transformStamped.transform, transform);
      geometry_msgs::Transform inverted_transform_msg;
      tf::transformTFToMsg(transform.inverse(), inverted_transform_msg);

      transformStamped.transform = inverted_transform_msg;

      tf2_msgs::TFMessage tfMsgs;
      tfMsgs.transforms.push_back(transformStamped);

      outBag.write("/tf", ros_utc_time, tfMsgs);
      outBag.write("/gt_box/cpt7/odom", ros_utc_time, odomMsg);
      odometryQuque_.pop();
    }

    if (!skipInspva_)
    {
      
      outBag.write("/gt_box/cpt7/inspva", ros_utc_time, inspvaMsg);
      INSPVAQueue_.pop();
    }

    skipOdometry_ = false;
    skipInspva_ = false;
  }

  return true;
}

bool RosbagRangeDataProcessorRos::associateAndWriteGPSmsgs() {

  // These msgs are at 50hz ~ 20ms between msgs.
  uint64_t allowedReceivedTimeDifference = 10; // 10 ms
  uint64_t allowedTimeDifferenceBetweenConsecutiveMsgs = 30; // 30 ms
  
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

    uint64_t seqNavsatFix = navsatFixMsg.header.seq;
    uint64_t seqGpsCommon = gpsCommonMsg.header.seq;

    while(seqNavsatFix != seqGpsCommon) {

      ROS_WARN_STREAM("Seq Numbers are not equal. seqNavsatFix: " << seqNavsatFix << " AND seqGpsCommon: " << seqGpsCommon);

      if (seqNavsatFix < prevSeqNavsatFix_)
      {
        ROS_ERROR_STREAM("ROS navsatFixMsg Seq number going back. Previous " << prevSeqNavsatFix_ << " AND seqNavsatFix: " << seqNavsatFix);
        rosFixQueue_.pop();
        navsatFixMsg = rosFixQueue_.front();
        seqNavsatFix = navsatFixMsg.header.seq;
        continue;
      }
      
      if (seqGpsCommon < prevSeqGpsCommon_)
      {
        ROS_ERROR_STREAM("ROS gpsCommonMsg Seq number going back. Previous " << prevSeqGpsCommon_ << " AND seqGpsCommon: " << seqGpsCommon);
        gpsCommonQuque_.pop();
        gpsCommonMsg = gpsCommonQuque_.front();
        seqGpsCommon = gpsCommonMsg.header.seq;
        continue;
      }

      if (seqNavsatFix > seqGpsCommon) {
        gpsCommonQuque_.pop();
        gpsCommonMsg = gpsCommonQuque_.front();
        seqGpsCommon = gpsCommonMsg.header.seq;
        continue;
      } else {
        rosFixQueue_.pop();
        navsatFixMsg = rosFixQueue_.front();
        seqNavsatFix = navsatFixMsg.header.seq;
        continue;
      }
    }

    ros::Duration receivedRosTimeDifference = navsatFixMsg.header.stamp - gpsCommonMsg.header.stamp;
    prevSeqNavsatFix_ = seqNavsatFix;
    prevSeqGpsCommon_ = seqGpsCommon;


    uint64_t gpsCommonMsgtoNSEC = gpsCommonMsg.header.stamp.toNSec();
    uint64_t navsatFixMsgtoNSEC = navsatFixMsg.header.stamp.toNSec();

    std::chrono::time_point<std::chrono::steady_clock, std::chrono::nanoseconds> receivedTimegpsCommonMsg(std::chrono::nanoseconds{gpsCommonMsgtoNSEC});
    auto receivedTimegpsCommonMsgMilliseconds = std::chrono::time_point_cast<std::chrono::milliseconds>(receivedTimegpsCommonMsg).time_since_epoch();

    std::chrono::time_point<std::chrono::steady_clock, std::chrono::nanoseconds> receivedTimenavsatFixMsg(std::chrono::nanoseconds{navsatFixMsgtoNSEC});
    auto receivedTimenavsatFixMsgMilliseconds = std::chrono::time_point_cast<std::chrono::milliseconds>(receivedTimenavsatFixMsg).time_since_epoch();

    if ( std::fabs(receivedRosTimeDifference.toSec()*1000) > allowedReceivedTimeDifference) {
      ROS_WARN("Time difference between the two messages is too large. Checking association.");
      // The time difference is this
      std::cout << std::setprecision(15) << "Diff: " << std::fabs(receivedRosTimeDifference.toSec()*1000) << std::endl;

      ROS_WARN_STREAM("receivedTimegpsCommonMsgMilliseconds: " << receivedTimegpsCommonMsgMilliseconds.count());
      ROS_WARN_STREAM("receivedTimenavsatFixMsgMilliseconds: " << receivedTimenavsatFixMsgMilliseconds.count());


      // If this happens, we know that the rosmsgs is ahead of corr imu and we need to hold the value of rosmsgs such that we can find a match of corrimu.
      if (receivedTimenavsatFixMsgMilliseconds.count() > receivedTimegpsCommonMsgMilliseconds.count()) {
        // This means some msgs were skipped. (not recorded)
        ros::Duration differenceBetweenConsecutiveGPSFixMsgs = navsatFixMsg.header.stamp - lastFixmsgsTime_;
        if ( differenceBetweenConsecutiveGPSFixMsgs.toSec()*1000 > allowedTimeDifferenceBetweenConsecutiveMsgs)  {
          std::cout << "NAVSATFIX IS AHEAD. Time Diff: " << std::setprecision(15)<< differenceBetweenConsecutiveGPSFixMsgs.toSec()*1000 << std::endl;
          skipGpsFix_ = true;
        }
      }

      if (receivedTimegpsCommonMsgMilliseconds.count() > receivedTimenavsatFixMsgMilliseconds.count()) {
        // Magic Number fix TT
        ros::Duration differenceBetweenConsecutiveGPSCOMMONMsgs = gpsCommonMsg.header.stamp - lastGPSCommonTime_;
        if ( differenceBetweenConsecutiveGPSCOMMONMsgs.toSec()*1000 > allowedTimeDifferenceBetweenConsecutiveMsgs) {
          // INSPVA msg is ahead of odom msg. We need to pop the odom msg.
          std::cout << "GPSCOMMON msg is ahead. Time Diff: " << std::setprecision(15)<< differenceBetweenConsecutiveGPSCOMMONMsgs.toSec()*1000 << std::endl;

          skipGPSCommon_ = true;
        }
      }

    }


    // Update time holders
    lastFixmsgsTime_ = navsatFixMsg.header.stamp;
    lastGPSCommonTime_ = gpsCommonMsg.header.stamp;


    // Differently gps_common provides gps time as float64 seconds.
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

    if (!skipCorrIMU_)
    {outBag.write("/gt_box/cpt7/gps/gps", ros_utc_time, gpsCommonMsg);
      gpsCommonQuque_.pop();
      
    }

    if (!skipROSIMU_)
    {
      outBag.write("/gt_box/cpt7/gps/fix", ros_utc_time, navsatFixMsg);
      rosFixQueue_.pop();
    }

    skipGpsFix_ = false;
    skipGPSCommon_ = false;
  }

  return true;
}

bool RosbagRangeDataProcessorRos::associateAndWriteIMUmsgs() {

  // The msgs are at 100hz ~ 10ms between msgs.
  uint64_t allowedReceivedTimeDifference = 5; // 5 ms
  uint64_t allowedTimeDifferenceBetweenConsecutiveMsgs = 15; // 15 ms
  double allowedConvertedTimeDifference = 0.1; // 100 ms
  
  // If the queue is empty, return false.
  if (corrIMUquque_.empty() || rosIMUQueue_.empty()) {
    ROS_WARN("Queue empty for IMU.");
    return false;
  }

  // if the queue sizes are not equal, return false. Write warning.
  if (corrIMUquque_.size() != rosIMUQueue_.size()) {
    ROS_WARN("IMU Queue sizes are not equal. Will try to associate messages.");
    // Write the number of msgs
    ROS_WARN_STREAM("corrIMUquque_ size: " << corrIMUquque_.size());
    ROS_WARN_STREAM("rosIMUQueue_ size: " << rosIMUQueue_.size());
    // return false;
  }
  
  // Iterate through the queues and associate the messages.
  while (!corrIMUquque_.empty() && !rosIMUQueue_.empty()) {

    novatel_oem7_msgs::CORRIMU corrIMU = corrIMUquque_.front();
    sensor_msgs::Imu rosIMU = rosIMUQueue_.front();

    uint64_t seqRosIMU = rosIMU.header.seq;
    uint64_t seqCorrIMU = corrIMU.header.seq;

    while(seqRosIMU != seqCorrIMU) {

      ROS_WARN_STREAM("Seq Numbers are not equal. seqRosIMU: " << seqRosIMU << " AND seqCorrIMU: " << seqCorrIMU);

      if (seqRosIMU < prevSeqRosIMU_)
      {
        ROS_ERROR_STREAM("ROS IMU Seq number going back. Previous " << prevSeqRosIMU_ << " AND seqRosIMU: " << seqRosIMU);
        rosIMUQueue_.pop();
        rosIMU = rosIMUQueue_.front();
        seqRosIMU = rosIMU.header.seq;
        continue;
      }
      
      if (seqCorrIMU < prevSeqCorrIMU_)
      {
        ROS_ERROR_STREAM("ROS CORRIMU Seq number going back. Previous " << prevSeqCorrIMU_ << " AND seqCorrIMU: " << seqCorrIMU);
        corrIMUquque_.pop();
        corrIMU = corrIMUquque_.front();
        seqCorrIMU = corrIMU.header.seq;
        continue;
      }

      if (seqRosIMU > seqCorrIMU) {
        // ROS_WARN_STREAM("ROS IMU Seq number is bigger. seqRosIMU " << seqRosIMU << " AND seqCorrIMU: " << seqCorrIMU);
        corrIMUquque_.pop();
        corrIMU = corrIMUquque_.front();
        
        seqCorrIMU = corrIMU.header.seq;
        continue;
      } else {
        // ROS_WARN("CORR IMU is ahead of ROS IMU. Will pop the ROS IMU.");
        // ROS_WARN_STREAM("seqCorrIMU number is bigger. seqCorrIMU " << seqCorrIMU << " AND seqRosIMU: " << seqRosIMU);
        rosIMUQueue_.pop();
        rosIMU = rosIMUQueue_.front();
        seqRosIMU = rosIMU.header.seq;
        continue;
      }
    }

    ros::Duration receivedRosTimeDifference = rosIMU.header.stamp - corrIMU.header.stamp;
    prevSeqRosIMU_ = seqRosIMU;
    prevSeqCorrIMU_ = seqCorrIMU;
    // ROS_ERROR_STREAM("Received ros IMU Time  : " << rosIMU.header.stamp);
    // ROS_ERROR_STREAM("Received CORR IMU time : " << corrIMU.header.stamp);

    // ROS_ERROR_STREAM("receivedRosTimeDifference : " << receivedRosTimeDifference*1000);


    // auto weekNum = message->nov_header.gps_week_number;
    // auto milliSeconds = message->nov_header.gps_week_milliseconds;
    // // ROS_INFO_STREAM("Original GPS time: ");
    // std::cout << std::setprecision(10) << "Week: " << weekNum << std::endl;
    // std::cout << std::setprecision(10) << "milliSeconds in Week: " << milliSeconds << std::endl;
    // std::cout << std::setprecision(10) << "Seconds in Week: " << (double)milliSeconds / 1000.0 << std::endl;

    // Get the header stamps of the messages. TIME OF RECORDING
    uint64_t rosIMUtoNSEC = rosIMU.header.stamp.toNSec();
    uint64_t corrIMUtoNSEC = corrIMU.header.stamp.toNSec();
    
    std::chrono::time_point<std::chrono::steady_clock, std::chrono::nanoseconds> receivedTimeRosImu(std::chrono::nanoseconds{rosIMUtoNSEC});
    auto receivedTimeRosImuMilliseconds = std::chrono::time_point_cast<std::chrono::milliseconds>(receivedTimeRosImu).time_since_epoch();

    std::chrono::time_point<std::chrono::steady_clock, std::chrono::nanoseconds> receivedTimeCorrIMU(std::chrono::nanoseconds{corrIMUtoNSEC});
    auto receivedTimeCorrIMUMilliseconds = std::chrono::time_point_cast<std::chrono::milliseconds>(receivedTimeCorrIMU).time_since_epoch();

    // Check if we roughlty received the msgs in similar times. This is only subject to communication delays. 100hz ~ 10ms between msgs.
    if ( std::fabs(receivedRosTimeDifference.toSec()*1000) > allowedReceivedTimeDifference) {
      ROS_WARN("Time difference between the two messages is too large. Will try to associate messages.");
      // The time difference is this
      std::cout << std::setprecision(15) << "Diff: " << std::fabs( receivedRosTimeDifference.toSec()*1000) << " ms." << std::endl;

      ROS_WARN_STREAM("receivedTimeRosImuMilliseconds: " << receivedTimeRosImuMilliseconds.count());
      ROS_WARN_STREAM("receivedTimeCorrIMUMilliseconds: " << receivedTimeCorrIMUMilliseconds.count());

      // If this happens, we know that the rosmsgs is ahead of corr imu and we need to hold the value of rosmsgs such that we can find a match of corrimu.
      if (receivedTimeRosImuMilliseconds.count() > receivedTimeCorrIMUMilliseconds.count()) {
        // This means some msgs were skipped. (not recorded)
        ros::Duration differenceBetweenConsecutiveRosIMUMsgs = rosIMU.header.stamp - lastIMUmsgsTime_;
        if ( differenceBetweenConsecutiveRosIMUMsgs.toSec()*1000 > allowedTimeDifferenceBetweenConsecutiveMsgs)  {
          std::cout << "Odom msg is ahead of inspva msg. We need to pop the inspva msg. Time Diff: " << std::setprecision(15)<< differenceBetweenConsecutiveRosIMUMsgs.toSec()*1000 << std::endl;
          skipROSIMU_ = true;
        }
      }

      if (receivedTimeCorrIMUMilliseconds.count() > receivedTimeRosImuMilliseconds.count()) {
        // Magic Number fix TT
        ros::Duration differenceBetweenConsecutiveRosCorrIMUMsgs = corrIMU.header.stamp - lastcorrIMUTime_;
        if ( differenceBetweenConsecutiveRosCorrIMUMsgs.toSec()*1000 > allowedTimeDifferenceBetweenConsecutiveMsgs) {
          // INSPVA msg is ahead of odom msg. We need to pop the odom msg.
          std::cout << "CorrIMU msg is ahead of ros IMU msg. We need to pop the odom msg. Time Diff: " << std::setprecision(15)<< differenceBetweenConsecutiveRosCorrIMUMsgs.toSec()*1000 << std::endl;

          skipCorrIMU_ = true;
        }
      }
    }

    // Update time holders
    lastIMUmsgsTime_ = rosIMU.header.stamp;
    lastcorrIMUTime_ = corrIMU.header.stamp;

    ///////////////////////////////////////////////////////////////////
    // GPS is ahead of UTC by 18 seconds, due to leap seconds.
    ///////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////
    const int64_t totalMsec= GPSTimeToMsec(corrIMU.nov_header);
    uint64_t gpsTimeMilli = gps2UtcWithMiilli(totalMsec);

    std::chrono::time_point<std::chrono::steady_clock, std::chrono::milliseconds> timePoint(std::chrono::milliseconds{gpsTimeMilli});
    auto secondsOut = std::chrono::time_point_cast<std::chrono::seconds>(timePoint).time_since_epoch();
    auto nanosecondsOut = std::chrono::time_point_cast<std::chrono::nanoseconds>(timePoint).time_since_epoch() - secondsOut;

    ros::Time ros_utc_time;
    ros_utc_time.sec = secondsOut.count();
    ros_utc_time.nsec = nanosecondsOut.count();

    // ROS_ERROR_STREAM("Estimated ros Time: " << ros_utc_time);
    // ROS_ERROR_STREAM("Received Ros time : " << corrIMU.header.stamp);

    ros::Duration conversionDiff = ros_utc_time - corrIMU.header.stamp;
    
    if (conversionDiff.toSec() > allowedConvertedTimeDifference)
    {
      ROS_ERROR_STREAM("BIG DIFFERENCE BETWEEN CONVERTED ROS TIME AND ORIGINAL RECEIVED TIME. GPS TIME CONVERSION MIGHT BE FAULTY : " << conversionDiff.toSec()*1000);
    }

    corrIMU.header.stamp = ros_utc_time;
    rosIMU.header.stamp = ros_utc_time;

    if (!skipCorrIMU_)
    {
      outBag.write("/gt_box/cpt7/corrimu", ros_utc_time, corrIMU);
      corrIMUquque_.pop();
      
    }

    if (!skipROSIMU_)
    {
      outBag.write("/gt_box/cpt7/gps/imu", ros_utc_time, rosIMU);
      rosIMUQueue_.pop();
    }

    skipROSIMU_ = false;
    skipCorrIMU_ = false;
    // const ros::WallTime first{ros::WallTime::now() + ros::WallDuration(0.5)};
    // ros::WallTime::sleepUntil(first);
  }

  return true;
}

bool RosbagRangeDataProcessorRos::processIMURosbag() {
  std::vector<std::string> IMUtopics;

  IMUtopics.push_back("/gt_box/cpt7/corrimu");
  IMUtopics.push_back("/gt_box/cpt7/gps/imu");

  // Create me a high resolution clock timer from std chrono
  // Start the timer.
  startTime_ = std::chrono::steady_clock::now();

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
                  << " Here we go for IMU Rosbag "
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

    if (messageInstance.getTopic() == "/gt_box/cpt7/corrimu") {
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
      sensor_msgs::Imu::ConstPtr message = messageInstance.instantiate<sensor_msgs::Imu>();
      if (message != nullptr) {
        rosIMUQueue_.push(*message);
        
      } else {
        isInvalidMessageInBag = true;
        ROS_WARN("Invalid message found in ROS bag.");
      }
    }

    // const ros::WallDuration processingWallDurationActual{ros::WallTime::now() - wallStampLastIteration};
    // const ros::Duration processingDurationActual{tracker - stampLastIteration};
    // const auto rosWallTimeRatio{processingDurationActual.toSec() / processingWallDurationActual.toSec()};
    // // Print walltime ratio
    // if (rosWallTimeRatio > 1.1) {
    //   ROS_WARN_STREAM("Walltime ratio is: " << rosWallTimeRatio);
    // }

  }

  associateAndWriteIMUmsgs();

  ROS_INFO("Finished running through the bag for IMU msgs.");
  const ros::Time bag_begin_time = view.getBeginTime();
  const ros::Time bag_end_time = view.getEndTime();
  endTime_ = std::chrono::steady_clock::now();

  std::cout << "Rosbag processing finished. Rosbag duration: " << (bag_end_time - bag_begin_time).toSec() << " sec."
            << " Time elapsed for processing: " << elapsedSeconds() << " sec. \n \n";

  bag.close();
  return true;
}

}
