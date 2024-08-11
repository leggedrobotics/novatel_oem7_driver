// #include <gflags/gflags.h>
// #include <glog/logging.h>
// #include <open3d/Open3D.h>
// #include "open3d_slam/Parameters.hpp"
// #include "open3d_slam_lua_io/parameter_loaders.hpp"
// #include "open3d_slam_ros/SlamMapInitializer.hpp"
// #include "open3d_slam_ros/creators.hpp"
// #include "open3d_slam_ros/helpers_ros.hpp"
// #include <novatel_oem7_driver/oem7_receiver_if.hpp>
#include <ros/ros.h>
// #include <novatel_oem7_driver/oem7_ros_messages.hpp>
// #include <novatel_oem7_msgs/HEADING2.h>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.h>
// #include <novatel_oem7_driver/oem7_ros_messages.hpp>
// #include <novatel_oem7_driver/oem7_messages.h>
// #include <novatel_oem7_driver/oem7_imu.hpp>
// #include "sensor_msgs/Imu.h"
// #include "novatel_oem7_msgs/CORRIMU.h"
// #include "novatel_oem7_msgs/IMURATECORRIMU.h"
// #include "novatel_oem7_msgs/INSSTDEV.h"
// #include "novatel_oem7_msgs/INSCONFIG.h"
// #include "novatel_oem7_msgs/INSPVA.h"
// #include "novatel_oem7_msgs/INSPVAX.h"
// #include "novatel_oem7_msgs/TIME.h"
// #include "novatel_oem7_msgs/HEADING2.h"
// #include "novatel_oem7_msgs/BESTGNSSPOS.h"
// #include "novatel_oem7_msgs/BESTPOS.h"
// #include "novatel_oem7_msgs/BESTVEL.h"
// #include "novatel_oem7_msgs/BESTUTM.h"
// #include "novatel_oem7_msgs/PPPPOS.h"
// #include "novatel_oem7_msgs/TERRASTARINFO.h"
// #include "novatel_oem7_msgs/TERRASTARSTATUS.h"
// #include "novatel_oem7_msgs/INSPVA.h"
// #include "novatel_oem7_msgs/INSPVAX.h"
// #include "novatel_oem7_msgs/INSCONFIG.h"
// #include "novatel_oem7_msgs/INSSTDEV.h"
// #include "novatel_oem7_msgs/CORRIMU.h"
// #include "novatel_oem7_msgs/RXSTATUS.h"
// #include "novatel_oem7_msgs/TIME.h"


// #include <boost/scoped_ptr.hpp>
// #include <oem7_ros_publisher.hpp>
#include "novatel_oem7_driver/replayer.hpp"

// #include <math.h>
// #include <map>

// #include <oem7_ros_publisher.hpp>


int main(int argc, char* argv[0]) {
  using namespace novatel_oem7_driver;

  ros::init(argc, argv, "cpt7_replay_node");
  ros::NodeHandlePtr nh(new ros::NodeHandle("~"));

  std::shared_ptr<RosbagRangeDataProcessorRos> rosbagRangeDataProcessorRos;
  // Write me a nice function that checks if parameters exist in the rosparam server.
  rosbagRangeDataProcessorRos = std::make_shared<RosbagRangeDataProcessorRos>(nh);
  rosbagRangeDataProcessorRos->initialize();

  ros::shutdown();
  return 0;
}
