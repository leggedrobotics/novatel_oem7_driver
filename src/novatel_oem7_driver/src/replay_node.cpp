#include <ros/ros.h>
#include "novatel_oem7_driver/replayer.hpp"

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