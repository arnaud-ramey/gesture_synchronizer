// ROS
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Int8.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Int16.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/Int32.h>
#include <std_msgs/UInt64.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/ColorRGBA.h>
#include <std_msgs/String.h>
// gesture_synchronizer
#include "gesture_synchronizer/simple_player.h"

//! instantiate a SimplePlayer
template<class T>
void launcher() {
  SimplePlayer<T> player;
  ROS_INFO("Starting a simple_player with name '%s'",
           player.get_joint_name().c_str());
  ros::spin();
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "simple_player");
  ros::NodeHandle nh_private("~");
  std::string joint_type = "";
  nh_private.param("joint_type", joint_type, joint_type);
  vision_utils::to_lowercase(joint_type);
  if (joint_type == "empty")
    launcher<std_msgs::Empty>();
  else if (joint_type == "bool")
    launcher<std_msgs::Bool>();
  else if (joint_type == "uint8")
    launcher<std_msgs::UInt8>();
  else if (joint_type == "int8")
    launcher<std_msgs::Int8>();
  else if (joint_type == "uint16")
    launcher<std_msgs::UInt16>();
  else if (joint_type == "int16")
    launcher<std_msgs::Int16>();
  else if (joint_type == "uint32")
    launcher<std_msgs::UInt32>();
  else if (joint_type == "int32")
    launcher<std_msgs::Int32>();
  else if (joint_type == "uint64")
    launcher<std_msgs::UInt64>();
  else if (joint_type == "int64")
    launcher<std_msgs::Int64>();
  else if (joint_type == "float32")
    launcher<std_msgs::Float32>();
  else if (joint_type == "float64")
    launcher<std_msgs::Float64>();
  else if (joint_type == "color")
    launcher<std_msgs::ColorRGBA>();
  else if (joint_type == "string")
    launcher<std_msgs::String>();

  //TEST(TestSuite, Point32)   { test_suite<geometry_msgs::Point32>(3); }
  //TEST(TestSuite, Point)     { test_suite<geometry_msgs::Point>(3); }
  //TEST(TestSuite, Vector3)   { test_suite<geometry_msgs::Vector3>(3); }
  //TEST(TestSuite, Pose2D)    { test_suite<geometry_msgs::Pose2D>(3); }
  //TEST(TestSuite, Quaternion){ test_suite<geometry_msgs::Quaternion>(4); }
  //TEST(TestSuite, Accel)     { test_suite<geometry_msgs::Accel>(6); }
  //TEST(TestSuite, Twist)     { test_suite<geometry_msgs::Twist>(6); }
  //TEST(TestSuite, Pose)      { test_suite<geometry_msgs::Pose>(7); }

  else {
    ROS_FATAL("The joint type '%s' is unknown", joint_type.c_str());
  }
  return 0;
}
