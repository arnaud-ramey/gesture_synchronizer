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
  ROS_INFO("Starting a simple_player with joint name '%s'",
           player.get_joint_name().c_str());
  ros::spin();
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "simple_player");
  ros::NodeHandle nh_private("~");
  std::string joint_type = "";
  nh_private.param("joint_type", joint_type, joint_type);
  vision_utils::to_lowercase(joint_type);
  // std_msgs
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
  // geometry msgs
  else if (joint_type == "accel")
    launcher<geometry_msgs::Accel>();
  else if (joint_type == "point32")
    launcher<geometry_msgs::Point32>();
  else if (joint_type == "point")
    launcher<geometry_msgs::Point>();
  else if (joint_type == "pose2d")
    launcher<geometry_msgs::Pose2D>();
  else if (joint_type == "pose")
    launcher<geometry_msgs::Pose>();
  else if (joint_type == "quaternion")
    launcher<geometry_msgs::Quaternion>();
  else if (joint_type == "twist")
    launcher<geometry_msgs::Twist>();
  else if (joint_type == "vector3")
    launcher<geometry_msgs::Vector3>();

  else {
    ROS_FATAL("The joint type '%s' is unknown", joint_type.c_str());
  }
  return 0;
}
