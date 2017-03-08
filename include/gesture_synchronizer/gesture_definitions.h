#ifndef GESTURE_DEFINITIONS_H
#define GESTURE_DEFINITIONS_H

#include <string>
#include <std_msgs/Header.h>
#include <ros/package.h>

/*! This file gathers useful definitions
  for the gesture_synchronizer package.
  For instance, the names of possible gestures
  and default used topics are here.
  */

namespace gesture_synchronizer {

/*! the default gesture topic.
    There are exchanged messages of type
    gesture_synchronizer/KeyframeGesture */
static const std::string gesture_topic = "keyframe_gesture";

/*! the default gesture topic.
    There are exchanged messages of type
    std_msgs::String */
static const std::string gesture_filename_topic = "keyframe_gesture_filename";

//! where the XML gesture files are stored
inline static const std::string gesture_files_folder() {
  return ros::package::getPath("gesture_msgs") + std::string("/data/");
}

/*! the topic used to confirm the joint is ready to play the gesture.
  The message itself is a std_msgs::Time ,
  it is used as an id for the wanted gesture
  and it is the stamp of the message */
static const std::string ack_topic = "keyframe_ack";

/*! the topic used to start playing the gesture
  once acks have been received.
  The message is a std_msgs::Time, the same as the ack. */
static const std::string play_order_topic = "keyframe_play_order";

// ==============================  <new>  ============================
/*! the topic used to stop the gesture playing.
  The message is a std_msgs::Bool. */
static const std::string stop_play_order_topic = "keyframe_stop";

// ==============================  </new>  ===========================

//! defines the acks we use
//typedef std_msgs::Time Ack;
typedef std_msgs::Header Ack;


//! the different default gestures
enum GestureDictionary {
  zero = 0,
  test_sin = 1,
  greet_jocker = 2,
  head_assert = 3,
  head_deny = 4,

  left_arm_up = 5,
  right_arm_up = 6,
  left_arm_down = 7,
  right_arm_down = 8,
  hor_neck_left = 9,
  ver_neck_up = 10,
  hor_neck_right = 11,
  ver_neck_down = 12,
  hor_neck_center = 13,
  ver_neck_center = 14,

  yupi = 15,
  tralara_jocker = 16,
  arrogant = 17,
  high_C = 18,
  blink = 19,
  gaze_loop = 20,
  gaze_hor_loop = 21,
  laugh = 22,
  wink_left = 23
};

} // end namespace gesture_synchronizer

#endif // GESTURE_DEFINITIONS_H
