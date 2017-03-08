#ifndef SIMPLE_PLAYER_H
#define SIMPLE_PLAYER_H

// ROS
#include "gesture_synchronizer/joint_player.h"
#include <vision_utils/to_lowercase.h>
#include <vision_utils/cast_from_string.h>

template<class _Msg>
class SimplePlayer : public JointPlayer {
public:
  //! alias for the C type: bool, int, float, etc.
  typedef typename _Msg::_data_type  _Type;

  SimplePlayer(const std::string & joint_name = "",
               const std::string & out_topic_name = "out")
    : JointPlayer(joint_name) {
    _pub = _nh_public.advertise<_Msg>(out_topic_name, 1);
  }

  bool gesture_preplay_hook() { return true; } // nothing to do
  void gesture_go_to_initial_position() {
    //if (!_joint_values.empty())
      //send(vision_utils::cast_from_string<_Type>(_joint_values.front()));
  }

  void gesture_play() {
    ROS_INFO("%s:gesture_play(id:%f, %li keytimes)",
             _joint_name.c_str(), _current_gesture.header.stamp.toSec(),
             _keytimes.size());
    ros::Time start_time = ros::Time::now();

    for (unsigned int key_idx = 0; key_idx < _keytimes.size(); ++key_idx) {
      ros::Time keytime = start_time + ros::Duration(_keytimes[key_idx]);
      gesture_synchronizer::JointValue key_value = _joint_values.at(key_idx);
      ROS_INFO("%s: Sleeping till %f, then sending '%s'...",
               _joint_name.c_str(), keytime.toSec(), key_value.c_str());
      ros::Time::sleepUntil(keytime);
      send(vision_utils::cast_from_string<_Type>(_joint_values[key_idx]));
    } // end loop key_idx
  }

  void gesture_stop() { return; } // nothing to do

protected:
  void send(const _Type & value) {
    _Msg msg;
    msg.data = value;
    _pub.publish(msg);
  }

  //! the publisher for communicating with the real user
  ros::Publisher _pub;
};

#endif // SIMPLE_PLAYER_H
