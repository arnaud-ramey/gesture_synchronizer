#ifndef joint_player_H
#define joint_player_H

// ros
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Time.h>
#include <std_msgs/Bool.h>

// gesture_msgs
#include "gesture_io.h"
#include "gesture_definitions.h"

class JointPlayer {
public:

  static const double RESOLUTION_TIME = 0.1; //seconds

  JointPlayer(const std::string & joint_name = "") :
    _nh_private("~")
  {
    // set status as idle
    _current_status = StatusIdle;
    _abort_sleep = false;

    // get joint name from params if not supplied
    if (joint_name.empty())
    _nh_private.param<gesture_synchronizer::JointName>("joint_name", _joint_name, "joint_name");
    else
      _joint_name = joint_name;

    // init _gesture_subscriber
    _gesture_subscriber = _nh_public.subscribe
        (gesture_synchronizer::gesture_topic, 1,
         &JointPlayer::gesture_cb, this);

    // init _gesture_ack_publisher
    _gesture_ack_publisher = _nh_public.advertise<gesture_synchronizer::Ack>
        (gesture_synchronizer::ack_topic, 100);

    // init _gesture_play_order_subscriber
    _gesture_play_order_subscriber = _nh_public.subscribe
        (gesture_synchronizer::play_order_topic, 1,
         & JointPlayer::play_order_callback, this);

    // init _gesture_stop_play_order_subscriber
    _gesture_stop_play_order_subscriber = _nh_public.subscribe
        (gesture_synchronizer::stop_play_order_topic, 1,
         & JointPlayer::stop_callback, this);
  }

  //////////////////////////////////////////////////////////////////////////////

  /*!
  A custom function called even before extracting the keys of the gesture.
  You can access the current gesture with
  _current_gesture
  The gesture data for this joint has been stored in
    _keytimes   and   _joint_values
  */
  virtual bool gesture_preplay_hook() = 0;

  //////////////////////////////////////////////////////////////////////////////

  /*! How to move joint to initial position,
   which value is _joint_values.front().
  It is called only if there are values for this joint.
  It must be blocking.
  */
  virtual void gesture_go_to_initial_position() = 0;

  //////////////////////////////////////////////////////////////////////////////

  /*! Where the proper gesture playing is done.
    It should be made use of:
    - _keytimes , containing the keyframe times in seconds,
    - _joint_values  , containing the string values of the wanted joint
  */
  virtual void gesture_play() = 0;

  //////////////////////////////////////////////////////////////////////////////

  /*! When the gesture is stopped, reset necessary parameters.
  */
  virtual void gesture_stop() = 0;

  //////////////////////////////////////////////////////////////////////////////

  /*!
  set a custom gesture
   \param gesture
  */
  void gesture_cb(const gesture_msgs::KeyframeGesture & gesture) {
    ROS_INFO("%s:gesture_cb(%f)", _joint_name.c_str(),
             gesture.header.stamp.toSec());
    // make a local copy
    _current_gesture = gesture;


    // build the x, y data
    bool success = gesture_synchronizer::extract_all_joint_keys
        (gesture, _joint_name, _keytimes, _joint_values);
    if (!success) {
      ROS_WARN("%s:There was an error setting the gesture with stamp %f. "
               "Not playing it.",
               _joint_name.c_str(),
               _current_gesture.header.stamp.toSec());
      _current_status = StatusIdle;
      return;
    }

    // if there is no keyframe gesture for this joint,
    // no need to move to initial position:
    // send the ack
    if (_keytimes.size() == 0) {
      ROS_DEBUG("%s:The gesture with id:%f does not contain keyframes "
                "for this joint. Sending an ack.",
                _joint_name.c_str(), gesture.header.stamp.toSec());
      publish_ack_for_current_gesture();
      _current_status = StatusIdle;
      return;
    }

    gesture_preplay_hook();

    _gesture_begin_time = _keytimes.front();
    _gesture_end_time = _keytimes.back();
    //  ROS_INFO("keytimes:%s, joint_values:%s",
    //         vision_utils::accessible_to_string(_keytimes).c_str(),
    //         vision_utils::accessible_to_string(_joint_values).c_str());

    // set the order that will make the joint move
    // at the initial position of the gesture
    _current_status = StatusMovingToInitialPos;
    gesture_go_to_initial_position();

    // now we wait till the joint has reached this position.
    // the gesture will be played in the joint state callback
    _current_status = StatusWaitingForPlayOrder;
    publish_ack_for_current_gesture();
  } // end gesture_cb();

  //////////////////////////////////////////////////////////////////////////////

  //! return the name of the joint
  inline std::string get_joint_name() const {
    return _joint_name;
  }

  //////////////////////////////////////////////////////////////////////////////

  //! set a new joint name (the default one is obtained by parameter)
  inline void set_joint_name(const std::string & name) {
    _joint_name = name;
  }

  // ==============================  <new>  ============================

  //////////////////////////////////////////////////////////////////////////////

  //! return false if it must finish
  bool waitUntil(double time){
    double param, fractpart, intpart;;
    param = time/RESOLUTION_TIME;
    fractpart = modf(param , &intpart);

    ros::Time end_time = ros::Time::now();
    //ROS_DEBUG("param %f, intpart %f, fractpart %f", param, intpart, fractpart);
    for(int i=0; i < intpart && !_abort_sleep; i++){
      end_time += ros::Duration(RESOLUTION_TIME);
      ros::Time::sleepUntil(end_time);
      ros::spinOnce();
    }

    if(!_abort_sleep && fractpart>0){
      end_time += ros::Duration(fractpart);
      ros::Time::sleepUntil(end_time);
      ros::spinOnce();
    }

    if(_abort_sleep){
      _abort_sleep = false;
      return false;
    }

    return true;
  }
  // ==============================  </new>  ===========================

protected:

  // ==============================  <new>  ============================
  //////////////////////////////////////////////////////////////////////////////

  /*! enalbe abort flag */
  void stop_callback(const std_msgs::Bool &stopp_msg) {
    //if u are affected
    if(stopp_msg.data && _current_status == StatusPlayingGesture){
      _abort_sleep = true;
      gesture_stop();
    }
  }
  // ==============================  </new>  ===========================

  //////////////////////////////////////////////////////////////////////////////

  //! send an ack to confirm the joint is at initial position.
  inline void publish_ack_for_current_gesture() {
    ROS_INFO("%s:publish_ack_for_current_gesture(id:%f)",
             _joint_name.c_str(), _current_gesture.header.stamp.toSec());
    gesture_synchronizer::Ack msg;
    msg.stamp = _current_gesture.header.stamp;
    msg.frame_id = _joint_name;
    _gesture_ack_publisher.publish(msg);
  } // end publish_ack_for_current_gesture();


  //////////////////////////////////////////////////////////////////////////////

  /*! the callback when we receive a play order.
    If it is the current gesture, we will play it, then return to idle. */
  void play_order_callback(const gesture_synchronizer::Ack & msg) {
    ROS_INFO("%s:play_order_callback(id:%f)",
             _joint_name.c_str(),
             _current_gesture.header.stamp.toSec());

    // if the play order corresponds to our gesture, play it
    if (_current_status == StatusWaitingForPlayOrder) {
      if (msg.stamp != _current_gesture.header.stamp) {
        ROS_WARN("%s:We received a play order for a gesture with a stamp (%f)"
                 " different from the current gesture (%f)."
                 " That is weird, not playing the gesture.",
                 _joint_name.c_str(),
                 msg.stamp.toSec(),
                 _current_gesture.header.stamp.toSec());
        _current_status = StatusIdle;
        return;
      } // end if id mismatch
      // play the gesture with no plotting
      _current_status = StatusPlayingGesture;
      gesture_play();
      publish_ack_for_current_gesture(); // send an ack once the play is over
      _current_status = StatusIdle;
    }
  } // end play_msg_callback();

  //////////////////////////////////////////////////////////////////////////////

  std::vector<std::string> string_split(std::string input, char splitChar){
    std::istringstream ss(input.c_str());
    std::string token;
    std::vector<std::string> tokens;

    while(std::getline(ss, token, splitChar)) {
      tokens.push_back(token);
    }

    return tokens;
  }

  //////////////////////////////////////////////////////////////////////////////

  enum Status {
    StatusIdle = 0,
    StatusMovingToInitialPos = 1,
    StatusWaitingForPlayOrder = 2,
    StatusPlayingGesture = 3
  };
  //! the current status of the player
  Status _current_status;
  ros::NodeHandle _nh_private;
  ros::NodeHandle _nh_public;
  //! the subscriber to keyframe-gesture messages
  ros::Subscriber _gesture_subscriber;
  //! the publisher for the ack once the joint-of-interest is ready
  ros::Publisher _gesture_ack_publisher;
  //! the subscriber for the play order of the current gesture
  ros::Subscriber _gesture_play_order_subscriber;
  //! the subscriber for stop the current reproduction of the play order
  ros::Subscriber _gesture_stop_play_order_subscriber;

  //! the current gesture
  gesture_msgs::KeyframeGesture _current_gesture;


  // ///  data for the motion
  gesture_synchronizer::JointName _joint_name;
  std::vector<gesture_synchronizer::Time> _keytimes;
  std::vector<gesture_synchronizer::JointValue> _joint_values;
  gesture_synchronizer::Time _gesture_begin_time;
  gesture_synchronizer::Time _gesture_end_time;
  bool _abort_sleep;
}; // end class JointPlayer

#endif // joint_player_H
