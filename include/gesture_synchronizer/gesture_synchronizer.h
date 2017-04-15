#ifndef gesture_synchronizer_H
#define gesture_synchronizer_H
#include <list>
// ROS
#include <ros/ros.h>
#include <std_msgs/Bool.h>
// vision_utils
#include <vision_utils/iterable_to_string.h>
// gesture_synchronizer
#include "gesture_definitions.h"
#include "gesture_io.h"
#include "gesture_msgs/KeyframeGesture.h"

#define DEBUG_PRINT(...)   {}
//#define DEBUG_PRINT(...)   ROS_INFO("GestureSynchronizer:" __VA_ARGS__)

enum Status {
  IDLE = 0,
  WAITING_ACKS = 1,
  READY_TO_PLAY = 2
};

class GestureSynchronizer {
public:

  GestureSynchronizer() {
    _gesture_synchronizer = _nh_public.advertise<gesture_msgs::KeyframeGesture>
        (gesture_synchronizer::gesture_topic_sync, 1);
    _ack_subscriber = _nh_public.subscribe
        (gesture_synchronizer::ack_topic, 100,
         &GestureSynchronizer::ack_callback, this);
    _play_order_publisher = _nh_public.advertise<gesture_synchronizer::Ack>
        (gesture_synchronizer::play_order_topic, 1);
    _stop_play_order_publisher = _nh_public.advertise<std_msgs::Bool>
        (gesture_synchronizer::stop_play_order_topic, 1);
    _status = IDLE;
    _abort_wait = false;
  } // end GestureSynchronizer

  ///////////////////////////////////////////////////////////////////////////

  bool publish_gesture_and_wait(gesture_msgs::KeyframeGesture & gesture) {
    // empty gesture -> nothing to do
    if (gesture.joint_names.empty())
      return true;

    //change status
    _status = WAITING_ACKS;
    DEBUG_PRINT("Publish gesture");
    //complete gesture information
    gesture.header.stamp = ros::Time::now();

    //Reset variables
    _current_gesture_stamp = gesture.header.stamp;
    // reset the missing acks
    _missing_acks.clear();
    for (unsigned int i = 0; i < gesture.joint_names.size(); ++i)
      _missing_acks.insert(gesture.joint_names[i]);

    //publish gesture
    _gesture_synchronizer.publish(gesture);

    // then wait for the listeners to confirm the reception of the gesture
    DEBUG_PRINT("Gesture with stamp %f: now waiting for %li acks: '%s'...",
                gesture.header.stamp.toSec(), _missing_acks.size(),
                vision_utils::iterable_to_string(_missing_acks).c_str());

    //wait for ack   (timeout = 3s)
    if (!waitForAck(3)) {
      ROS_WARN("Timeout while waiting for acks, %li acks missing:%s",
               _missing_acks.size(),
               vision_utils::iterable_to_string(_missing_acks).c_str());
      return false;
    }

    //Allow gesture start
    play_gesture();

    // get gesture duraction
    gesture_synchronizer::Time min_time, max_time;
    gesture_synchronizer::get_gesture_min_max_times(gesture, min_time, max_time);
    // then wait the needed time
    gesture_synchronizer::Time timeout = max_time + 3; // add 3 second for safety
    DEBUG_PRINT("Gesture with stamp %f: starting playing...  (timeout: %f seconds)",
                gesture.header.stamp.toSec(),timeout);
    //wait for ack   (timeout = gesturetime+10s)
    if (!waitForAck(timeout)) {
      ROS_WARN("Gesture play timeout exceeded");
      return false;
    }

    ROS_INFO("Gesture with stamp %f: gesture is over.",
             gesture.header.stamp.toSec());
    _status = IDLE;
    return true;
  }

  ///////////////////////////////////////////////////////////////////////////

  void publish_stop_and_wait() {
    //change status
    _status = WAITING_ACKS;

    //publish stop
    DEBUG_PRINT("Publish stop message");
    std_msgs::Bool message;
    message.data = true;
    _stop_play_order_publisher.publish(message);

    //wait for ack   (timeout = 3s)
    DEBUG_PRINT("Waiting the ACKs");
    bool success = waitForAck(3);
    if (!success) {
      ROS_WARN("Still alive timeout exceeded");
      return;
    }

    DEBUG_PRINT("Gesture play already stopped");
    _status = IDLE;
  }

  ///////////////////////////////////////////////////////////////////////////

  inline std::string get_gesture_topic() const {
    return _gesture_synchronizer.getTopic();
  }

  ///////////////////////////////////////////////////////////////////////////

  Status getStatus() {
    return _status;
  }

  void abortWait(){
    _abort_wait = true;
  }


  ///////////////////////////////////////////////////////////////////////////
  bool readFile(const std::string fileName,
                gesture_msgs::KeyframeGesture & gesture) {
    //read
    bool success = gesture_synchronizer::load_from_xml_file
        (gesture, fileName);

    return success;
  }

  ///////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////

private:

  ///////////////////////////////////////////////////////////////////////////
  //! return false if aborted or timeout exceeded
  bool waitForAck(double timeout){
    ros::Time end_time = ros::Time::now() + ros::Duration(timeout);

    while(_status == WAITING_ACKS && ros::Time::now() < end_time
          && !_abort_wait){
      usleep(100 /*ms*/ * 1000 /*us*/);
      //ros::Time::sleepUntil(end_time);
      ros::spinOnce();
    }

    if(_abort_wait){
      _abort_wait = false;
      return false;
    }
    else if(_status != WAITING_ACKS)
      return true;
    else
      return false;
  }

  ///////////////////////////////////////////////////////////////////////////
  void play_gesture() {
    DEBUG_PRINT("play_gesture(%f)", _ack_msg.stamp.toSec());
    //change status
    _status = WAITING_ACKS;
    _play_order_publisher.publish(_ack_msg);
  }

  ///////////////////////////////////////////////////////////////////////////

  void ack_callback(const gesture_synchronizer::Ack & ack_msg) {
    if (_status == IDLE) // dismiss the ack
      return;

    // if the ack corresponds to the emitted message,
    // increment the number of acks
    if (ack_msg.stamp != _current_gesture_stamp) {
      ROS_WARN("GestureSynchronizer: we received ack from '%s' for a gesture "
               "'%f' different from ours '%f'.",
               ack_msg.frame_id.c_str(),
               ack_msg.stamp.toSec(), _current_gesture_stamp.toSec());
      return;
    }
    _missing_acks.erase(ack_msg.frame_id);
    DEBUG_PRINT("Received ack from '%s' for gesture '%f', %li acks missing",
                ack_msg.frame_id.c_str(),
                _current_gesture_stamp.toSec() , _missing_acks.size());

    // if enough acks received, play the gesture
    if (_missing_acks.empty()) {
      _ack_msg = ack_msg;
      _status = READY_TO_PLAY;
    } // end if enough acks
  } // end ack_callback()

  /////////////////////////////////////////////////////////////////////////////

  Status _status;
  ros::NodeHandle _nh_public;
  //! the publisher for gesture messages
  ros::Publisher _gesture_synchronizer;
  //! the subscriber to acks, emitted by gesture joint players
  ros::Subscriber _ack_subscriber;
  //! the publisher for the play order, once all acks have been received
  ros::Publisher _play_order_publisher;
  //! the publisher for the stop play order
  ros::Publisher _stop_play_order_publisher;
  //! the number of acks we expect
  std::set<std::string> _missing_acks;
  //! the stamp of the published gesture
  ros::Time _current_gesture_stamp;

  gesture_synchronizer::Ack _ack_msg;
  bool _abort_wait;
}; // end class GestureSynchronizer

#endif // gesture_synchronizer_H
