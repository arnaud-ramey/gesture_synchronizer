#ifndef interpolation_player_H
#define interpolation_player_H

// utils
#include <vision_utils/interpolator.h>
#include <vision_utils/gnuplot-cpp/gnuplot_i.hpp>
#include <vision_utils/accessible_to_string.h>
#include <vision_utils/timer.h>
#include <gesture_synchronizer/third_parties/spline.h>
// gesture_synchronizer
#include "joint_player.h"
#include "gesture_io.h"

/*!
  This is a player for keyframe gestures.
  It subscribes to the topic where gestures are emitted.
  When a gesture is received, it:
  1) goes to the start position
  2) send the ack message for this gesture
  3) waits for the play message
  4) when the play message has been received, it really moves.

  Needs to be implemented by the user:
  - gesture_go_to_initial_position()
  - send_joint_command_angle()

  */
template<class _Msg>
class InterpolationPlayer : public JointPlayer {
public:
  //! the maximmum error allowed for sending an ack message
  static const double POSITION_TOLERANCE_ACK = 2E-1;

  //////////////////////////////////////////////////////////////////////////////

  //! ctor - init the plotter
  InterpolationPlayer() {
    _pub = _nh_public.advertise<_Msg>("out", 1);
    // set plotter to dumb (no X)
    _plotter.set_terminal_std("dumb");
  } // end ctor

  //////////////////////////////////////////////////////////////////////////////

  /*! How to move to initial position.
    It must be blocking.
    */
  virtual void gesture_go_to_initial_position() {
    ROS_INFO("%s:gesture_go_to_initial_position(id:%f)",
             _joint_name.c_str(), _current_gesture.header.stamp.toSec());

    // send the initial order
    send_joint_command_angle( get_initial_joint_value() );

    // wait for it to be realized
    ros::Duration sleep_time(0.1);
    while(!_is_initial_position_reached) {
      // spin to enable the receiving of the joint states
      ros::spinOnce();
      sleep_time.sleep();
    }
  } // end gesture_go_to_initial_position();

  //////////////////////////////////////////////////////////////////////////////

  /*! Where the proper gesture playing is done.
      It must be blocking till the gesture is over.
      It should be made use of:
        - _keytimes , containing the keyframe times in seconds,
        - _joint_values  , containing the string values of the wanted joint
    */
  virtual void gesture_play() {
    ROS_INFO("%s:gesture_play(id:%f, %li keytimes)",
             _joint_name.c_str(), _current_gesture.header.stamp.toSec(),
             _keytimes.size());
    gesture_play_plot(false, false);
  }

  ////////////////////////////////////////////////////////////////////////////

  /*! When the gesture is stopped, reset necessary parameters.
  */
  virtual void gesture_stop() {
  }

protected:

  //////////////////////////////////////////////////////////////////////////////

  //! send an order to the joint - must be implemented by children
  virtual bool send_joint_command_angle(const _Msg & msg) {
    _pub.publish(msg);
    return true;
  }

  //////////////////////////////////////////////////////////////////////////////

  /*!
    A custom function called even before extracting the keys of the gesture.
    You can access the current gesture with
    _current_gesture
    The gesture data for this joint has been stored in
      _keytimes   and   _joint_values
    */
  virtual bool gesture_preplay_hook() {
    ROS_INFO("%s:gesture_preplay_hook()", _joint_name.c_str());
    // set initial position as not reached
    _is_initial_position_reached = false;

    // create data if some is missing
    if (_keytimes.size() == 1) {
      _keytimes.push_back(_keytimes.front() + 1);
      _joint_values.push_back(_joint_values.front());
      ROS_INFO("%s:there was only one keframe. "
               "Adding one intermediate keyframe. "
               "keytimes:%s, joint_values:%s",
               _joint_name.c_str(),
               vision_utils::accessible_to_string(_keytimes).c_str(),
               vision_utils::accessible_to_string(_joint_values).c_str());
    }

    // restore begin and end times
    _gesture_begin_time = _keytimes.front();
    _gesture_end_time = _keytimes.back();
    ROS_INFO_STREAM("_gesture_begin_time:" << _gesture_begin_time
                    << ", _gesture_end_time:" << _gesture_end_time);

    // create the interpolator
    return _interpolator.train(_keytimes, _joint_values);
  } // end gesture_preplay_hook()

  //////////////////////////////////////////////////////////////////////////////

  bool gesture_play_plot(bool plot_at_end = true, bool plot_at_each_frame = false) {
    ROS_INFO("%s:gesture_play_plot(id:%f)",
             _joint_name.c_str(), _current_gesture.header.stamp.toSec());

    // do nothing if the gesture is empty
    if (_keytimes.size() == 0)
      return true;

    vision_utils::Timer _gesture_timer;
    ros::Rate _rate(10); // Hz
    std::vector<gesture_synchronizer::Time> plotter_x;
    std::vector<_Msg> plotter_y;
    if (plot_at_each_frame || plot_at_end) {
      int expected_time_steps = (_gesture_end_time - _gesture_begin_time)
          / _rate.expectedCycleTime().toSec();
      plotter_x.reserve(expected_time_steps);
      plotter_y.reserve(expected_time_steps);
    } // end if plot_at_each_frame || plot_at_end

    while(true) {
      vision_utils::Timer::Time curr_time_sec = _gesture_timer.getTimeMilliseconds() / 1000.f;
      // stop when neeed
      if (curr_time_sec > _gesture_end_time - _gesture_begin_time) {
        // send final position
        send_joint_command_angle(get_final_joint_value());
        // quit
        break;
      }

      // get the value of the joint at that time
      _Msg curr_joint_value;
      if (!_interpolator.predict(curr_time_sec, curr_joint_value)) {
        ROS_WARN("interpolator.predict() at time %g returned an error!",
                 curr_time_sec);
        return false;
      }
      ROS_DEBUG("Time %g sec: emitting '%s'.", curr_time_sec,
                vision_utils::msg2string(curr_joint_value).c_str());
      send_joint_command_angle(curr_joint_value);

      // plot the stuff
      if (plot_at_each_frame || plot_at_end) {
        plotter_x.push_back(curr_time_sec);
        plotter_y.push_back(curr_joint_value);
      } // end if plot_at_each_frame || plot_at_end
      if (plot_at_each_frame) {
        _plotter.reset_all();
        _plotter.remove_tmpfiles();
        _plotter.set_style("lines").plot_xy(plotter_x, plotter_y, _joint_name);
        _plotter.showonscreen();
      }

      // sleep needed time
      _rate.sleep();
    } // end while (true)

    if (plot_at_end) {
      _plotter.reset_all();
      _plotter.remove_tmpfiles();
      _plotter.set_style("lines").plot_xy(plotter_x, plotter_y, _joint_name);
      _plotter.showonscreen();
    }
    return true;
  } // end gesture_play_plot();

  //////////////////////////////////////////////////////////////////////////////

  _Msg get_initial_joint_value() const {
    _Msg out;
    _interpolator.predict(_gesture_begin_time, out);
    return out;
  }

  //////////////////////////////////////////////////////////////////////////////

  _Msg get_final_joint_value() const {
    _Msg out;
    _interpolator.predict(_gesture_end_time, out);
    return out;
  }

  //////////////////////////////////////////////////////////////////////////////

  //! the interpolator for intermediate positions
  vision_utils::Interpolator<_Msg> _interpolator;
  //! the plotter for showing the data
  Gnuplot _plotter;
  //! the publisher for communicating with the real user
  ros::Publisher _pub;

  bool _is_initial_position_reached;
}; // end class InterpolationPlayer

#endif // interpolation_player_H
