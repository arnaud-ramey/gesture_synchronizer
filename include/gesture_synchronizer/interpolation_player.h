#ifndef interpolation_player_H
#define interpolation_player_H

// utils
//#include <interpolation/interpolator.h>
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

class InterpolationPlayer : public JointPlayer {
public:
  typedef double AngleValue;

  //! the maximmum error allowed for sending an ack message
  static const double POSITION_TOLERANCE_ACK = 2E-1;


  //////////////////////////////////////////////////////////////////////////////

  //! ctor - init the plotter
  InterpolationPlayer()
  {
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

  /*!
    Convert a string command order to a joint one
   \param curr_joint_value
   \param out
   \return bool
  */
  static inline bool joint_order_to_angle_value
  (const gesture_synchronizer::JointValue & curr_joint_value, AngleValue & out) {
    bool success = false;
    out = vision_utils::cast_from_string<double>(curr_joint_value, success);
    if (!success)
      ROS_WARN("could not cast %s to a motor order.",curr_joint_value.c_str());
    return success;
  }

  //////////////////////////////////////////////////////////////////////////////

  //! send an order to the joint - must be implemented by children
  virtual bool send_joint_command_angle(const AngleValue & curr_joint_value) = 0;

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

    // convert all _joint_values string values to doubles
    _joint_values_double.clear();
    for (unsigned int idx = 0; idx < _joint_values.size(); ++idx) {
      AngleValue curr_value;
      bool success =
          joint_order_to_angle_value(_joint_values[idx], curr_value);
      if (!success) {
        ROS_WARN("%s:impossible to set the current gesture.", _joint_name.c_str());
        return false;
      }
      _joint_values_double.push_back(curr_value);
    } // end loop idx

    // if not enough data, do nothing
    if (_keytimes.size() == 0) {
      ROS_INFO_STREAM(_joint_name << ":_keytimes is empty.");
      return true;
    }

    // check keys for initial and final gesture
    gesture_synchronizer::Time gesture_min_time, gesture_max_time;
    gesture_synchronizer::get_gesture_min_max_times(_current_gesture,
                                              gesture_min_time, gesture_max_time);
    // duplicate the first key at time 0
    if (_keytimes.front() > gesture_min_time) {
      ROS_INFO_STREAM(_joint_name << ":adding a key at gesture_min_time.");
      _keytimes.insert(_keytimes.begin(), gesture_min_time);
      _joint_values_double.insert(_joint_values_double.begin(),
                                  _joint_values_double.front());
    }
    if (_keytimes.back() < gesture_max_time) {
      ROS_INFO_STREAM(_joint_name << ":adding a key at gesture_max_time.");
      _keytimes.insert(_keytimes.end(), gesture_max_time);
      _joint_values_double.insert(_joint_values_double.end(),
                                  _joint_values_double.back());
    }

    // create data if some is missing
    if (_keytimes.size() == 1) {
      _keytimes.push_back(_keytimes.front() + 1);
      _keytimes.push_back(_keytimes.front() + 2);
      _joint_values_double.push_back(_joint_values_double.front());
      _joint_values_double.push_back(_joint_values_double.front());
      ROS_INFO("%s:there was only one keframe. "
               "Adding two intermediate keyframes. "
               "keytimes:%s, joint_values:%s",
               _joint_name.c_str(),
               vision_utils::accessible_to_string(_keytimes).c_str(),
               vision_utils::accessible_to_string(_joint_values).c_str());
    }

    if (_keytimes.size() == 2) {
      gesture_synchronizer::Time new_keytime =
          (_keytimes.front() + _keytimes.back()) / 2;
      AngleValue new_joint_value =
          (_joint_values_double.front() + _joint_values_double.back()) / 2;
      _keytimes.insert(_keytimes.begin() + 1, new_keytime);
      _joint_values_double.insert(_joint_values_double.begin() + 1,
                                  new_joint_value);
      ROS_INFO("%s:there were only two keyframes. "
               "Adding an intermediate keyframe. "
               "keytimes:%s, joint_values:%s",
               _joint_name.c_str(),
               vision_utils::accessible_to_string(_keytimes).c_str(),
               vision_utils::accessible_to_string(_joint_values_double).c_str());
    }

    // restore begin and end times
    _gesture_begin_time = _keytimes.front();
    _gesture_end_time = _keytimes.back();
    ROS_INFO_STREAM("_gesture_begin_time:" << _gesture_begin_time
                    << ", _gesture_end_time:" << _gesture_end_time);

    // create the interpolator
    _interpolator.set_points(_keytimes, _joint_values_double);
    return true;
  } // end gesture_preplay_hook()

  //////////////////////////////////////////////////////////////////////////////

  /*! the extension of GestureJointPlayer::send_joint_command()
    that sends the order to the ros joint topic */
  virtual void send_joint_command(const gesture_synchronizer::JointValue & joint_value_str) {
    ROS_INFO_THROTTLE(3, "%s:send_joint_command(%s)",
                      _joint_name.c_str(), joint_value_str.c_str());
    // convert to float
    AngleValue joint_value;
    bool success = joint_order_to_angle_value(joint_value_str, joint_value);
    if (!success) {
      ROS_WARN_STREAM(_joint_name << ": impossible to cast '"
                      << joint_value_str << "' to a joint value");
      return;
    }
    success = send_joint_command_angle(joint_value);
    if (!success) {
      ROS_WARN_STREAM(_joint_name << ": send_joint_command_angle("
                      << joint_value << ") returned an error code.");
      return;
    }
  } // end send_joint_command();

  //////////////////////////////////////////////////////////////////////////////

  void gesture_play_plot(bool plot_at_end = true, bool plot_at_each_frame = false) {
    ROS_INFO("%s:gesture_play_plot(id:%f)",
             _joint_name.c_str(), _current_gesture.header.stamp.toSec());

    // do nothing if the gesture is empty
    if (_keytimes.size() == 0)
      return;

    vision_utils::Timer _gesture_timer;
    ros::Rate _rate(10); // Hz
    std::vector<gesture_synchronizer::Time> plotter_x;
    std::vector<AngleValue> plotter_y;
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
      AngleValue curr_joint_value = _interpolator(curr_time_sec);
      ROS_DEBUG("Time %g sec: emitting %g.", curr_time_sec, curr_joint_value);
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

  } // end gesture_play_plot();

  //////////////////////////////////////////////////////////////////////////////

  AngleValue get_initial_joint_value() const {
    return _interpolator(_gesture_begin_time);
  }

  //////////////////////////////////////////////////////////////////////////////

  AngleValue get_final_joint_value() const {
    return _interpolator(_gesture_end_time);
  }

  //////////////////////////////////////////////////////////////////////////////

  std::vector<AngleValue> _joint_values_double;
  //! the interpolator for intermediate positions
  ::tk::spline _interpolator;
  //! the plotter for showing the data
  Gnuplot _plotter;

  bool _is_initial_position_reached;
}; // end class InterpolationPlayer

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

template<class _Msg>
class MyInterpolationPlayer : public InterpolationPlayer {
public:
  //! alias for the C type: bool, int, float, etc.
  typedef typename _Msg::_data_type  _Type;

  MyInterpolationPlayer() {
    _pub = _nh_public.advertise<_Msg>("out", 1);
  }

  void gesture_go_to_initial_position() {
    if (!_joint_values.empty())
      send_joint_command_angle(vision_utils::cast_from_string
                               <InterpolationPlayer::AngleValue>(_joint_values.front()));
  }

protected:
  bool send_joint_command_angle(const InterpolationPlayer::AngleValue & curr_joint_value) {
    _Msg msg;
    msg.data = curr_joint_value;
    _pub.publish(msg);
    return true;
  }

  //! the publisher for communicating with the real user
  ros::Publisher _pub;
}; // end class MyInterpolationPlayer

#endif // interpolation_player_H
