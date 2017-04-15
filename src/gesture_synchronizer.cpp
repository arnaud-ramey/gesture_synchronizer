/*!
  \file        gesture_synchronizer.cpp
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2012/5/22

________________________________________________________________________________

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
________________________________________________________________________________

This node subscribes to the topic where the wanted gestures are sent,
load the corresponding XML files, and send the loaded gesture
on the corresponding topic.

\section Subscriptions
  - \b gesture_synchronizer::gesture_filename_topic
        [std_msgs::String]
        The filenames of the wanted gestures.

\section Publications
  - \b gesture_synchronizer::gesture_topic
        [gesture_msgs::KeyframeGesture]
        The loaded gestures.

 */

#include <queue>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <std_msgs/String.h>

#include "gesture_synchronizer/gesture_io.h"
#include "gesture_synchronizer/gesture_synchronizer.h"
#include "gesture_msgs/KeyframeGesture.h"
#include <vision_utils/to_lowercase.h>

GestureSynchronizer *gesture_pub;
std::list<gesture_msgs::KeyframeGesture> _gestures_queue;
std::list<gesture_msgs::KeyframeGesture>::iterator _gestures_queue_it;
bool _stop;

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

void gesture_cb(const gesture_msgs::KeyframeGesture & gesture){
  if(gesture.priority ==
          gesture_msgs::KeyframeGesture::PRIORITY_QUEUE_BEGINNING){
    _gestures_queue.push_front(gesture);
  }
    else if(gesture.priority ==
          gesture_msgs::KeyframeGesture::PRIORITY_QUEUE_FLUSH){
    _gestures_queue.clear();
    _gestures_queue.push_front(gesture);
  }
  else if(gesture.priority ==
          gesture_msgs::KeyframeGesture::PRIORITY_PREEMPTIVE_QUEUE_BEGINNING){
    if(gesture_pub->getStatus() != IDLE){
      gesture_pub->abortWait();
      _stop = true;
    }
    _gestures_queue.push_front(gesture);
  }
  else if(gesture.priority ==
          gesture_msgs::KeyframeGesture::PRIORITY_PREEMPTIVE_QUEUE_FLUSH){
    if(gesture_pub->getStatus() != IDLE){
      gesture_pub->abortWait();
      _stop = true;
    }
    _gestures_queue.clear();
    _gestures_queue.push_front(gesture);
  }
  /**
      Add gesture at the end of the queue
    */
  else /*if(gesture.priority == "queue_end")*/{
    _gestures_queue.push_back(gesture);
  }
}

////////////////////////////////////////////////////////////////////////////////
/**
  In this callback it is necessary to order the message queue depending on msg demands
*/
void gesture_filename_cb(const std_msgs::StringConstPtr & msg) {
  ROS_INFO("gesture_filename_cb('%s')", msg->data.c_str());

  std::vector<std::string> fileName_priority;
  vision_utils::StringSplit(msg->data.c_str(), ";", &fileName_priority);
  //ROS_DEBUG("SIZE  = %d",fileName_priority.size());

  if(fileName_priority.size() == 2){
    //read file
    gesture_msgs::KeyframeGesture gesture;
    bool success = gesture_pub->readFile(fileName_priority[1], gesture);
    if (!success) {
      ROS_WARN("Impossible to load the gesture '%s'", fileName_priority[1].c_str());
      /*TODO -  Enviar un topic de error? */
      return;
    }
    std::string prio = fileName_priority[0];
    vision_utils::to_lowercase(prio);
    if (prio.find("preemptive_queue_beginning"))
      gesture.priority = gesture_msgs::KeyframeGesture::PRIORITY_PREEMPTIVE_QUEUE_BEGINNING;
    else if (prio.find("preemptive_queue_flush"))
      gesture.priority = gesture_msgs::KeyframeGesture::PRIORITY_PREEMPTIVE_QUEUE_FLUSH;
    else if (prio.find("queue_beginning"))
      gesture.priority = gesture_msgs::KeyframeGesture::PRIORITY_QUEUE_BEGINNING;
    else if (prio.find("queue_flush"))
      gesture.priority = gesture_msgs::KeyframeGesture::PRIORITY_QUEUE_FLUSH;
    else
      gesture.priority = gesture_msgs::KeyframeGesture::PRIORITY_QUEUE_END;
    //Add gesture to the queue
    gesture_cb(gesture);
  }

  else if(fileName_priority.size() == 1){
    //read file
    gesture_msgs::KeyframeGesture gesture;
    bool success = gesture_pub->readFile(fileName_priority[0], gesture);
    if (!success) {
      ROS_WARN("Impossible to load the gesture '%s'", fileName_priority[1].c_str());
      /*TODO -  Enviar un topic de error? */
      return;
    }
    gesture.priority = gesture_msgs::KeyframeGesture::PRIORITY_QUEUE_END;
    //Add gesture at the end of the queue
    gesture_cb(gesture);
  }
  else
    ROS_ERROR("Recived malformed message: '%s'",msg->data.c_str());

} // end gesture_filename_cb()

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char** argv) {
  ros::init(argc, argv, "gesture_synchronizer");
  ros::NodeHandle nh_public;

  ros::Subscriber gesture_filename_sub = nh_public.subscribe
      (gesture_synchronizer::gesture_filename_topic, 1, gesture_filename_cb);

  ros::Subscriber gesture_KeyframeGesture_sub = nh_public.subscribe
      (gesture_synchronizer::gesture_topic, 1, gesture_cb);

  gesture_pub = new GestureSynchronizer();
  _stop = false;

  ROS_INFO("gesture_synchronizer: getting XML filenames from '%s', "
           "republishing them to '%s'",
           nh_public.resolveName(gesture_synchronizer::gesture_filename_topic).c_str(),
           gesture_pub->get_gesture_topic().c_str());

  ros::Rate rate(10);
  while (ros::ok()) {

    if(_stop){
      //if(gesture_pub->getStatus() != IDLE)
      gesture_pub->publish_stop_and_wait();
      _stop = false;
    }
    else if(!_gestures_queue.empty()){
      gesture_msgs::KeyframeGesture gesture = _gestures_queue.front();
      _gestures_queue.pop_front();
      gesture_pub->publish_gesture_and_wait(gesture);
    }

    ros::spinOnce();
    rate.sleep();
  } // end while (ros::ok())

  delete gesture_pub;
  return 0;
} // end main()
