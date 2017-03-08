// ROS
#include <gtest/gtest.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int16.h>
// gesture_synchronizer
#include "vision_utils/timer.h"
#include "gesture_synchronizer/gesture_synchronizer.h"
#include "gesture_synchronizer/simple_player.h"
//#include "gesture_synchronizer/interpolation_player.h"

#define ASSERT_TRUE_TIMEOUT(cond, timeout) { ::vision_utils::Timer timer; while (timer.getTimeSeconds() < timeout && !(cond)) usleep(50 * 1000); } ASSERT_TRUE(cond)

std::vector<char> cb1vals;
void cb1(const std_msgs::Int8 & msg) {
  ROS_WARN("cb1():%i", msg.data);
  cb1vals.push_back(msg.data);
}
std::string cb1vals2str() { return vision_utils::iterable_to_string(cb1vals); }

std::vector<short> cb2vals;
void cb2(const std_msgs::Int16 & msg) {
  ROS_WARN("cb2():%i", msg.data);
  cb2vals.push_back(msg.data);
}
std::string cb2vals2str() { return vision_utils::iterable_to_string(cb2vals); }

TEST(TestSuite, empty) {
  GestureSynchronizer gs;
  ASSERT_TRUE(gs.getStatus() == IDLE);
}

////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, empty_gesture) {
  GestureSynchronizer gs;
  gesture_msgs::KeyframeGesture gesture;
  ASSERT_TRUE(gs.publish_gesture_and_wait(gesture));
}

////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, set_joint_name) {
  ros::NodeHandle nh_private("~");
  // create the joint player
  nh_private.setParam("joint_name", "joint2");
  SimplePlayer<std_msgs::Int8> player("joint1");
  ASSERT_TRUE(player.get_joint_name() == "joint1");
  SimplePlayer<std_msgs::Int8> player2;
  ASSERT_TRUE(player2.get_joint_name() == "joint2");
}

////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, publish_simple) {
  cb1vals.clear();
  // create the joint player
  std::string joint_name = "joint1";
  SimplePlayer<std_msgs::Int8> player(joint_name);
  // play a simple gesture made of a single joint value
  GestureSynchronizer gs;
  gesture_msgs::KeyframeGesture gesture;
  gesture_synchronizer::set_joint_at_given_time(gesture, joint_name, "a", 0);
  ASSERT_TRUE(gs.publish_gesture_and_wait(gesture));
  // check joint value was effectively sent
  ASSERT_TRUE_TIMEOUT(cb1vals.size() == 1, 1) << cb1vals2str();
  ASSERT_TRUE(cb1vals.front() == 'a') << cb1vals2str();
}

////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, publish_timeshift) {
  cb1vals.clear();
  // create the joint player
  std::string joint_name = "joint1";
  SimplePlayer<std_msgs::Int8> player(joint_name);
  // play a simple gesture made of a single joint value
  GestureSynchronizer gs;
  gesture_msgs::KeyframeGesture gesture;
  gesture_synchronizer::set_joint_at_given_time(gesture, joint_name, "b", 0.5);
  gesture_synchronizer::set_joint_at_given_time(gesture, joint_name, "z", 1);
  ASSERT_TRUE(gs.publish_gesture_and_wait(gesture));
  // check joint value was effectively sent
  ASSERT_TRUE_TIMEOUT(cb1vals.size() == 2, 2) << cb1vals2str();
  ASSERT_TRUE(cb1vals.front() == 'b') << cb1vals2str();
  ASSERT_TRUE(cb1vals.back() == 'z') << cb1vals2str();
}

////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, publish_2joints) {
  cb1vals.clear();
  cb2vals.clear();
  // create the joint player
  std::string joint1_name = "joint1", joint2_name = "joint2";
  SimplePlayer<std_msgs::Int8> p1(joint1_name);
  SimplePlayer<std_msgs::Int16> p2(joint2_name, "out2");
  // play a simple gesture made of a single joint value
  GestureSynchronizer gs;
  gesture_msgs::KeyframeGesture gesture;
  gesture_synchronizer::set_joint_at_given_time(gesture, joint1_name, "a", 0);
  // use two different methods to set the value
  gesture_synchronizer::set_joint_at_given_time_<short>(gesture, joint2_name,
                                                 42, 0);
  gesture_synchronizer::set_joint_at_given_time(gesture, joint2_name, "43", 1);
  ASSERT_TRUE(gs.publish_gesture_and_wait(gesture));
  // check joint value was effectively sent
  ASSERT_TRUE_TIMEOUT(cb1vals.size() == 1, 2) << cb1vals2str();
  ASSERT_TRUE(cb1vals.front() == 'a') << cb1vals2str();
  ASSERT_TRUE_TIMEOUT(cb2vals.size() == 2, 2) << cb2vals2str();
  ASSERT_TRUE(cb2vals.front() == 42) << cb2vals2str();
  ASSERT_TRUE(cb2vals.back() == 43) << cb2vals2str();
}

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char **argv){
  ros::init(argc, argv, "gtest_gesture_synchronizer");
  ros::AsyncSpinner spinner(0);
  spinner.start();
  ros::NodeHandle nh_public;
  ros::Subscriber joint1_sub = nh_public.subscribe("out", 1, cb1);
  ros::Subscriber joint2_sub = nh_public.subscribe("out2", 1, cb2);
  // Run all the tests that were declared with TEST() or TEST_F()
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

