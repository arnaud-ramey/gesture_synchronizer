// ROS
#include <gtest/gtest.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int16.h>
// gesture_synchronizer
#include "vision_utils/timer.h"
#include "gesture_synchronizer/gesture_synchronizer.h"
#include "gesture_synchronizer/simple_player.h"
#include "gesture_synchronizer/interpolation_player.h"

#define ASSERT_TRUE_TIMEOUT(cond, timeout) { ::vision_utils::Timer timer; while (timer.getTimeSeconds() < timeout && !(cond)) usleep(50 * 1000); } ASSERT_TRUE(cond)

vision_utils::Timer timer;
std::vector<double> cb_times;

std::vector<bool> cb_Empty_vals;
std::string cb_Empty_vals2str() { return vision_utils::iterable_to_string(cb_Empty_vals); }
void cb_Empty(const std_msgs::Empty & /*msg*/) {
  ROS_WARN("cb_Empty()");
  cb_times.push_back(timer.getTimeSeconds());
  cb_Empty_vals.push_back(true);
}

std::vector<char> cb_Int8_vals;
std::string cb_Int8_vals2str() { return vision_utils::iterable_to_string(cb_Int8_vals); }
void cb_Int8(const std_msgs::Int8 & msg) {
  ROS_WARN("cb_Int8():%i", msg.data);
  cb_times.push_back(timer.getTimeSeconds());
  cb_Int8_vals.push_back(msg.data);
}

std::vector<short> cb_Int16_vals;
std::string cb_Int16_vals2str() { return vision_utils::iterable_to_string(cb_Int16_vals); }
void cb_Int16(const std_msgs::Int16 & msg) {
  ROS_WARN("cb_Int16():%i", msg.data);
  cb_times.push_back(timer.getTimeSeconds());
  cb_Int16_vals.push_back(msg.data);
}

std::vector<std::string> cb_String_vals;
std::string cb_String_vals2str() { return vision_utils::iterable_to_string(cb_String_vals); }
void cb_String(const std_msgs::String & msg) {
  ROS_WARN("cb_String():'%s'", msg.data.c_str());
  cb_times.push_back(timer.getTimeSeconds());
  cb_String_vals.push_back(msg.data);
}

////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, empty) {
  GestureSynchronizer gs;
  ASSERT_TRUE(gs.getStatus() == IDLE);
}

////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, empty_gesture) {
  GestureSynchronizer gs;
  gesture_msgs::KeyframeGesture gesture;
  ASSERT_TRUE(gs.publish_gesture_and_wait(gesture));
  ASSERT_TRUE_TIMEOUT(gs.getStatus() == IDLE, 1);
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

template<class _Msg, class _T>
void test_simple(std::vector<_T> & cb_vals,
                 const std::string & topic_name,
                 const std::string & joint_value,
                 const _T & exp_value) {
  cb_vals.clear();
  // create the joint player
  std::string joint_name = "joint1";
  SimplePlayer<_Msg> player(joint_name, topic_name);
  // play a simple gesture made of a single joint value
  GestureSynchronizer gs;
  gesture_msgs::KeyframeGesture gesture;
  gesture_synchronizer::set_joint_at_given_time(gesture, joint_name, joint_value, 0);
  ASSERT_TRUE(gs.publish_gesture_and_wait(gesture));
  // check joint value was effectively sent
  ASSERT_TRUE_TIMEOUT(cb_vals.size() == 1, 1) << vision_utils::iterable_to_string(cb_vals);
  ASSERT_TRUE(cb_vals.front() == exp_value)   << vision_utils::iterable_to_string(cb_vals);
  ASSERT_TRUE_TIMEOUT(gs.getStatus() == IDLE, 1);
}

TEST(TestSuite, publish_simple_Empty) {
  test_simple<std_msgs::Empty, bool>(cb_Empty_vals, "out_Empty", "", true);
}
TEST(TestSuite, publish_simple_Int8) {
  test_simple<std_msgs::Int8, char>(cb_Int8_vals, "out_Int8", "42", (char) 42);
}
TEST(TestSuite, publish_simple_Int16) {
  test_simple<std_msgs::Int16, short>(cb_Int16_vals, "out_Int16", "42", (short) 42);
}
TEST(TestSuite, publish_simple_String) {
  test_simple<std_msgs::String, std::string>(cb_String_vals, "out_String", "toto", "toto");
}

////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, publish_no_number) {
  cb_Int8_vals.clear();
  // create the joint player
  std::string joint_name = "joint1";
  SimplePlayer<std_msgs::Int8> player(joint_name, "out_Int8");
  // play a simple gesture made of a single joint value
  GestureSynchronizer gs;
  gesture_msgs::KeyframeGesture gesture;
  gesture_synchronizer::set_joint_at_given_time(gesture, joint_name, "xxx", 0);
  ASSERT_TRUE(gs.publish_gesture_and_wait(gesture));
  sleep(1);
  // check joint value was effectively sent
  ASSERT_TRUE_TIMEOUT(cb_Int8_vals.size() == 0, 1) << cb_Int8_vals2str();
  ASSERT_TRUE_TIMEOUT(gs.getStatus() == IDLE, 1);
}

////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, publish_timeshift) {
  cb_Int8_vals.clear();
  // create the joint player
  std::string joint_name = "joint1";
  SimplePlayer<std_msgs::Int8> player(joint_name, "out_Int8");
  // play a simple gesture made of a single joint value
  GestureSynchronizer gs;
  gesture_msgs::KeyframeGesture gesture;
  gesture_synchronizer::set_joint_at_given_time(gesture, joint_name, "42", 0.5);
  gesture_synchronizer::set_joint_at_given_time(gesture, joint_name, "50", 1);
  ASSERT_TRUE(gs.publish_gesture_and_wait(gesture));
  // check joint value was effectively sent
  ASSERT_TRUE_TIMEOUT(cb_Int8_vals.size() == 2, 2) << cb_Int8_vals2str();
  ASSERT_TRUE(cb_Int8_vals.front() == (char) 42) << cb_Int8_vals2str();
  ASSERT_TRUE(cb_Int8_vals.back() == (char) 50) << cb_Int8_vals2str();
}

////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, publish_2joints) {
  cb_Int8_vals.clear();
  cb_Int16_vals.clear();
  // create the joint player
  std::string joint1_name = "joint1", joint2_name = "joint2";
  SimplePlayer<std_msgs::Int8> p1(joint1_name, "out_Int8");
  SimplePlayer<std_msgs::Int16> p2(joint2_name, "out_Int16");
  // play a simple gesture made of a single joint value
  GestureSynchronizer gs;
  gesture_msgs::KeyframeGesture gesture;
  gesture_synchronizer::set_joint_at_given_time(gesture, joint1_name, "42", 0);
  // use two different methods to set the value
  gesture_synchronizer::set_joint_at_given_time_<short>(gesture, joint2_name, 43, 0);
  gesture_synchronizer::set_joint_at_given_time(gesture, joint2_name, "44", 1);
  ASSERT_TRUE(gs.publish_gesture_and_wait(gesture));
  // check joint value was effectively sent
  ASSERT_TRUE_TIMEOUT(cb_Int8_vals.size() == 1, 2) << cb_Int8_vals2str();
  ASSERT_TRUE(cb_Int8_vals.front() == (char) 42) << cb_Int8_vals2str();
  ASSERT_TRUE_TIMEOUT(cb_Int16_vals.size() == 2, 2) << cb_Int16_vals2str();
  ASSERT_TRUE(cb_Int16_vals.front() == 43) << cb_Int16_vals2str();
  ASSERT_TRUE(cb_Int16_vals.back() == 44) << cb_Int16_vals2str();
}

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char **argv){
  ros::init(argc, argv, "gtest_gesture_synchronizer");
  ros::AsyncSpinner spinner(0);
  spinner.start();
  ros::NodeHandle nh_public;
  ros::Subscriber joint_Empty_sub  = nh_public.subscribe("out_Empty", 1, cb_Empty);
  ros::Subscriber joint_Int8_sub   = nh_public.subscribe("out_Int8", 1, cb_Int8);
  ros::Subscriber joint_Int16_sub  = nh_public.subscribe("out_Int16", 1, cb_Int16);
  ros::Subscriber joint_String_sub = nh_public.subscribe("out_String", 1, cb_String);
  // Run all the tests that were declared with TEST() or TEST_F()
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

