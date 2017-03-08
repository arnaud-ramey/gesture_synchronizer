/*!
 * \file gtest_gesture_io.cpp
 *
 * Testing gesture_io interface from gesture player package
 *
 * \date Oct 2014
 * \author Irene Pérez
 */

#include <gtest/gtest.h>
#include <ros/package.h>
#include "gesture_synchronizer/gesture_io.h"
#include "gesture_synchronizer/gesture_definitions.h"
#include <vision_utils/iterable_to_string.h>

////////////////////////////////////////////////////////////////////////////////
// Auxiliar function to compare two gestures
////////////////////////////////////////////////////////////////////////////////
template<class _T>
bool vectors_equal(const std::vector<_T> & V1, const std::vector<_T> & V2) {
  if(V1.size() != V2.size()) {
    std::cout << "Different size of vectors:" << V1.size() << ", " << V2.size() << std::endl
              << vision_utils::iterable_to_string(V1)
              << ", " << vision_utils::iterable_to_string(V2) << std::endl;
    return false;
  }
  unsigned int n = V1.size();
  for (unsigned int i = 0; i < n; ++i) {
    if (V1[i] == V2[i])
      continue;
    std::cout << "Vectors differ:" << vision_utils::iterable_to_string(V1)
              << ", " << vision_utils::iterable_to_string(V2) << std::endl;
    return false;
  } // end for i
  return true;
}
bool gestures_equal(const gesture_msgs::KeyframeGesture & gesture1,
                    const gesture_msgs::KeyframeGesture & gesture2) {
  return (vectors_equal(gesture1.joint_names, gesture2.joint_names)
          && vectors_equal(gesture1.joint_values, gesture2.joint_values)
          && vectors_equal(gesture1.keytimes, gesture2.keytimes));
}

////////////////////////////////////////////////////////////////////////////////
// Test negative keytime
////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, test_negative_keytime) {
  gesture_msgs::KeyframeGesture gesture;
  bool success = gesture_synchronizer::set_joint_at_given_time(gesture, "joint2", "0", -1);
  EXPECT_FALSE(success);

}

////////////////////////////////////////////////////////////////////////////////
// Test load correct xml
////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, test_load_correct_xml) {

  gesture_msgs::KeyframeGesture gesture;
  std::ostringstream xml_full_path;
  xml_full_path << gesture_synchronizer::gesture_files_folder() << "test_correct_gesture.xml";

  bool success_reading = gesture_synchronizer::load_from_xml_file
      (gesture, xml_full_path.str());
  EXPECT_TRUE(success_reading);

  gesture_msgs::KeyframeGesture reference_gesture;
  gesture_synchronizer::set_joint_at_given_time(reference_gesture, "joint2", "0", 0);
  gesture_synchronizer::set_joint_at_given_time(reference_gesture, "joint2", "1", 2);
  gesture_synchronizer::set_joint_at_given_time(reference_gesture, "joint1", "0.5", 0.5);

  ASSERT_TRUE(gestures_equal(gesture, reference_gesture));
}

////////////////////////////////////////////////////////////////////////////////
// Test non existing xml
////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, test_non_existing_xml) {

  gesture_msgs::KeyframeGesture gesture;
  bool success_reading = gesture_synchronizer::load_from_xml_file
      (gesture, "non_existing.xml");
  EXPECT_FALSE(success_reading);
}

////////////////////////////////////////////////////////////////////////////////
// Test load xml with keytimes not in the correct order
////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, test_load_disordered_xml) {

  gesture_msgs::KeyframeGesture gesture;
  std::ostringstream xml_full_path;
  xml_full_path << gesture_synchronizer::gesture_files_folder() << "test_unordered_gesture.xml";

  bool success_reading = gesture_synchronizer::load_from_xml_file
      (gesture, xml_full_path.str());

  EXPECT_TRUE(success_reading);

  gesture_msgs::KeyframeGesture reference_gesture;
  gesture_synchronizer::set_joint_at_given_time(reference_gesture, "joint2", "0", 0);
  gesture_synchronizer::set_joint_at_given_time(reference_gesture, "joint2", "1", 2);
  gesture_synchronizer::set_joint_at_given_time(reference_gesture, "joint1", "0.5", 0.5);
  gesture_synchronizer::set_joint_at_given_time(reference_gesture, "joint2", "1", 1);
  gesture_synchronizer::set_joint_at_given_time(reference_gesture, "joint1", "1", 10);

  ASSERT_TRUE(gestures_equal(gesture, reference_gesture));
}


////////////////////////////////////////////////////////////////////////////////
// Test create and load xml
////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, test_create_and_load_xml) {

  gesture_msgs::KeyframeGesture gesture;
  gesture_synchronizer::set_joint_at_given_time(gesture, "joint2", "0", 0);
  gesture_synchronizer::set_joint_at_given_time(gesture, "joint2", "1", 2);
  gesture_synchronizer::set_joint_at_given_time(gesture, "joint1", "0.5", 0.5);
  gesture_synchronizer::save_to_xml_file_labeled(gesture, "/tmp/gesture.xml");

  gesture_msgs::KeyframeGesture gesture_clone;
  bool success_reading = gesture_synchronizer::load_from_xml_file(gesture_clone, "/tmp/gesture.xml");
  EXPECT_TRUE(success_reading);

  ASSERT_TRUE(gestures_equal(gesture, gesture_clone));
}

////////////////////////////////////////////////////////////////////////////////
// Test clearing a gesture at given times for all or specific joints
////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, test_clearing) {
  gesture_msgs::KeyframeGesture gesture;
  gesture_synchronizer::set_joint_at_given_time(gesture, "joint1", "0.5", 0.5);
  gesture_synchronizer::set_joint_at_given_time(gesture, "joint1", "1", 0.7);
  gesture_synchronizer::set_joint_at_given_time(gesture, "joint2", "0", 0);
  gesture_synchronizer::set_joint_at_given_time(gesture, "joint2", "1", 0.3);
  gesture_synchronizer::set_joint_at_given_time(gesture, "joint3", "1", 0.3);

  gesture_synchronizer::clear_all_joints_at_given_time(gesture, 1, 0.2);

  gesture_msgs::KeyframeGesture gesture_no_clearing;
  gesture_synchronizer::set_joint_at_given_time(gesture_no_clearing, "joint1", "0.5", 0.5);
  gesture_synchronizer::set_joint_at_given_time(gesture_no_clearing, "joint1", "1", 0.7);
  gesture_synchronizer::set_joint_at_given_time(gesture_no_clearing, "joint2", "0", 0);
  gesture_synchronizer::set_joint_at_given_time(gesture_no_clearing, "joint2", "1", 0.3);
  gesture_synchronizer::set_joint_at_given_time(gesture_no_clearing, "joint3", "1", 0.3);
  ASSERT_TRUE(gestures_equal(gesture, gesture_no_clearing));

  gesture_synchronizer::clear_joint_at_given_time(gesture, "joint2", 0, 0);

  gesture_msgs::KeyframeGesture gesture_clearing_specific_joint;
  gesture_synchronizer::set_joint_at_given_time(gesture_clearing_specific_joint, "joint1", "0.5", 0.5);
  gesture_synchronizer::set_joint_at_given_time(gesture_clearing_specific_joint, "joint1", "1", 0.7);
  gesture_synchronizer::set_joint_at_given_time(gesture_clearing_specific_joint, "joint2", "1", 0.3);
  gesture_synchronizer::set_joint_at_given_time(gesture_clearing_specific_joint, "joint3", "1", 0.3);

  ASSERT_TRUE(gestures_equal(gesture, gesture_clearing_specific_joint));

  gesture_synchronizer::clear_all_joints_at_given_time(gesture, 0.4, 0.11);  //gesture, time, tolerance

  gesture_msgs::KeyframeGesture gesture_clearing_various;
  gesture_synchronizer::set_joint_at_given_time(gesture_clearing_various, "joint1", "1", 0.7);

  ASSERT_TRUE(gestures_equal(gesture, gesture_clearing_various));
}

////////////////////////////////////////////////////////////////////////////////
// Test set_joint_at_given_time
////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, test_set_joints_at_given_time) {
  gesture_msgs::KeyframeGesture g;
  ASSERT_TRUE(gesture_synchronizer::set_joint_at_given_time(g, "joint1", "0", 0));
  ASSERT_TRUE(gesture_synchronizer::set_joint_at_given_time(g, "joint0", "0", 0));
  ASSERT_TRUE(gesture_synchronizer::set_joint_at_given_time(g, "joint2", "0", 0));
  ASSERT_TRUE(g.joint_names.size() == 3);
  ASSERT_TRUE(g.joint_names[0] == "joint0") << g.joint_names[0];
  ASSERT_TRUE(g.joint_names[1] == "joint1") << g.joint_names[1];
  ASSERT_TRUE(g.joint_names[2] == "joint2") << g.joint_names[2];
}

////////////////////////////////////////////////////////////////////////////////
// Test get_all_joints_at_given_time
////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, test_get_joints_at_given_time) {
  gesture_msgs::KeyframeGesture gin, gout, exp;
  gesture_synchronizer::set_joint_at_given_time(gin, "joint1", "0.5", 0.5);
  gesture_synchronizer::set_joint_at_given_time(gin, "joint1", "1", 0.7);
  gesture_synchronizer::set_joint_at_given_time(gin, "joint2", "0", 0);
  gesture_synchronizer::set_joint_at_given_time(gin, "joint2", "1", 0.3);
  gesture_synchronizer::set_joint_at_given_time(gin, "joint3", "1", 0.3);

  gesture_synchronizer::get_all_joints_at_given_time(gin, gout, 0.2, 0);
  ASSERT_TRUE(gestures_equal(gout, exp));

  gesture_synchronizer::get_all_joints_at_given_time(gin, gout, -0.3, 0);
  ASSERT_TRUE(gestures_equal(gout, exp));

  gesture_synchronizer::get_all_joints_at_given_time(gin, gout, 0.3, 0);
  gesture_synchronizer::set_joint_at_given_time(exp, "joint2", "1", 0.3);
  gesture_synchronizer::set_joint_at_given_time(exp, "joint3", "1", 0.3);
  ASSERT_TRUE(gestures_equal(gout, exp));

  gesture_synchronizer::clear_gesture(gout);
  gesture_synchronizer::clear_gesture(exp);
  gesture_synchronizer::get_all_joints_at_given_time(gin, gout, 0.4, 0.11);
  gesture_synchronizer::set_joint_at_given_time(exp, "joint2", "1", 0.3);
  gesture_synchronizer::set_joint_at_given_time(exp, "joint3", "1", 0.3);
  gesture_synchronizer::set_joint_at_given_time(exp, "joint1", "0.5", 0.5);
  ASSERT_TRUE(gestures_equal(gout, exp));

  // include all values
  gesture_synchronizer::clear_gesture(gout);
  gesture_synchronizer::get_all_joints_at_given_time(gin, gout, 0.4, 0.5);
  ASSERT_TRUE(gestures_equal(gout, gin));
}

////////////////////////////////////////////////////////////////////////////////
int main(int argc, char **argv){
  // Run all the tests that were declared with TEST() or TEST_F()
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

