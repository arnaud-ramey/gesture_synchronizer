catkin_add_gtest(gtest_gesture_io       gtest_gesture_io.cpp)
target_link_libraries(gtest_gesture_io  ${catkin_LIBRARIES})

catkin_add_gtest(gtest_gesture_synchronizer       gtest_gesture_synchronizer.cpp)
target_link_libraries(gtest_gesture_synchronizer  ${catkin_LIBRARIES})
