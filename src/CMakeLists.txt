SET(INCDIR ../include/gesture_synchronizer)

### gesture_gui
QT4_WRAP_UI(UI_GENERATED gesture_gui.ui)
QT4_WRAP_CPP(MOC_GENERATED ${INCDIR}/gesture_gui.h)
add_executable(gesture_gui.exe       gesture_gui.cpp
  ${INCDIR}/gesture_gui.h
  ${UI_GENERATED} ${MOC_GENERATED})
TARGET_LINK_LIBRARIES(gesture_gui.exe ${QT_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(gesture_gui.exe
    gesture_msgs_generate_messages_cpp
    ${catkin_EXPORTED_TARGETS})

### gesture_synchronizer
add_executable(gesture_synchronizer.exe   gesture_synchronizer.cpp
  ${INCDIR}/gesture_definitions.h
  ${INCDIR}/gesture_io.h
  ${INCDIR}/gesture_synchronizer.h)
target_link_libraries(gesture_synchronizer.exe   ${catkin_LIBRARIES})
add_dependencies(gesture_synchronizer.exe
  gesture_msgs_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

### interpolation_player
add_executable(interpolation_player.exe   interpolation_player.cpp
  ${INCDIR}/interpolation_player.h
  ${INCDIR}/joint_player.h)
target_link_libraries(interpolation_player.exe   ${catkin_LIBRARIES})
add_dependencies(interpolation_player.exe
  gesture_msgs_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

### simple_player
add_executable(simple_player.exe   simple_player.cpp
  ${INCDIR}/simple_player.h
  ${INCDIR}/joint_player.h)
target_link_libraries(simple_player.exe   ${catkin_LIBRARIES})
add_dependencies(simple_player.exe
  gesture_msgs_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
