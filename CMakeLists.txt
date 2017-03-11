### 1 - Required CMake Version
cmake_minimum_required(VERSION 2.8.3)
### 2- Package Name
project(gesture_synchronizer)
SET(CMAKE_BUILD_TYPE RelWithDebInfo) #None Debug Release RelWithDebInfo MinSizeRel
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra") # add extra warnings
### 3 - Find other CMake/Catkin packages needed for build
find_package(catkin REQUIRED COMPONENTS roscpp roslib
                                        std_msgs  gesture_msgs  vision_utils)
## System dependencies are found with CMake's conventions
FIND_PACKAGE(Qt4 REQUIRED)
INCLUDE(${QT_USE_FILE})
add_definitions(${QT_DEFINITIONS})
### 4 - Message/Service/Action Generators (add_message_files(), add_service_files(), add_action_files())
# nothing to do
### 5 - Invoke message/service/action generation (generate_messages())
## Generate added messages and services with any dependencies listed here
### 6 - Specify package build info export (catkin_package())
catkin_package(
  INCLUDE_DIRS src
  CATKIN_DEPENDS roscpp roslib std_msgs
  DEPENDS gesture_msgs
)
### 7 - Libraries/Executables to build (add_library()/add_executable()/target_link_libraries())
include_directories(
  include  ${catkin_INCLUDE_DIRS}  ${CMAKE_CURRENT_BINARY_DIR}
)
ADD_SUBDIRECTORY(src)
### 8 - Tests to build (catkin_add_gtest())
ADD_SUBDIRECTORY(test)
### 9 - Install rules (install())
install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
)
