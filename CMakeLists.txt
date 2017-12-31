cmake_minimum_required(VERSION 2.8.3)
project(behavior_look_at_uav)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED)

add_definitions(-std=c++11)

# Directories definition
set(BEHAVIOR_LOOK_AT_UAV_SOURCE_DIR
  src/source
)

set(BEHAVIOR_LOOK_AT_UAV_INCLUDE_DIR
  src/include
)

# Files declaration
set(BEHAVIOR_LOOK_AT_UAV_SOURCE_FILES
  ${BEHAVIOR_LOOK_AT_UAV_SOURCE_DIR}/behavior_look_at_uav.cpp
  ${BEHAVIOR_LOOK_AT_UAV_SOURCE_DIR}/behavior_look_at_uav_main.cpp
)

set(BEHAVIOR_LOOK_AT_UAV_HEADER_FILES
  ${BEHAVIOR_LOOK_AT_UAV_INCLUDE_DIR}/behavior_look_at_uav.h
)

### Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
  COMPONENTS
  roscpp
  std_msgs
  behavior_process
  droneMsgsROS
)

###################################
## catkin specific configuration ##
###################################
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS ${BEHAVIOR_LOOK_AT_UAV_INCLUDE_DIR}
  CATKIN_DEPENDS
  roscpp
  std_msgs
  drone_process
  behavior_process
  droneMsgsROS
  DEPENDS yaml-cpp
)

###########
## Build ##
###########
include_directories(
  ${BEHAVIOR_LOOK_AT_UAV_INCLUDE_DIR}
  ${BEHAVIOR_LOOK_AT_UAV_SOURCE_DIR}
)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_library(behavior_look_at_uav_lib ${BEHAVIOR_LOOK_AT_UAV_SOURCE_FILES} ${BEHAVIOR_LOOK_AT_UAV_HEADER_FILES})
add_dependencies(behavior_look_at_uav_lib ${catkin_EXPORTED_TARGETS})
target_link_libraries(behavior_look_at_uav_lib ${catkin_LIBRARIES})
target_link_libraries(behavior_look_at_uav_lib yaml-cpp)

add_executable(behavior_look_at_uav ${BEHAVIOR_LOOK_AT_UAV_SOURCE_DIR}/behavior_look_at_uav_main.cpp)
add_dependencies(behavior_look_at_uav ${catkin_EXPORTED_TARGETS})
target_link_libraries(behavior_look_at_uav behavior_look_at_uav_lib)
target_link_libraries(behavior_look_at_uav ${catkin_LIBRARIES})

#############
## Testing ##
#############
#if (CATKIN_ENABLE_TESTING)
#  catkin_add_gtest(BehaviorManagerTest ${BEHAVIOR_DESCRIPTOR_TEST_DIR}/behavior_manager_test.cpp)
#  target_link_libraries(BehaviorManagerTest behavior_take_off_lib)
#  target_link_libraries(BehaviorManagerTest ${catkin_LIBRARIES})

  #find_package(rostest REQUIRED)
  #add_rostest(launch/lau.test)
#endif()
