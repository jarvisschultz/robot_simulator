cmake_minimum_required(VERSION 2.8.3)
project(robot_simulator)

find_package(catkin REQUIRED COMPONENTS
  puppeteer_msgs
  std_msgs
  rospy
  roscpp
  nav_msgs
  tf
  tf_conversions
  angles
  visualization_msgs)

set(ROS_BUILD_TYPE Release)

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  )


catkin_package()

## add eigen functionality
include_directories(${EIGEN_INCLUDE_DIRS})
add_definitions(${EIGEN_DEFINITIONS})


add_executable(robot_simulator src/robot_simulator.cpp)
add_executable(rand_test src/rand_test.cpp)
add_executable(mass_marker src/mass_marker.cpp)
add_executable(planar_coordinator src/planar_coordinator.cpp)


add_dependencies(robot_simulator ${catkin_EXPORTED_TARGETS})
add_dependencies(rand_test ${catkin_EXPORTED_TARGETS})
add_dependencies(mass_marker ${catkin_EXPORTED_TARGETS})
add_dependencies(planar_coordinator ${catkin_EXPORTED_TARGETS})


target_link_libraries(robot_simulator ${catkin_LIBRARIES})
target_link_libraries(rand_test ${catkin_LIBRARIES})
target_link_libraries(mass_marker ${catkin_LIBRARIES})
target_link_libraries(planar_coordinator ${Boost_LIBRARIES} ${catkin_LIBRARIES})
