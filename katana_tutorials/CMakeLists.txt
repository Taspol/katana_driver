cmake_minimum_required(VERSION 2.8.3)
project(katana_tutorials)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  actionlib
  trajectory_msgs
  control_msgs
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
  CATKIN_DEPENDS
  roscpp
  actionlib
  trajectory_msgs
  control_msgs
  sensor_msgs
  DEPENDS
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(follow_joint_trajectory_client src/follow_joint_trajectory_client.cpp)
target_link_libraries(follow_joint_trajectory_client ${catkin_LIBRARIES})
add_dependencies(follow_joint_trajectory_client ${catkin_EXPORTED_TARGETS})

add_executable(pr2_joint_trajectory_client src/pr2_joint_trajectory_client.cpp)
target_link_libraries(pr2_joint_trajectory_client ${catkin_LIBRARIES})
add_dependencies(pr2_joint_trajectory_client ${catkin_EXPORTED_TARGETS})

add_executable(test_inverse_kinematics src/test_inverse_kinematics.cpp)
target_link_libraries(test_inverse_kinematics ${catkin_LIBRARIES})
add_dependencies(test_inverse_kinematics ${catkin_EXPORTED_TARGETS})

add_executable(sixR_inverse_kinematics src/sixR_inverse_kinematics.cpp)
target_link_libraries(sixR_inverse_kinematics ${catkin_LIBRARIES})
add_dependencies(sixR_inverse_kinematics ${catkin_EXPORTED_TARGETS})

add_executable(offset_inverse_kinematics src/offset_inverse_kinematics.cpp)
target_link_libraries(offset_inverse_kinematics ${catkin_LIBRARIES})
add_dependencies(offset_inverse_kinematics ${catkin_EXPORTED_TARGETS})

install(TARGETS follow_joint_trajectory_client pr2_joint_trajectory_client test_inverse_kinematics sixR_inverse_kinematics offset_inverse_kinematics
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(DIRECTORY launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
