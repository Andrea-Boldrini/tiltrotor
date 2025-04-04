cmake_minimum_required(VERSION 3.0.2)
project(model_plugins)

find_package(gazebo REQUIRED)    

find_package(catkin REQUIRED COMPONENTS
  gazebo_plugins                    
  gazebo_ros                        
  roscpp
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

include_directories(${GAZEBO_INCLUDE_DIRS})            
link_directories(${GAZEBO_LIBRARY_DIRS})               
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_library(joint_setposition_plugin SHARED src/joint_setposition_plugin.cc)
target_link_libraries(joint_setposition_plugin ${GAZEBO_LIBRARIES} ${roscpp_LIBRARIES} ${std_msgs_LIBRARIES})

add_library(gazebo_joint_torsional_spring SHARED src/gazebo_joint_torsional_spring.cc)
target_link_libraries(gazebo_joint_torsional_spring ${GAZEBO_LIBRARIES})

add_library(controller_setposition_plugin SHARED src/controller_setposition_plugin.cc)
target_link_libraries(controller_setposition_plugin ${GAZEBO_LIBRARIES} ${roscpp_LIBRARIES} ${mavros_msgs_LIBRARIES})

add_library(gripper_joint_plugin SHARED src/gripper_joint_plugin.cc)
target_link_libraries(gripper_joint_plugin ${GAZEBO_LIBRARIES} ${roscpp_LIBRARIES} ${std_msgs_LIBRARIES})
