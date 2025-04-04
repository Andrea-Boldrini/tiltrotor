cmake_minimum_required(VERSION 3.0.2)
project(tiltrotor_drone)

find_package(catkin REQUIRED 
    COMPONENTS
    mavros_msgs
    roscpp 
    rospy 
    std_msgs
    tf2
    tf2_ros
    dynamic_reconfigure
)

generate_dynamic_reconfigure_options(
  config/flight_controller.cfg
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS mavros_msgs
                   roscpp 
                   rospy 
                   std_msgs
                   tf2
                   tf2_ros
                   dynamic_reconfigure
)

include_directories(include
                    ${catkin_INCLUDE_DIRS})

add_executable(flight_control_exe src/flight_controller_main.cpp)
add_library(flight_control_lib src/flight_controller.cpp)
target_link_libraries(flight_control_exe flight_control_lib ${catkin_LIBRARIES})
add_dependencies(flight_control_exe ${PROJECT_NAME}_gencfg)
