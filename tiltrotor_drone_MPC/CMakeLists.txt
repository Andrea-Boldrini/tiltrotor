cmake_minimum_required(VERSION 3.0.2)
project(tiltrotor_drone)

# Required packages
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

# Find CasADi using pkg-config
find_package(PkgConfig REQUIRED)
pkg_check_modules(CASADI REQUIRED casadi)

# Find Eigen3
find_package(Eigen3 REQUIRED)

# Find qpOASES
find_package(qpOASES REQUIRED)

# Dynamic reconfigure
generate_dynamic_reconfigure_options(
  config/flight_controller.cfg
)

# Catkin package info
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

# Include directories
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${CASADI_INCLUDE_DIRS} 
    ${EIGEN3_INCLUDE_DIR}
    ${qpOASES_INCLUDE_DIR}
)

# Add source files
add_library(flight_control_lib
    src/flight_controller.cpp
    src/mpc_solver.cpp
    src/wrench_estimator.cpp
)

# Add executable
add_executable(flight_control_exe src/flight_controller_main.cpp)

# Link libraries for flight_control_lib
target_link_libraries(flight_control_lib
    ${catkin_LIBRARIES}
    ${CASADI_LIBRARIES} 
    ${qpOASES_LIBRARIES} 
)

# Link libraries for flight_control_exe
target_link_libraries(flight_control_exe
    flight_control_lib
    ${catkin_LIBRARIES}
    ${CASADI_LIBRARIES}
    ${qpOASES_LIBRARIES} 
)

# Dynamic reconfigure dependency
add_dependencies(flight_control_exe ${PROJECT_NAME}_gencfg)

