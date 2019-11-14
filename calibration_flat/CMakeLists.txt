cmake_minimum_required(VERSION 2.8.3)
project(calibration_flat)

## Compile as C++11, supported in ROS Kinetic and newer
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if (COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif (COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else ()
    message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif ()

find_package(catkin REQUIRED COMPONENTS
        actionlib
        actionlib_msgs
        geometry_msgs
        sensor_msgs
        message_generation
        roscpp
        tf2
        tf2_ros
        genmsg
        std_msgs
        moveit_msgs
        sensor_msgs
        iiwa_ros
        iiwa_msgs
        )
find_package(Boost REQUIRED COMPONENTS filesystem)
find_package(Eigen3 REQUIRED)

add_action_files(
        FILES
        ArmToLaserScannerCalibration.action
)
add_message_files(
        FILES
        UserConfirmation.msg
)

generate_messages(
        DEPENDENCIES
        std_msgs
        actionlib_msgs
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIRS}
)


catkin_package(
        INCLUDE_DIRS
        include
        LIBRARIES
        ${PROJECT_NAME}_lib
        CATKIN_DEPENDS
        actionlib
        message_runtime
        geometry_msgs
        sensor_msgs
        tf2
        tf2_ros
        iiwa_ros
        iiwa_msgs
        DEPENDS
        Eigen3


)

add_library(${PROJECT_NAME}_lib
        src/ArmToLaserScannerCalibrationNode.cpp
        src/Robot.cpp
        src/LaserScanner.cpp
        src/UserInput.cpp
        src/CalibrationObjectPoints.cpp)

add_dependencies(
        ${PROJECT_NAME}_lib
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(
        ${PROJECT_NAME}_lib
        ${catkin_LIBRARIES}
        ${Boost_LIBRARIES}
)

add_executable(
        arm_to_laser_scanner_calibration_node
        src/main.cpp
)

add_dependencies(
        arm_to_laser_scanner_calibration_node
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}

)

target_link_libraries(
        arm_to_laser_scanner_calibration_node
        ${PROJECT_NAME}_lib
        ${catkin_LIBRARIES}
)

