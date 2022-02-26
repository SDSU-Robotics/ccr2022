# Project's name
PROJECT (ccr2022)

# Specify the minimum version for CMake
cmake_minimum_required(VERSION 3.1)

add_definitions(-std=c++11)

set(THREADS_PREFER_PTHREAD_FLAG ON)

find_package(catkin REQUIRED)
find_package (Threads REQUIRED)
find_package(catkin REQUIRED COMPONENTS
	roscpp
	rospy
	std_msgs
	sensor_msgs
	laser_geometry
	tf2
	tf2_ros
	geometry_msgs
	tf
	nav_msgs
)

catkin_package()

include_directories (src include ${catkin_INCLUDE_DIRS})

# HERE ARE THE THINGS YOU NEED FOR EACH FILE YOU CREATE
# ONLY FOR CPP FILES DON'T ADD H FILES THANKS
# add_executable(<name> ${PROJECT_SOURCE_DIR}/src/<name>.cpp)
# target_link_libraries(<name> ${catkin_LIBRARIES} Threads::Threads)
# target_link_libraries(<name> ${PROJECT_SOURCE_DIR}/lib/x86-64/libCTRE_Phoenix.a)
# target_link_libraries(<name> ${PROJECT_SOURCE_DIR}/lib/x86-64/libCTRE_PhoenixCCI.a)
# target_link_libraries(<name> ${PROJECT_SOURCE_DIR}/lib/x86-64/libCTRE_PhoenixPlatformLinuxSocketCan.a)
# target_link_libraries(<name> ${PROJECT_SOURCE_DIR}/lib/x86-64/libCTRE_PhoenixCanutils.a)

add_executable(MainController ${PROJECT_SOURCE_DIR}/src/MainController.cpp)
target_link_libraries(MainController ${catkin_LIBRARIES} Threads::Threads)

target_link_libraries(MainController ${PROJECT_SOURCE_DIR}/lib/x86-64/libCTRE_Phoenix.a)
target_link_libraries(MainController ${PROJECT_SOURCE_DIR}/lib/x86-64/libCTRE_PhoenixCCI.a)
target_link_libraries(MainController ${PROJECT_SOURCE_DIR}/lib/x86-64/libCTRE_PhoenixPlatformLinuxSocketCan.a)
target_link_libraries(MainController ${PROJECT_SOURCE_DIR}/lib/x86-64/libCTRE_PhoenixCanutils.a)

add_executable(MainDrive ${PROJECT_SOURCE_DIR}/src/MainDrive.cpp)
target_link_libraries(MainDrive ${catkin_LIBRARIES} Threads::Threads)

target_link_libraries(MainDrive ${PROJECT_SOURCE_DIR}/lib/x86-64/libCTRE_Phoenix.a)
target_link_libraries(MainDrive ${PROJECT_SOURCE_DIR}/lib/x86-64/libCTRE_PhoenixCCI.a)
target_link_libraries(MainDrive ${PROJECT_SOURCE_DIR}/lib/x86-64/libCTRE_PhoenixPlatformLinuxSocketCan.a)
target_link_libraries(MainDrive ${PROJECT_SOURCE_DIR}/lib/x86-64/libCTRE_PhoenixCanutils.a)
