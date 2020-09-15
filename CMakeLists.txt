cmake_minimum_required(VERSION 3.0.2)
project(sunsensor_pose_estimation)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
set(CMAKE_CXX_STANDARD)
#set(CMAKE_CXX_COMPILER "/usr/bin/gcc")
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  message_generation
  roscpp
  std_msgs
  message_filters
  std_msgs
  cspice
  #googletest
)
find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
#add_subdirectory(external/cspice)

option(UNIT_TEST "Build the unit test" ON)

add_message_files(
  FILES
  eulAng.msg
  s_vec.msg
  angles.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  )

catkin_package(
  DEPENDS
  	roscpp gazebo_ros
  INCLUDE_DIRS include ../cspice/include
  CATKIN_DEPENDS gazebo_ros roscpp std_msgs cspice ignition message_filters message_runtime googletest
)



include_directories(
 include
  ${catkin_INCLUDE_DIRS}
  /home/dev/src/cspice/include
)


add_executable(${PROJECT_NAME}_spice_pub src/spicy.cpp)
add_dependencies(${PROJECT_NAME}_spice_pub ${PROJECT_NAME}_generate_messages_cpp)
add_executable(${PROJECT_NAME}_spice_pub_earth src/spicy_earth.cpp)
add_dependencies(${PROJECT_NAME}_spice_pub_earth ${PROJECT_NAME}_generate_messages_cpp)
add_executable(${PROJECT_NAME}_pose_est src/poseEstimation.cpp)
add_dependencies(${PROJECT_NAME}_pose_est ${PROJECT_NAME}_generate_messages_cpp)
add_executable(${PROJECT_NAME}_sunsensor src/ss.cpp)
add_dependencies(${PROJECT_NAME}_sunsensor ${PROJECT_NAME}_generate_messages_cpp)
add_executable(${PROJECT_NAME}_inclinometer src/inclinometer.cpp)
add_dependencies(${PROJECT_NAME}_inclinometer ${PROJECT_NAME}_generate_messages_cpp)
add_executable(${PROJECT_NAME}_true_pose src/true_pose.cpp)
add_dependencies(${PROJECT_NAME}_true_pose ${PROJECT_NAME}_generate_messages_cpp)
#add_executable(${PROJECT_NAME}_sunPlugin src/sunPlugin.cpp)


 target_link_libraries(${PROJECT_NAME}_spice_pub
   ${catkin_LIBRARIES}
    "${CMAKE_CURRENT_SOURCE_DIR}/../cspice/lib/cspice.a"
   "${CMAKE_CURRENT_SOURCE_DIR}/../cspice/lib/libcspice.a"
 )
 
 target_link_libraries(${PROJECT_NAME}_pose_est
   ${catkin_LIBRARIES}
    "${CMAKE_CURRENT_SOURCE_DIR}/../cspice/lib/cspice.a"
   "${CMAKE_CURRENT_SOURCE_DIR}/../cspice/lib/libcspice.a"
 )
 
  target_link_libraries(${PROJECT_NAME}_spice_pub_earth
   ${catkin_LIBRARIES}
    "${CMAKE_CURRENT_SOURCE_DIR}/../cspice/lib/cspice.a"
   "${CMAKE_CURRENT_SOURCE_DIR}/../cspice/lib/libcspice.a"
 )
 
  target_link_libraries(${PROJECT_NAME}_sunsensor
   ${catkin_LIBRARIES}
   "${CMAKE_CURRENT_SOURCE_DIR}/../cspice/lib/cspice.a"
   "${CMAKE_CURRENT_SOURCE_DIR}/../cspice/lib/libcspice.a"
 )
 
   target_link_libraries(${PROJECT_NAME}_inclinometer
   ${catkin_LIBRARIES}
 )
 
   target_link_libraries(${PROJECT_NAME}_true_pose
   ${catkin_LIBRARIES}
 )
 
add_library(${PROJECT_NAME}_sun src/sunPlugin.cpp)
#target_link_libraries(${PROJECT_NAME}_sunPlugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
 
target_link_libraries(${PROJECT_NAME}_sun ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

install(
	TARGETS
	 ${PROJECT_NAME}_spice_pub
	 ${PROJECT_NAME}_spice_pub_earth
	 ${PROJECT_NAME}_pose_est
	 ${PROJECT_NAME}_sunsensor
	 ${PROJECT_NAME}_inclinometer
	 ${PROJECT_NAME}_true_pose
	RUNTIME DESTINATION
	 ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
 )
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)


#############
## Testing ##
#############

if(UNIT_TESTS)
	enable_testing()
	include(GoogleTest)
	add_subdirectory(tst)
endif()






# catkin_add_nosetests(test)
