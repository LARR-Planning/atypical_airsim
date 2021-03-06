cmake_minimum_required(VERSION 3.10.0)
project(atypical_ros)

# set this to path to AirSim root folder if you want your catkin workspace in a custom directory
set(AIRSIM_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../../../../AirSim/)

add_subdirectory("${AIRSIM_ROOT}/cmake/rpclib_wrapper" rpclib_wrapper)
add_subdirectory("${AIRSIM_ROOT}/cmake/AirLib" AirLib)
add_subdirectory("${AIRSIM_ROOT}/cmake/MavLinkCom" MavLinkCom)

set(CMAKE_CXX_STANDARD 11) 
set(RPC_LIB_INCLUDES " ${AIRSIM_ROOT}/external/rpclib/rpclib-2.2.1/include")
set(RPC_LIB rpc) # name of .a file with lib prefix
message(STATUS "found RPC_LIB_INCLUDES=${RPC_LIB_INCLUDES}")

# todo eigen3 in AirLib already
# find_package(Eigen3 REQUIRED) 
find_package(Boost REQUIRED)
find_package(OpenCV REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  mavros_msgs
  message_generation
  nav_msgs
  # nodelet
  roscpp
  rospy
  sensor_msgs
  std_msgs
  std_srvs
  tf
  tf2
  tf2_ros
  driving_msgs
)

add_message_files(
  FILES
  GimbalAngleEulerCmd.msg
  GimbalAngleQuatCmd.msg
  GPSYaw.msg
  VelCmd.msg
  VelCmdGroup.msg
)

add_service_files(
  FILES
  SetGPSPosition.srv
  Takeoff.srv
  TakeoffGroup.srv
  Land.srv
  LandGroup.srv
  Reset.srv
  SetLocalPosition.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  # LIBRARIES airsim_ros
  CATKIN_DEPENDS message_runtime roscpp std_msgs # nodelet 
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${AIRSIM_ROOT}/AirLib/deps/eigen3
  ${AIRSIM_ROOT}/AirLib/include
  ${RPC_LIB_INCLUDES}
  ${AIRSIM_ROOT}/MavLinkCom/include
  ${AIRSIM_ROOT}/MavLinkCom/common_utils
  ${OpenCV_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_library(atypical_airsim_settings_parser src/airsim_settings_parser.cpp)
target_link_libraries(atypical_airsim_settings_parser ${catkin_LIBRARIES} AirLib)

#add_library(pd_position_controller_simple src/pd_position_controller_simple.cpp)
#add_dependencies(pd_position_controller_simple ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#add_dependencies(pd_position_controller_simple ${catkin_EXPORTED_TARGETS})

#add_library(airsim_ros src/airsim_ros_wrapper.cpp)
#add_dependencies(airsim_ros ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(airsim_ros ${catkin_LIBRARIES} ${OpenCV_LIBS} yaml-cpp AirLib airsim_settings_parser)

#add_executable(airsim_node src/airsim_node.cpp)
#target_link_libraries(airsim_node airsim_ros ${catkin_LIBRARIES} AirLib)

#####
add_library(atypical_airsim_ros_car src/airsim_ros_wrapper_car.cpp)
add_dependencies(atypical_airsim_ros_car ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(atypical_airsim_ros_car ${catkin_LIBRARIES} ${OpenCV_LIBS} yaml-cpp AirLib atypical_airsim_settings_parser )

add_executable(atypical_airsim_car_node src/airsim_car_node.cpp)
target_link_libraries(atypical_airsim_car_node ${catkin_LIBRARIES} AirLib atypical_airsim_ros_car )
#####


#add_executable(pd_position_controller_simple_node src/pd_position_controller_simple_node.cpp)
#target_link_libraries(pd_position_controller_simple_node pd_position_controller_simple airsim_ros_car ${catkin_LIBRARIES} AirLib)

add_executable(twist2stamp src/twist2stamp.cpp)
target_link_libraries(twist2stamp ${catkin_LIBRARIES} AirLib)


add_library(pid_controller src/car_pid.cpp)
add_dependencies(pid_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pid_controller ${catkin_LIBRARIES} AirLib )

add_executable(pid_node src/pid_wrapper_node.cpp src/pid_wrapper.cpp)
target_link_libraries(pid_node ${catkin_LIBRARIES} AirLib pid_controller )

install(TARGETS
    #list of nodes
    #pd_position_controller_simple_node
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS 
    #list of shared libraries
    #airsim_ros
    atypical_airsim_ros_car
    #pd_position_controller_simple
    pid_controller
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(FILES 
  #list of necessary files (xml...)
  README.md
  # settings.json
  # nodelet_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# for shipping binaries. run `$ catin_make install`
install(DIRECTORY
  launch
  rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  # PATTERN ".git" EXCLUDE
)
