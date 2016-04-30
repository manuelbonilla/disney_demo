cmake_minimum_required(VERSION 2.8.3)
project( disney_demo )

find_package(orocos_kdl REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs

  cv_bridge
  image_transport 
  std_srvs
  sensor_msgs
  message_generation
  tf
  tf_conversions
  cmake_modules  #need for eigen
  controller_interface
  control_msgs
  forward_command_controller
  control_toolbox
  realtime_tools
  # urdf
  # kdl_parser

  sensor_msgs
  controller_interface
  control_msgs
  forward_command_controller
  control_toolbox
  realtime_tools
  urdf
  roscpp
  kdl_parser
  message_generation
  cmake_modules
  roscpp 
  tf_conversions 
  message_generation 
  std_msgs
  geometry_msgs 
  sensor_msgs
  lwr_controllers
  calibration
)


include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

link_directories(${catkin_LIBRARY_DIRS})
link_directories(${orocos_kdl_LIBRARY_DIRS})


add_definitions(-std=c++11 --all-warnings -O3) 

find_library(KDL_LIBRARY REQUIRED NAMES orocos-kdl HINTS ${orocos_kdl_LIBRARY_DIRS})
include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})



add_message_files(
   FILES
   PoseRPY.msg
   # RPY.msg
   # MultiPriorityTask.msg
)







catkin_package(
  CATKIN_DEPENDS message_runtime
)

catkin_package(
  DEPENDS orocos_kdl
  CATKIN_DEPENDS
  controller_interface
  control_msgs
  control_toolbox
  realtime_tools
  urdf
  forward_command_controller
  kdl_parser
  message_runtime
  tf_conversions
  lwr_hw
  lwr_controllers
  calibration
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME})


###########
## Build ##
###########


include_directories(${catkin_INCLUDE_DIRS} include)



# Declare a C++ executable
add_executable(disney_demo_node src/main.cpp src/disneyDemo.cpp)
## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(disney_demo_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# Specify libraries to link a library or executable target against
target_link_libraries(disney_demo_node
  ${catkin_LIBRARIES}
)


# Declare a C++ executable
add_executable(read_hand_node src/read_hand.cpp src/qbmove_communications.cpp)
## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(read_hand_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# Specify libraries to link a library or executable target against
target_link_libraries(read_hand_node
  ${catkin_LIBRARIES}  ${catkin_INCLUDE_DIRS}
)






add_library(${PROJECT_NAME}  src/teleoperation_controller_mt.cpp src/teleoperation_controller_mt_effort.cpp)
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} kdl_parser ${orocos_kdl_LIBRARIES})



install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(FILES disney_demo_controllers.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})