cmake_minimum_required(VERSION 3.0.2)
project(FABRIK_kinematics)


#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)


add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs moveit_core moveit_kinematics
  moveit_ros_planning moveit_core pluginlib tf_conversions moveit_ros_planning_interface)

find_package(Boost REQUIRED COMPONENTS system)
find_package(orocos_kdl)
find_package(kdl_parser)
find_package(Eigen3 REQUIRED)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES FABRIK_moveit_plugin
  CATKIN_DEPENDS roscpp geometry_msgs moveit_core moveit_kinematics moveit_ros_planning
    pluginlib tf_conversions
  DEPENDS Boost
)


###########
## Build ##
###########
include_directories(SYSTEM ${Boost_INCLUDE_DIR})
include_directories(include ${catkin_INCLUDE_DIRS})


add_library(FABRIK_moveit_plugin src/FABRIK_moveit_plugin.cpp src/chainiksolver_vel_mimic_svd.cpp)
#add_library(FABRIK_kinematic src/FABRIK_kinematic.cpp)


target_link_libraries(FABRIK_moveit_plugin
  ${catkin_LIBRARIES}
  ${kdl_parser_LIBRARIES}
  ${orocos_kdl_LIBRARIES}
  ${Boost_LIBRARIES})


#############
## Install ##
#############

install(TARGETS FABRIK_moveit_plugin
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# install header files
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# install moveit plugin description file
install(FILES FABRIK_moveit_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

