cmake_minimum_required(VERSION 3.0.2)
project(FABRIK_kinematics_plugin)


#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)


add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs moveit_core moveit_kinematics
  moveit_ros_planning moveit_core pluginlib tf_conversions)

find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES FABRIK_kinematics_plugin
  CATKIN_DEPENDS roscpp geometry_msgs moveit_core moveit_kinematics moveit_ros_planning
    pluginlib tf_conversions
  DEPENDS Boost
)

include_directories(include)
add_library(FABRIK_kinematics_plugin src/FABRIK_kinematics_plugin.cpp)

###########
## Build ##
###########
set(MOVEIT_LIB_NAME FABRIK_kinematics_plugin)
set(CATKIN_PACKAGE_SHARE_DESTINATION home/atieh/Documents/PHD/PaintingWithFabrik/src/FABRIK_kinematics_plugin)
include_directories(SYSTEM ${Boost_INCLUDE_DIR})
include_directories(include ${catkin_INCLUDE_DIRS})
install(TARGETS ${MOVEIT_LIB_NAME} 
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

# install header files
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# install moveit plugin description file
install(FILES FABRIK_kinematics_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
#############
## Install ##
#############



