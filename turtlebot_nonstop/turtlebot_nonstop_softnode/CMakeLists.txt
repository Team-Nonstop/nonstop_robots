cmake_minimum_required(VERSION 2.8.3)
project(turtlebot_nonstop_softnode)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS geometry_msgs kobuki_msgs nav_msgs nodelet roscpp sensor_msgs tf)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   geometry_msgs#   kobuki_msgs#   nav_msgs#   sensor_msgs
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need



catkin_package(
   INCLUDE_DIRS include
   LIBRARIES turtlebot_nonstop_softnode softturtlebot_nonstop_nodelet
   CATKIN_DEPENDS nav_msgs sensor_msgs geometry_msgs kobuki_msgs roscpp nodelet tf
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(turtlebot_nonstop_softnode src/library/fake_turtlebot_nonstop_ros.cpp
                            src/library/fake_turtlebot_nonstop.cpp
)
target_link_libraries(turtlebot_nonstop_softnode ${catkin_LIBRARIES})

add_library(softturtlebot_nonstop_nodelet src/nodelet/turtlebot_nonstop_nodelet.cpp)
target_link_libraries(softturtlebot_nonstop_nodelet turtlebot_nonstop_softnode)


install(TARGETS turtlebot_nonstop_softnode
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(TARGETS softturtlebot_nonstop_nodelet
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)


install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(DIRECTORY plugins
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY param
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY rviz
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
