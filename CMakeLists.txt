cmake_minimum_required(VERSION 3.0.2)
project(turtlebot2i_lfe_navigation)

add_compile_options(-std=c++11)

set(CMAKE_BUILD_TYPE Release)

###################
## Find Packages ##
###################

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  cmake_modules
  ros_astra_sdk_wrapper
  actionlib
  move_base_msgs
  geometry_msgs
  message_generation
  tf
  dynamic_reconfigure
)

find_package(Boost REQUIRED COMPONENTS system thread)

################################################
## Declare ROS messages, services and actions ##
################################################

add_message_files(
        FILES
        BOLogMsg.msg
        STLogMsg.msg
)

generate_messages(
        DEPENDENCIES
        std_msgs
        geometry_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

#add dynamic reconfigure api
generate_dynamic_reconfigure_options(
        cfg/LfeNavReconfigure.cfg
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS
    roscpp
    std_msgs
    ros_astra_sdk_wrapper
    move_base_msgs
    geometry_msgs
    message_generation
    actionlib
    cmake_modules
    tf
    dynamic_reconfigure
)

###########
## Build ##
###########

## lfe_navigation library
include_directories(
    SYSTEM
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
)

add_library(lfe_navigation
    src/dynamic_obstacle_listener.cpp
    src/nav_lfe.cpp
    src/navigation_manager.cpp
    src/lfe_nav_config.cpp
        src/lfe_nav_logger.cpp include/turtlebot2i_lfe_navigation/lfe_nav_logger.h)

add_dependencies(lfe_navigation ${PROJECT_NAME}_gencfg ${PROJECT_NAME}_generate_messages_cpp)

target_link_libraries(lfe_navigation
        ${catkin_LIBRARIES}
        ${Boost_LIBRARIES}
)

## lfe_navigation_node executable
add_executable(lfe_navigation_node
        src/nav_lfe.cpp
)

target_link_libraries(lfe_navigation_node
        lfe_navigation
        ${catkin_LIBRARIES}
        ${Boost_LIBRARIES}
)

add_dependencies(lfe_navigation_node
        ${catkin_EXPORTED_TARGETS}
)


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_turtlebot2i_lfe_navigation.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
