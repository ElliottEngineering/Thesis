cmake_minimum_required(VERSION 3.0.2)
project(ros_robotics)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages



find_package(catkin REQUIRED COMPONENTS
  roslaunch
  urdf
  controller_manager
  joint_state_controller
  robot_state_publisher
  roscpp
  std_msgs
  gazebo_msgs
  genmsg
  rospy
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

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

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )
generate_messages(DEPENDENCIES std_msgs)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
	CATKIN_DEPENDS message_runtime gazebo_ros 
#  INCLUDE_DIRS include
#  LIBRARIES ros_robotics
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########




## Specify additional locations of header files
## Your package locations should be listed before other locations

include_directories(include ${catkin_INCLUDE_DIRS})

## Declare a C++ library
add_library(terrain_library
 src/terrain_library.cpp
)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
#add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/ros_robotics_node.cpp)

#frosty boy

#add_executable(terrain_model src/terrain_model.cpp)
#target_link_libraries(terrain_model ${catkin_LIBRARIES})
#target_link_libraries(terrain_model terrain_library)

#add_executable(motor_model src/motor_model.cpp)
#target_link_libraries(motor_model ${catkin_LIBRARIES})


#add_executable(rope_model_FB src/rope_model_FB.cpp)
#target_link_libraries(rope_model_FB ${catkin_LIBRARIES})
#target_link_libraries(rope_model_FB terrain_library)

#add_executable(controller_FB src/controller_FB.cpp)
#target_link_libraries(controller_FB ${catkin_LIBRARIES})
#target_link_libraries(controller_FB terrain_library)



#worm motion robot
#add_executable(terrain_model_worm src/terrain_model_worm.cpp)
#target_link_libraries(terrain_model_worm ${catkin_LIBRARIES})
#target_link_libraries(terrain_model_worm terrain_library)

#add_executable(motor_model_worm src/motor_model_worm.cpp)
#target_link_libraries(motor_model_worm ${catkin_LIBRARIES})
#target_link_libraries(motor_model_worm terrain_library)

#add_executable(controller_worm src/controller_worm.cpp)
#target_link_libraries(controller_worm ${catkin_LIBRARIES})
#target_link_libraries(controller_worm terrain_library)



##triple vehicle train
#add_executable(terrain_model_train3 src/terrain_model_train3.cpp)
#target_link_libraries(terrain_model_train3 ${catkin_LIBRARIES})
#target_link_libraries(terrain_model_train3 terrain_library)

#add_executable(motor_model_train3 src/motor_model_train3.cpp)
#target_link_libraries(motor_model_train3 ${catkin_LIBRARIES})
#target_link_libraries(motor_model_train3 terrain_library)

#add_executable(controller_train3 src/controller_train3.cpp)
#target_link_libraries(controller_train3 ${catkin_LIBRARIES})
#target_link_libraries(controller_train3 terrain_library)

#add_executable(controller_train3_final src/controller_train3_final.cpp)
#target_link_libraries(controller_train3_final ${catkin_LIBRARIES})
#target_link_libraries(controller_train3_final terrain_library)

#add_executable(controller_train3_reverse src/controller_train3_reverse.cpp)
#target_link_libraries(controller_train3_reverse ${catkin_LIBRARIES})
#target_link_libraries(controller_train3_reverse terrain_library)

#add_executable(cable_model_train3 src/cable_model_train3.cpp)
#target_link_libraries(cable_model_train3 ${catkin_LIBRARIES})
#target_link_libraries(cable_model_train3 terrain_library)

#add_executable(speed_control_train3 src/speed_control_train3.cpp)
#target_link_libraries(speed_control_train3 ${catkin_LIBRARIES})
#target_link_libraries(speed_control_train3 terrain_library)

##five vehicle train
#add_executable(terrain_model_train5 src/terrain_model_train5.cpp)
#target_link_libraries(terrain_model_train5 ${catkin_LIBRARIES})
#target_link_libraries(terrain_model_train5 terrain_library)

#add_executable(motor_model_train5 src/motor_model_train5.cpp)
#target_link_libraries(motor_model_train5 ${catkin_LIBRARIES})
#target_link_libraries(motor_model_train5 terrain_library)

add_executable(controller_train5_crab src/controller_train5_crab.cpp)
target_link_libraries(controller_train5_crab ${catkin_LIBRARIES})
target_link_libraries(controller_train5_crab terrain_library)


add_executable(controller_train5_crab2 src/controller_train5_crab2.cpp)
target_link_libraries(controller_train5_crab2 ${catkin_LIBRARIES})
target_link_libraries(controller_train5_crab2 terrain_library)

add_executable(controller_train5 src/controller_train5.cpp)
target_link_libraries(controller_train5 ${catkin_LIBRARIES})
target_link_libraries(controller_train5 terrain_library)

add_executable(speed_control_train5 src/speed_control_train5.cpp)
target_link_libraries(speed_control_train5 ${catkin_LIBRARIES})
target_link_libraries(speed_control_train5 terrain_library)

#add_executable(spring_model_train5 src/spring_model_train5.cpp)
#target_link_libraries(spring_model_train5 ${catkin_LIBRARIES})
#target_link_libraries(spring_model_train5 terrain_library)

add_executable(energy_train5 src/energy_train5.cpp)
target_link_libraries(energy_train5 ${catkin_LIBRARIES})
target_link_libraries(energy_train5 terrain_library)

add_executable(controller_train5_1 src/controller_train5_1.cpp)
target_link_libraries(controller_train5_1 ${catkin_LIBRARIES})
target_link_libraries(controller_train5_1 terrain_library)

#add_executable(motor_model_service_train5 src/motor_model_service_train5.cpp)
#target_link_libraries(motor_model_service_train5 ${catkin_LIBRARIES})
#target_link_libraries(motor_model_service_train5 terrain_library)

add_executable(immobilization_detect_train5 src/immobilization_detect_train5.cpp)
target_link_libraries(immobilization_detect_train5 ${catkin_LIBRARIES})
target_link_libraries(immobilization_detect_train5 terrain_library)


#big idea vehicle
#add_executable(terrain_model_big_idea src/terrain_model_big_idea.cpp)
#target_link_libraries(terrain_model_big_idea ${catkin_LIBRARIES})
#target_link_libraries(terrain_model_big_idea terrain_library)

#add_executable(motor_model_big_idea src/motor_model_big_idea.cpp)
#target_link_libraries(motor_model_big_idea ${catkin_LIBRARIES})
#target_link_libraries(motor_model_big_idea terrain_library)

#add_executable(controller_big_idea_winch src/controller_big_idea_winch.cpp)
#target_link_libraries(controller_big_idea_winch ${catkin_LIBRARIES})
#target_link_libraries(controller_big_idea_winch terrain_library)





## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
catkin_install_python(PROGRAMS scripts/Xt.py scripts/Xt_train3.py scripts/Xt_worm.py scripts/Xt_robot43.py scripts/Xt_bigidea.py scripts/Xt_train5.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_ros_robotics.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
