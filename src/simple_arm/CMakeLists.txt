cmake_minimum_required(VERSION 2.8.3)
project(simple_arm)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
	std_msgs
	message_generation
	controller_manager
)

################################################
## Declare ROS messages, services and actions ##
################################################
## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

### Generate services in the 'srv' folder
add_service_files(
  FILES
  GoToPosition.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

### Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################


###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES simple_arm
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########
include_directories(include ${catkin_INCLUDE_DIRS})

# add_executable(simple_mover src/simple_mover.cpp)
# target_link_libraries(simple_mover ${catkin_LIBRARIES})
# add_dependencies(simple_mover simple_arm_generate_messages_cpp)

add_executable(arm_mover src/arm_mover.cpp)
target_link_libraries(arm_mover ${catkin_LIBRARIES})
add_dependencies(arm_mover simple_arm_generate_messages_cpp)

#############
## Install ##
#############


#############
## Testing ##
#############
