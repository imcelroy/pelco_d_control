cmake_minimum_required(VERSION 3.0.2)
project(pelco_d_control)

find_package(catkin REQUIRED COMPONENTS
    rospy
    std_msgs
    message_generation
)

catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
)


###################################
## catkin specific configuration ##
###################################

catkin_package()

catkin_install_python(PROGRAMS scripts/pelco_d_control_node.py
DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
