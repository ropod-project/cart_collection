cmake_minimum_required(VERSION 2.8.3)
project(cart_collection)

find_package(catkin REQUIRED
  COMPONENTS
    roslint
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
    rospy
    smach
    smach_ros
)

### TESTS
roslint_python()

