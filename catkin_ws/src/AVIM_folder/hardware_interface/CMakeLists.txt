cmake_minimum_required(VERSION 3.0.2)
project(hardware_interface)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs geometry_msgs sensor_msgs
)

# Install Python scripts
catkin_install_python(PROGRAMS
  src/motor_driver.py
  src/lights_controller.py
  src/ultrasonic_sensors.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
