# Inspired by https://github.com/RethinkRobotics-opensource/rosnodejs_examples/blob/kinetic-devel/CMakeLists.txt

cmake_minimum_required(VERSION 2.8.3)
project(electron_webrtc)

find_package(catkin REQUIRED)

catkin_package()

install(PROGRAMS
   scripts/start_node.bash
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

