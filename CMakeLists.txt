cmake_minimum_required(VERSION 2.8.3)
project(audi_a8_can)
add_compile_options(-std=c++11)
set(CPLUSPLUS_VER "c++11")
#调试
# SET(CMAKE_BUILD_TYPE "Debug")
# SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g2 -ggdb")  
# SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall") 
# 配置交叉编译环境
SET(CMAKE_SYSTEM_NAME Linux)
SET(CMAKE_C_COMPILER   /opt/linaro-arm-linux-gnueabihf-4.9/bin/arm-linux-gnueabihf-gcc)
SET(CMAKE_CXX_COMPILER /opt/linaro-arm-linux-gnueabihf-4.9/bin/arm-linux-gnueabihf-g++)

# find_package(catkin REQUIRED COMPONENTS
#   roscpp
#   rospy
#   std_msgs
#   roslib
#   autodrive_msgs
#   sensor_msgs
#   visualization_msgs
#   nav_msgs
#   tf
# )

find_package(Eigen3 REQUIRED)
# catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES audi_A8_can
# #  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
#  DEPENDS Eigen
# )
# message ("aaaaaaaaaaaaaaaaa")
# find_package(pthread)
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${PROJECT_SOURCE_DIR}/include/audi_a8_can
  ${PROJECT_SOURCE_DIR}/src/can_matrix_spi
  ${PROJECT_SOURCE_DIR}/src
  # /usr/include 
)
# message ("aaaaaaaaaaaaaaaaa")
add_executable(audi_a8_adapter
  src/test_audi_a8_can.cpp
  src/audi_a8_control.cpp
  src/audi_a8_msgs_processor.cpp

  src/can_adapters/can_base.cpp
  src/can_adapters/CANTrans.cpp
  src/can_adapters/CANContainer.cpp
  src/can_adapters/can_signal.cpp
  src/can_adapters/can_translator.cpp
  
  src/can_matrix_spi/recv.cpp
  src/can_matrix_spi/send.cpp
  src/can_matrix_spi/service_handler_module.cpp
  src/can_matrix_spi/spi_dbus_hal_slave.cpp
  src/can_matrix_spi/spi_dbus_hal.cpp  
)
# message ("aaaaaaaaaaaaaaaaa")
target_link_libraries(audi_a8_adapter 
# ${catkin_LIBRARIES} ${PROJECT_SOURCE_DIR}/src/lib/libcontrolcan.so  
pthread )
# add_dependencies(audi_a8_adapter autodrive_msgs_generate_messages_cpp)
