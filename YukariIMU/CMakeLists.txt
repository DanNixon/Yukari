cmake_minimum_required ( VERSION 2.8 )

project ( YukariIMU )

add_library (
  ${PROJECT_NAME}
  ${CMAKE_CURRENT_SOURCE_DIR}/IMUFrame.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/ExtendedIMUFrame.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/MSPGrabber.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/MSPGrabberIMU.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/MSPGrabberAttitude.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/MSPClient.cpp )

target_link_libraries (
  ${PROJECT_NAME}
  LINK_PUBLIC
  YukariMaths
  serial )

include ( AddTests )
AddTests ( ${PROJECT_NAME}
           YukariMaths
           serial )
