project(YukariIMU)

set(HEADERS
  DummyIMUGrabber.h
  IIMUGrabber.h
  IMUConstants.h
  IMUFrame.h
  ExtendedIMUFrame.h
  MSPGrabber.h
  MSPGrabberIMU.h
  MSPGrabberAttitude.h
  MSPClient.h)

set(SOURCES
  DummyIMUGrabber.cpp
  IMUFrame.cpp
  ExtendedIMUFrame.cpp
  MSPGrabber.cpp
  MSPGrabberIMU.cpp
  MSPGrabberAttitude.cpp
  MSPClient.cpp)

add_library(${PROJECT_NAME} ${HEADERS} ${SOURCES})

target_link_libraries(${PROJECT_NAME}
  YukariMaths
  serial)

include(AddTests)
AddTests(${PROJECT_NAME}
  YukariMaths
  serial)

set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER Libraries)
