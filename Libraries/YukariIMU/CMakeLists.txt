project(YukariIMU)

set(HEADERS
  DummyIMUGrabber.h
  IIMUGrabber.h
  IMUConstants.h
  IMUFrame.h
  MSPGrabber.h
  MSPGrabberIMU.h
  MSPGrabberAttitude.h
  MSPClient.h)

set(SOURCES
  DummyIMUGrabber.cpp
  IMUFrame.cpp
  MSPGrabber.cpp
  MSPGrabberIMU.cpp
  MSPGrabberAttitude.cpp
  MSPClient.cpp)

set(LIB_DEPS
  YukariMaths
  serial)

set(TESTS
  test/IMUFrameTest.cpp
  test/MSPClientTest.cpp)

add_library(${PROJECT_NAME} ${HEADERS} ${SOURCES})
target_link_libraries(${PROJECT_NAME} ${LIB_DEPS})

include(AddTests)
AddTests(${PROJECT_NAME} TESTS LIB_DEPS)

set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER Libraries)
