project(YukariIMU)

set(HEADERS
  DummyIMUGrabber.h
  FileIMUGrabber.h
  IIMUGrabber.h
  IMUFrame.h
  IMUGrabberFactory.h
  MSPGrabber.h
  MSPGrabberAttitude.h
  TeensyIMUDevice.h)

set(SOURCES
  DummyIMUGrabber.cpp
  FileIMUGrabber.cpp
  IMUFrame.cpp
  IMUGrabberFactory.cpp
  MSPGrabber.cpp
  MSPGrabberAttitude.cpp
  TeensyIMUDevice.cpp)

set(LIB_DEPS
  YukariCommon
  YukariMaths
  YukariMSP
  serial)

set(TESTS
  test/IMUFrameTest.cpp)

set(TEST_DEPS
  ${PROJECT_NAME}
  ${LIB_DEPS})

add_library(${PROJECT_NAME} ${HEADERS} ${SOURCES})
target_link_libraries(${PROJECT_NAME} ${LIB_DEPS})

include(AddTests)
AddTests(${PROJECT_NAME} TESTS TEST_DEPS)

set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER Libraries)
