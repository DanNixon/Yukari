project(YukariCloudCapture)

set(HEADERS
  DummyCloudGrabber.h
  ICloudGrabber.h
  OpenNI2CloudGrabber.h)

set(SOURCES
  DummyCloudGrabber.cpp
  OpenNI2CloudGrabber.cpp)

set(LIB_DEPS
  YukariCommon
  ${PCL_LIBRARIES})

add_library(${PROJECT_NAME} ${HEADERS} ${SOURCES})
target_link_libraries(${PROJECT_NAME} ${LIB_DEPS})

set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER Libraries)
