project(YukariCloudCapture)

set(HEADERS
  DummyCloudGrabber.h
  ICloudGrabber.h
  OpenNI2CloudGrabber.h)

set(SOURCES
  DummyCloudGrabber.cpp
  OpenNI2CloudGrabber.cpp)

add_library(${PROJECT_NAME} ${HEADERS} ${SOURCES})

target_link_libraries(${PROJECT_NAME}
  YukariCommon
  ${PCL_LIBRARIES})

set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER Libraries)
