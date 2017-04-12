project(YukariCloudCapture)

set(HEADERS
  CloudGrabberFactory.h
  DummyCloudGrabber.h
  ICloudGrabber.h
  OpenNI2CloudGrabber.h
  PCLCloudGrabberWrapper.h)

set(SOURCES)

set(LIB_DEPS
  YukariCommon
  ${PCL_LIBRARIES})

add_library(${PROJECT_NAME} ${HEADERS} ${SOURCES})
target_link_libraries(${PROJECT_NAME} ${LIB_DEPS})

set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER Libraries)
