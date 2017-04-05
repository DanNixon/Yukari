project(CapturePointClouds)

set(HEADERS
  CaptureCLI.h
  CaptureController.h
  CaptureFactory.h)

set(SOURCES
  CaptureCLI.cpp
  CaptureController.cpp
  CaptureFactory.cpp
  main.cpp)

add_executable(${PROJECT_NAME} ${HEADERS} ${SOURCES})

target_link_libraries(${PROJECT_NAME}
  YukariCaptureTriggers
  YukariCLI
  YukariCloudCapture
  YukariCommon
  YukariIMU
  YukariMaths
  YukariMSP
  serial
  ${PCL_LIBRARIES})

set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER Executables)
