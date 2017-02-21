project(YukariCaptureApp)

set(HEADERS
  CaptureCLI.h
  CaptureController.h
  CaptureFactory.h
  CLITrigger.h
  ITrigger.h
  SignalTrigger.h
  TimelapseCaptureTrigger.h)

set(SOURCES
  CaptureCLI.cpp
  CaptureController.cpp
  CaptureFactory.cpp
  CLITrigger.cpp
  SignalTrigger.cpp
  TimelapseCaptureTrigger.cpp
  main.cpp)

add_executable(${PROJECT_NAME} ${HEADERS} ${SOURCES})

target_link_libraries(${PROJECT_NAME}
  YukariCLI
  YukariCloudCapture
  YukariCommon
  YukariIMU
  YukariMaths
  serial
  ${PCL_LIBRARIES})

set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER Executables)
