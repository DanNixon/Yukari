project(YukariCaptureApp)

set(HEADERS
  CaptureController.h
  CaptureFactory.h
  ITrigger.h
  SignalTrigger.h
  TimelapseCaptureTrigger.h)

set(SOURCES
  CaptureController.cpp
  CaptureFactory.cpp
  SignalTrigger.cpp
  TimelapseCaptureTrigger.cpp
  main.cpp)

add_executable(${PROJECT_NAME} ${HEADERS} ${SOURCES})

target_link_libraries(${PROJECT_NAME}
  YukariCommon
  YukariMaths)

set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER Executables)
