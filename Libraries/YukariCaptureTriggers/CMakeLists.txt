project(YukariCaptureTriggers)

set(HEADERS
  CLITrigger.h
  ITrigger.h
  MSPAUXTrigger.h
  SignalTrigger.h
  TimelapseCaptureTrigger.h)

set(SOURCES
  CLITrigger.cpp
  MSPAUXTrigger.cpp
  SignalTrigger.cpp
  TimelapseCaptureTrigger.cpp)

set(LIB_DEPS
  YukariCLI
  YukariCommon
  YukariIMU
  serial)

add_library(${PROJECT_NAME} ${HEADERS} ${SOURCES})
target_link_libraries(${PROJECT_NAME} ${LIB_DEPS})

set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER Libraries)
