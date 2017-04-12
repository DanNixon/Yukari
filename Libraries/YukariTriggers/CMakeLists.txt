project(YukariTriggers)

set(HEADERS
  CLITrigger.h
  ITrigger.h
  PeriodicTrigger.h
  SignalTrigger.h
  TriggerFactory.h)

set(SOURCES
  CLITrigger.cpp
  PeriodicTrigger.cpp
  SignalTrigger.cpp
  TriggerFactory.cpp)

set(LIB_DEPS
  YukariCLI
  YukariCommon)

add_library(${PROJECT_NAME} ${HEADERS} ${SOURCES})
target_link_libraries(${PROJECT_NAME} ${LIB_DEPS})

set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER Libraries)
