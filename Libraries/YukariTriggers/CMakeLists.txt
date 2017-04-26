project(YukariTriggers)

set(HEADERS
  ITrigger.h
  PeriodicTrigger.h
  ProxyTrigger.h
  SignalTrigger.h
  TriggerFactory.h)

set(SOURCES
  PeriodicTrigger.cpp
  ProxyTrigger.cpp
  SignalTrigger.cpp
  TriggerFactory.cpp)

set(LIB_DEPS
  YukariCommon)

add_library(${PROJECT_NAME} ${HEADERS} ${SOURCES})
target_link_libraries(${PROJECT_NAME} ${LIB_DEPS})

set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER Libraries)
