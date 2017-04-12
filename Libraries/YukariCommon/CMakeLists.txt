project(YukariCommon)

set(HEADERS
  LoggingService.h
  NamedDistLogSink.h
  SignalBroker.h)

set(SOURCES
  LoggingService.cpp
  SignalBroker.cpp)

set(LIB_DEPS
  ${Boost_LIBRARIES})

add_library(${PROJECT_NAME} ${HEADERS} ${SOURCES})
target_link_libraries(${PROJECT_NAME} ${LIB_DEPS})

set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER Libraries)
