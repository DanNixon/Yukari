project(YukariCommon)

set(HEADERS
  ConfigurationManager.h
  LoggingService.h
  SignalBroker.h)

set(SOURCES
  ConfigurationManager.cpp
  LoggingService.cpp
  SignalBroker.cpp)

add_library(${PROJECT_NAME} ${HEADERS} ${SOURCES})

target_link_libraries(${PROJECT_NAME}
  ${Boost_LIBRARIES})

set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER Libraries)
