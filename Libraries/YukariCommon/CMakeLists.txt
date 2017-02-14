project(YukariCommon)

set(HEADERS
  ConfigurationManager.h
  LoggingService.h)

set(SOURCES
  ConfigurationManager.cpp
  LoggingService.cpp)

add_library(${PROJECT_NAME} ${HEADERS} ${SOURCES})

target_link_libraries(${PROJECT_NAME}
  ${Boost_LIBRARIES})

set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER Libraries)
