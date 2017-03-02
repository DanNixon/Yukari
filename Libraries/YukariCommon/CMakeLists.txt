project(YukariCommon)

set(HEADERS
  ConfigurationManager.h
  LoggingService.h
  SignalBroker.h
  StringValueConversion.h)

set(SOURCES
  ConfigurationManager.cpp
  LoggingService.cpp
  SignalBroker.cpp
  StringValueConversion.cpp)

set(LIB_DEPS
  ${Boost_LIBRARIES})

set(TESTS
  test/StringValueConversionTest.cpp)

add_library(${PROJECT_NAME} ${HEADERS} ${SOURCES})
target_link_libraries(${PROJECT_NAME} ${LIB_DEPS})

include(AddTests)
AddTests(${PROJECT_NAME} TESTS LIB_DEPS)

set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER Libraries)
