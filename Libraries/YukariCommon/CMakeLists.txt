project(YukariCommon)

set(HEADERS
  FilesystemHelpers.h
  FilteredDirectoryIterator.h
  LoggingService.h
  MapHelpers.h
  NamedDistLogSink.h
  SignalBroker.h
  StringParsers.h)

set(SOURCES
  FilesystemHelpers.cpp
  LoggingService.cpp
  SignalBroker.cpp
  StringParsers.cpp)

set(LIB_DEPS
  ${Boost_LIBRARIES})

set(TESTS
  test/StringParsersTest.cpp
  test/MapHelpersTest.cpp)

set(TEST_DEPS
  ${PROJECT_NAME}
  ${LIB_DEPS})

add_library(${PROJECT_NAME} ${HEADERS} ${SOURCES})
target_link_libraries(${PROJECT_NAME} ${LIB_DEPS})

include(AddTests)
AddTests(${PROJECT_NAME} TESTS TEST_DEPS)

set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER Libraries)
