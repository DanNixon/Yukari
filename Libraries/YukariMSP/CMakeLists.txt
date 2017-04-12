project(YukariMSP)

set(HEADERS
  MSPClient.h
  MSPParsers.h)

set(SOURCES
  MSPClient.cpp
  MSPParsers.cpp)

set(LIB_DEPS
  YukariCommon
  YukariMaths
  serial)

set(TESTS
  test/MSPClientTest.cpp
  test/MSPParsersTest.cpp)

set(TEST_DEPS
  ${PROJECT_NAME}
  ${LIB_DEPS})

add_library(${PROJECT_NAME} ${HEADERS} ${SOURCES})
target_link_libraries(${PROJECT_NAME} ${LIB_DEPS})

include(AddTests)
AddTests(${PROJECT_NAME} TESTS TEST_DEPS)

set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER Libraries)
