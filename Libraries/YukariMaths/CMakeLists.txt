project(YukariMaths)

set(HEADERS
  Transform.h
  Units.h)

set(SOURCES
  Transform.cpp)

set(LIB_DEPS
  YukariCommon)

set(TESTS
  test/QuaternionTest.cpp
  test/TransformTest.cpp)

set(TEST_DEPS
  ${PROJECT_NAME}
  ${LIB_DEPS})

add_library(${PROJECT_NAME} ${HEADERS} ${SOURCES})
target_link_libraries(${PROJECT_NAME} ${LIB_DEPS})

include(AddTests)
AddTests(${PROJECT_NAME} TESTS TEST_DEPS)

set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER Libraries)
