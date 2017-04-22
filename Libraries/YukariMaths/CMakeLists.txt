project(YukariMaths)

set(HEADERS
  BaseMathType.h
  Quaternion.h
  Transform.h
  Units.h
  Vector3.h)

set(SOURCES
  Quaternion.cpp
  Transform.cpp
  Vector3.cpp)

set(LIB_DEPS
  YukariCommon)

set(TESTS
  test/QuaternionTest.cpp
  test/TransformTest.cpp
  test/Vector3Test.cpp)

set(TEST_DEPS
  ${PROJECT_NAME}
  ${LIB_DEPS})

add_library(${PROJECT_NAME} ${HEADERS} ${SOURCES})
target_link_libraries(${PROJECT_NAME} ${LIB_DEPS})

include(AddTests)
AddTests(${PROJECT_NAME} TESTS TEST_DEPS)

set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER Libraries)
