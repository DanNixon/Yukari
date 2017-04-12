project(YukariMaths)

set(HEADERS
  BaseMathType.h
  Quaternion.h
  Units.h
  Vector3.h
  Vector4.h)

set(SOURCES
  Quaternion.cpp
  Vector3.cpp
  Vector4.cpp)

set(TESTS
  test/QuaternionTest.cpp
  test/Vector3Test.cpp
  test/Vector4Test.cpp)

set(TEST_DEPS
  ${PROJECT_NAME})

add_library(${PROJECT_NAME} ${HEADERS} ${SOURCES})

include(AddTests)
AddTests(${PROJECT_NAME} TESTS TEST_DEPS)

set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER Libraries)
