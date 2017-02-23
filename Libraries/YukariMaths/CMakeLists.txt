project(YukariMaths)

set(HEADERS
  BaseMathType.h
  Matrix3.h
  Matrix4.h
  Quaternion.h
  Units.h
  Vector3.h
  Vector4.h)

set(SOURCES
  Matrix3.cpp
  Matrix4.cpp
  Quaternion.cpp
  Vector3.cpp
  Vector4.cpp)

set(TESTS
  test/Matrix3Test.cpp
  test/Matrix4Test.cpp
  test/QuaternionTest.cpp
  test/Vector3Test.cpp
  test/Vector4Test.cpp)

add_library(${PROJECT_NAME} ${HEADERS} ${SOURCES})

include(AddTests)
AddTests(${PROJECT_NAME} TESTS "")

set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER Libraries)
