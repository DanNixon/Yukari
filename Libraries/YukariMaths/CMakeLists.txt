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

add_library(${PROJECT_NAME} ${HEADERS} ${SOURCES})

include(AddTests)
AddTests(${PROJECT_NAME})

set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER Libraries)
