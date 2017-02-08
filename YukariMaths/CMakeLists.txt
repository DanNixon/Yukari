project(YukariMaths)

add_library(${PROJECT_NAME}
  ${CMAKE_CURRENT_SOURCE_DIR}/Matrix3.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/Matrix4.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/Quaternion.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/Vector3.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/Vector4.cpp)

include(AddTests)
AddTests(${PROJECT_NAME})
