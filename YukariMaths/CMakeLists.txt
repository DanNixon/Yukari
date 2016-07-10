cmake_minimum_required ( VERSION 2.8 )

project ( YukariMaths )

add_library (
  ${PROJECT_NAME}
  ${CMAKE_CURRENT_SOURCE_DIR}/Matrix4.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/Quat.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/Vector.cpp )

include ( AddTests )
AddTests ( ${PROJECT_NAME} )
