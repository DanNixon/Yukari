cmake_minimum_required ( VERSION 2.8 )

project ( YukariCaptureApplication )

add_executable (
  ${PROJECT_NAME}
  ${CMAKE_CURRENT_SOURCE_DIR}/main.cpp )

target_link_libraries (
  ${PROJECT_NAME}
  LINK_PUBLIC
  YukariCommon )
