cmake_minimum_required ( VERSION 2.8 )

project ( YukariCommon )

add_library (
  ${PROJECT_NAME}
  ${CMAKE_CURRENT_SOURCE_DIR}/LoggingService.cpp )

#include ( AddTests )
#AddTests ( ${PROJECT_NAME} )
