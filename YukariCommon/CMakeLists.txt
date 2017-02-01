cmake_minimum_required ( VERSION 2.8 )

project ( YukariCommon )

set ( Boost_USE_STATIC_LIBS ON )
set ( Boost_USE_MULTITHREADED ON )
set ( Boost_USE_STATIC_RUNTIME ON )
find_package ( Boost REQUIRED COMPONENTS system )

add_library (
  ${PROJECT_NAME}
  ${CMAKE_CURRENT_SOURCE_DIR}/ConfigurationManager.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/LoggingService.cpp )

#include ( AddTests )
#AddTests ( ${PROJECT_NAME} )
