cmake_minimum_required ( VERSION 2.8 )

project ( YukariCommon )

set ( Boost_USE_STATIC_LIBS ON )
set ( Boost_USE_STATIC_RUNTIME ON )
set ( Boost_USE_MULTITHREADED ON )
find_package ( Boost REQUIRED COMPONENTS system filesystem )

add_library (
  ${PROJECT_NAME}
  ${CMAKE_CURRENT_SOURCE_DIR}/ConfigurationManager.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/LoggingService.cpp )

target_link_libraries (
  ${PROJECT_NAME}
  LINK_PUBLIC
  ${Boost_LIBRARIES} )
