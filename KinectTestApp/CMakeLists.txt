cmake_minimum_required ( VERSION 2.8 )

project ( KinectTestApp )

find_package ( PCL 1.7 REQUIRED )

include_directories ( ${PCL_INCLUDE_DIRS} )
link_directories ( ${PCL_LIBRARY_DIRS} )
add_definitions ( ${PCL_DEFINITIONS} )

add_executable (
  ${PROJECT_NAME}
  ${CMAKE_CURRENT_SOURCE_DIR}/main.cpp )

target_link_libraries (
  ${PROJECT_NAME}
  LINK_PUBLIC
  YukariCommon
  ${PCL_LIBRARIES} )
