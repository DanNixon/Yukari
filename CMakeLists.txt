cmake_minimum_required ( VERSION 2.8 )

project ( Yukari )

set ( CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

set ( CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib )
set ( CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib )
set ( CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin )

enable_testing ()

# C++11
if(UNIX)
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=gnu++0x")
endif()

# Lcov
find_program ( LCOV_PATH lcov )
if ( LCOV_PATH )
  include ( CodeCoverage )
  SET ( GCC_COVERAGE_COMPILE_FLAGS "-fprofile-arcs -ftest-coverage" )
  SET ( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GCC_COVERAGE_COMPILE_FLAGS}" )
endif ( LCOV_PATH )

# All warnings
include ( EnableExtraCompilerWarnings )
globally_enable_extra_compiler_warnings()

include ( DoxygenTargets )

# Includes
include_directories ( ${CMAKE_CURRENT_SOURCE_DIR} )

# Libraries
add_subdirectory ( YukariMaths )

# Generate test coverage report
if ( LCOV_PATH )
  SETUP_TARGET_FOR_COVERAGE(
    coverage
    ctest
    coverage )
endif ( LCOV_PATH )

# Docs
add_doxygen(
  Doxyfile
  OUTPUT_DIRECTORY docs
  NO_PDF )
