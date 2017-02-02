cmake_minimum_required ( VERSION 2.8 )

project ( Yukari )

set ( CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

set ( CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib )
set ( CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib )
set ( CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin )

enable_testing ()

if(UNIX)
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=gnu++0x -pthread")
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

# External library: Boost
set ( Boost_USE_STATIC_LIBS ON )
set ( Boost_USE_STATIC_RUNTIME OFF )
set ( Boost_USE_MULTITHREADED ON )
find_package ( Boost REQUIRED
               COMPONENTS system filesystem unit_test_framework )

# Includes
include_directories ( ${CMAKE_CURRENT_SOURCE_DIR}
                      ${CMAKE_CURRENT_SOURCE_DIR}/ThirdParty
					  ${Boost_INCLUDE_DIRS} )

# Libraries
add_subdirectory ( YukariCommon )
add_subdirectory ( YukariMaths )
add_subdirectory ( YukariIMU )

# Executables
add_subdirectory ( YukariCaptureApplication )

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
