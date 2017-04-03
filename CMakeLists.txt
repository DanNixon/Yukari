cmake_minimum_required(VERSION 2.8)

project(Yukari)

set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

enable_testing()

set_property(GLOBAL PROPERTY USE_FOLDERS ON)

# No min/max macros from MSVC
add_definitions(-DNOMINMAX)

# All warnings
include(EnableExtraCompilerWarnings)
globally_enable_extra_compiler_warnings()

include(DoxygenTargets)

set(Boost_NO_BOOST_CMAKE TRUE)
set(Boost_NO_SYSTEM_PATHS TRUE)
set(Boost_DETAILED_FAILURE_MSG ON)

# External library: PCL
set(BOOST_INCLUDEDIR "${Boost_INCLUDE_DIR}")
find_package(PCL 1.6 REQUIRED)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# External library: Boost
set(Boost_USE_STATIC_LIBS ON)
set(Boost_USE_STATIC_RUNTIME OFF)
set(Boost_USE_MULTITHREADED ON)
find_package(Boost REQUIRED
  COMPONENTS system filesystem unit_test_framework serialization thread date_time iostreams chrono program_options regex)

if(UNIX)
  # This fixed an issue, I seem to recall it was a runetime issue
  list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")
  list(REMOVE_ITEM VTK_LIBRARIES "vtkproj4")

  # Fixes issues with PCL/boost on Ubuntu 14.04 when compiled in Debug mode
  set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O1")

  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=gnu++0x -pthread")
endif()

# Includes
include_directories(Libraries
  ThirdParty/boost_qvm
  ThirdParty/serial/include
  ThirdParty/spdlog
  ${Boost_INCLUDE_DIRS}
  ${VTK_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS})

# Third party libraries
add_subdirectory(ThirdParty/serial)

# Libraries
add_subdirectory(Libraries/YukariAlgorithms)
add_subdirectory(Libraries/YukariCLI)
add_subdirectory(Libraries/YukariCloudCapture)
add_subdirectory(Libraries/YukariCommon)
add_subdirectory(Libraries/YukariIMU)
add_subdirectory(Libraries/YukariMaths)
add_subdirectory(Libraries/YukariProcessing)

# Executables
add_subdirectory(Executables/CapturePointClouds)
add_subdirectory(Executables/CloudGrabberTest)
add_subdirectory(Executables/IMUGrabberTest)
add_subdirectory(Executables/PCDToMesh)
add_subdirectory(Executables/PCDViewer)
add_subdirectory(Executables/PointCloudAlignment)

# Docs
add_doxygen(
  Doxyfile
  OUTPUT_DIRECTORY docs
  NO_PDF)
