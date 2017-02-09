cmake_minimum_required(VERSION 2.8)

project(Yukari)

set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

enable_testing()

# All warnings
include(EnableExtraCompilerWarnings)
globally_enable_extra_compiler_warnings()

include(DoxygenTargets)

set(Boost_NO_BOOST_CMAKE TRUE)
set(Boost_NO_SYSTEM_PATHS TRUE)
set(Boost_DETAILED_FAILURE_MSG ON)

# External library: PCL
find_package(PCL 1.6 REQUIRED)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# External library: Boost
set(Boost_USE_STATIC_LIBS ON)
set(Boost_USE_STATIC_RUNTIME OFF)
set(Boost_USE_MULTITHREADED ON)
find_package(Boost REQUIRED
  COMPONENTS system filesystem unit_test_framework serialization thread date_time iostreams chrono mpi)

if(UNIX)
  list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")
  list(REMOVE_ITEM VTK_LIBRARIES "vtkproj4")
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=gnu++0x -pthread")
endif()

# Includes
include_directories(${CMAKE_CURRENT_SOURCE_DIR}
                    ${CMAKE_CURRENT_SOURCE_DIR}/ThirdParty/boost_qvm
                    ${CMAKE_CURRENT_SOURCE_DIR}/ThirdParty/spdlog
                    ${CMAKE_CURRENT_SOURCE_DIR}/ThirdParty/serial/include
                    ${Boost_INCLUDE_DIRS}
                    ${VTK_INCLUDE_DIRS}
                    ${PCL_INCLUDE_DIRS})

# Third party libraries
add_subdirectory(ThirdParty/serial)

# Libraries
add_subdirectory(YukariCommon)
add_subdirectory(YukariMaths)
add_subdirectory(YukariIMU)

# Executables
add_subdirectory(YukariCaptureApplication)
add_subdirectory(KinectTestApp)
add_subdirectory(IMUDeviceTest)

# Docs
add_doxygen(
  Doxyfile
  OUTPUT_DIRECTORY docs
  NO_PDF)
