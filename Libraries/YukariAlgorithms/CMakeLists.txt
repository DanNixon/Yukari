project(YukariAlgorithms)

set(HEADERS
  ApplyTransformationToCloud.h
  ConcatenatePointClouds.h
  IMUFrameToEigenTransformation.h
  LoadPointCloud.h
  TriangulatePointCloud.h)

set(SOURCES
  ApplyTransformationToCloud.cpp
  ConcatenatePointClouds.cpp
  IMUFrameToEigenTransformation.cpp
  LoadPointCloud.cpp
  TriangulatePointCloud.cpp)

set(LIB_DEPS
  YukariCommon
  YukariProcessing
  ${PCL_LIBRARIES})

set(TESTS
  test/ApplyTransformationToCloudTest.cpp
  test/ConcatenatePointCloudsTest.cpp
  test/IMUFrameToEigenTransformationTest.cpp
  test/LoadPointCloudCloudTest.cpp
  test/TriangulatePointCloudTest.cpp)

add_library(${PROJECT_NAME} ${HEADERS} ${SOURCES})
target_link_libraries(${PROJECT_NAME} ${LIB_DEPS})

include(AddTests)
AddTests(${PROJECT_NAME} TESTS LIB_DEPS)

set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER Libraries)
