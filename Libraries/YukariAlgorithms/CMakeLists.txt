project(YukariAlgorithms)

set(HEADERS
  AlgorithmFactory.h
  ApplyTransformationToCloud.h
  ConcatenatePointClouds.h
  GenerateMeshFromPointCloud.h
  IMUFrameToEigenTransformation.h
  LoadPointCloud.h
  RemoveNaNFromPointCloud.h
  SaveVTKMesh.h
  TriangulatePointCloud.h)

set(SOURCES
  AlgorithmFactory.cpp
  ApplyTransformationToCloud.cpp
  ConcatenatePointClouds.cpp
  GenerateMeshFromPointCloud.cpp
  IMUFrameToEigenTransformation.cpp
  LoadPointCloud.cpp
  RemoveNaNFromPointCloud.cpp
  SaveVTKMesh.cpp
  TriangulatePointCloud.cpp)

set(LIB_DEPS
  YukariCommon
  YukariProcessing
  ${PCL_LIBRARIES})

set(TESTS
  test/ApplyTransformationToCloudTest.cpp
  test/ConcatenatePointCloudsTest.cpp
  test/GenerateMeshFromPointCloudTest.cpp
  test/IMUFrameToEigenTransformationTest.cpp
  test/LoadPointCloudTest.cpp
  test/RemoveNaNFromPointCloudTest.cpp
  test/SaveVTKMeshTest.cpp
  test/TriangulatePointCloudTest.cpp)

add_library(${PROJECT_NAME} ${HEADERS} ${SOURCES})
target_link_libraries(${PROJECT_NAME} ${LIB_DEPS})

include(AddTests)
AddTests(${PROJECT_NAME} TESTS LIB_DEPS)

set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER Libraries)
