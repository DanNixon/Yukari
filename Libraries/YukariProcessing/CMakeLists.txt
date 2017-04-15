project(YukariProcessing)

set(HEADERS
  CloudOperations.h
  GenerateMeshFromPointCloud.h
  SpatialOperations.h)

set(SOURCES)

set(LIB_DEPS
  YukariCommon
  YukariIMU
  ${PCL_LIBRARIES})

set(TESTS
  test/CloudOperationsTest.cpp
  test/GenerateMeshFromPointCloudTest.cpp)

add_library(${PROJECT_NAME} ${HEADERS} ${SOURCES})
target_link_libraries(${PROJECT_NAME} ${LIB_DEPS})

include(AddTests)
AddTests(${PROJECT_NAME} TESTS LIB_DEPS)

set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER Libraries)
