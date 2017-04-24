project(YukariProcessing)

set(HEADERS
  CloudOperations.h
  FrameProcessingTaskFactory.h
  GenerateMeshFromPointCloud.h
  IFrameProcessingTask.h
  SpatialOperations.h
  TaskAppendTransformedClouds.h
  TaskNDTIncrementalAlignment.h
  TaskNDTWorldAlignment.h
  TaskNDTWorldSegmentAlignment.h
  TaskSaveRawCloud.h
  TaskSaveRawIMUFrame.h)

set(SOURCES)

set(LIB_DEPS
  YukariCommon
  YukariIMU
  ${PCL_LIBRARIES})

set(TESTS
  test/GenerateMeshFromPointCloudTest.cpp
  test/IFrameProcessingTaskTest.cpp)

add_library(${PROJECT_NAME} ${HEADERS} ${SOURCES})
target_link_libraries(${PROJECT_NAME} ${LIB_DEPS})

include(AddTests)
AddTests(${PROJECT_NAME} TESTS LIB_DEPS)

set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER Libraries)
