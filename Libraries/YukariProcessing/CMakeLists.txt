project(YukariProcessing)

set(HEADERS
  CloudOperations.h
  FrameProcessingTaskFactory.h
  IFrameProcessingTask.h
  ITaskAlignment.h
  TaskAppendTransformedClouds.h
  TaskDownsampleCloud.h
  TaskICPWorldAlignment.h
  TaskNDTIncrementalAlignment.h
  TaskNDTWorldAlignment.h
  TaskNDTWorldSegmentAlignment.h
  TaskPairAlignment.h
  TaskSaveRawCloud.h
  TaskSaveRawIMUFrame.h)

set(SOURCES
  FrameProcessingTaskFactory.cpp
  IFrameProcessingTask.cpp
  ITaskAlignment.cpp
  TaskAppendTransformedClouds.cpp
  TaskDownsampleCloud.cpp
  TaskICPWorldAlignment.cpp
  TaskNDTIncrementalAlignment.cpp
  TaskNDTWorldAlignment.cpp
  TaskNDTWorldSegmentAlignment.cpp
  TaskPairAlignment.cpp
  TaskSaveRawCloud.cpp
  TaskSaveRawIMUFrame.cpp)

set(LIB_DEPS
  YukariCommon
  YukariIMU
  ${PCL_LIBRARIES})

set(TESTS
  test/IFrameProcessingTaskTest.cpp
  test/ITaskAlignmentTest.cpp)

set(TEST_DEPS
  ${PROJECT_NAME}
  ${LIB_DEPS})

add_library(${PROJECT_NAME} ${HEADERS} ${SOURCES})
target_link_libraries(${PROJECT_NAME} ${LIB_DEPS})

include(AddTests)
AddTests(${PROJECT_NAME} TESTS TEST_DEPS)

set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER Libraries)
