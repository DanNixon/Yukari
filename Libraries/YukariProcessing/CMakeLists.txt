project(YukariProcessing)

set(HEADERS
  CloudOperations.h
  FrameProcessingTaskFactory.h
  IFrameProcessingTask.h
  ITaskAlignment.h
  ITaskIncrementalAlignment.h
  ITaskWorldAlignment.h
  PairRegistrationPointRepresentation.h
  TaskAppendTransformedClouds.h
  TaskDownsampleCloud.h
  TaskICPIncrementalAlignment.h
  TaskICPWorldAlignment.h
  TaskNDTIncrementalAlignment.h
  TaskNDTWorldAlignment.h
  TaskPairIncrementalAlignment.h
  TaskPairWorldAlignment.h
  TaskSaveRawCloud.h
  TaskSaveRawIMUFrame.h)

set(SOURCES
  FrameProcessingTaskFactory.cpp
  IFrameProcessingTask.cpp
  ITaskAlignment.cpp
  ITaskIncrementalAlignment.cpp
  ITaskWorldAlignment.cpp
  TaskAppendTransformedClouds.cpp
  TaskDownsampleCloud.cpp
  TaskICPIncrementalAlignment.cpp
  TaskICPWorldAlignment.cpp
  TaskNDTIncrementalAlignment.cpp
  TaskNDTWorldAlignment.cpp
  TaskPairIncrementalAlignment.cpp
  TaskPairWorldAlignment.cpp
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
