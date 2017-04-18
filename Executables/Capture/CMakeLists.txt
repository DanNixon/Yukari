project(Capture)

set(HEADERS
  CaptureController.h
  CaptureFactory.h
  IPostCaptureTask.h
  TaskSaveRawCloud.h
  TaskSaveRawIMUFrame.h
  Types.h)

set(SOURCES
  CaptureController.cpp
  CaptureFactory.cpp
  TaskSaveRawCloud.cpp
  TaskSaveRawIMUFrame.cpp
  main.cpp)

add_executable(${PROJECT_NAME} ${HEADERS} ${SOURCES})

target_link_libraries(${PROJECT_NAME}
  YukariCommon
  YukariIMU
  YukariMaths
  YukariMSP
  YukariTriggers
  serial
  ${PCL_LIBRARIES})

set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER Executables)
