project(YukariProcessingApp)

set(HEADERS
  ProcessingCLI.h)

set(SOURCES
  main.cpp
  ProcessingCLI.cpp)

add_executable(${PROJECT_NAME} ${HEADERS} ${SOURCES})

target_link_libraries(${PROJECT_NAME}
  YukariAlgorithms
  YukariCLI
  YukariCloudCapture
  YukariCommon
  YukariIMU
  YukariMaths
  YukariProcessing
  ${PCL_LIBRARIES})

set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER Executables)
