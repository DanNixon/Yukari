project(CloudGrabberTest)

set(HEADERS
  CloudGrabberVisualisation.h)

set(SOURCES
  main.cpp)

add_executable(${PROJECT_NAME} ${HEADERS} ${SOURCES})

target_link_libraries(${PROJECT_NAME}
  YukariCommon
  YukariMaths
  YukariIMU
  YukariTriggers
  serial
  ${PCL_LIBRARIES})

set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER Executables/Test)
