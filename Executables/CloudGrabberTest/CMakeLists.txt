project(CloudGrabberTest)

set(HEADERS
  CloudGrabberVisualisation.h)

set(SOURCES
  CloudGrabberVisualisation.cpp
  main.cpp)

add_executable(${PROJECT_NAME} ${HEADERS} ${SOURCES})

target_link_libraries(${PROJECT_NAME}
  YukariCommon
  YukariCloudCapture
  ${PCL_LIBRARIES})

set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER Executables)
