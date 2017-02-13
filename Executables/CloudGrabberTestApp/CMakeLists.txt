project(CloudGrabberTestApp)

add_executable(${PROJECT_NAME}
  main.cpp
  CloudGrabberVisualisation.cpp)

target_link_libraries(${PROJECT_NAME}
  YukariCommon
  YukariCloudCapture
  ${PCL_LIBRARIES})

set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER Executables)
