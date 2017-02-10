project(YukariCloudCapture)

add_library(${PROJECT_NAME})

target_link_libraries(${PROJECT_NAME}
  YukariCommon
  ${PCL_LIBRARIES})

set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER Libraries)