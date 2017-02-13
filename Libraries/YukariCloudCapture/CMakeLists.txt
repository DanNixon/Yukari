project(YukariCloudCapture)

add_library(${PROJECT_NAME}
  ${CMAKE_CURRENT_SOURCE_DIR}/OpenNI2CloudGrabber.cpp)

target_link_libraries(${PROJECT_NAME}
  YukariCommon
  ${PCL_LIBRARIES})

set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER Libraries)
