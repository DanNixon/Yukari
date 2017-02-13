project(YukariProcessing)

add_library(${PROJECT_NAME}
  ${CMAKE_CURRENT_SOURCE_DIR}/IAlgorithm.cpp)

target_link_libraries(${PROJECT_NAME}
  YukariCommon
  YukariIMU
  ${PCL_LIBRARIES})

set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER Libraries)
