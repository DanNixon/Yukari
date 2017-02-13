project(YukariCommon)

add_library(${PROJECT_NAME}
  ${CMAKE_CURRENT_SOURCE_DIR}/ConfigurationManager.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/LoggingService.cpp)

target_link_libraries(${PROJECT_NAME}
  ${Boost_LIBRARIES})

set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER Libraries)