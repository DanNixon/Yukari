project(YukariCommon)

add_library(${PROJECT_NAME}
  ${CMAKE_CURRENT_SOURCE_DIR}/ConfigurationManager.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/LoggingService.cpp)

target_link_libraries(${PROJECT_NAME} LINK_PUBLIC
  ${Boost_LIBRARIES})
