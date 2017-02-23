project(YukariProcessing)

set(HEADERS
  IAlgorithm.h
  Property.h)

set(SOURCES
  IAlgorithm.cpp
  Property.cpp)

add_library(${PROJECT_NAME} ${HEADERS} ${SOURCES})

target_link_libraries(${PROJECT_NAME}
  YukariCommon
  YukariIMU
  ${PCL_LIBRARIES})

include(AddTests)
AddTests(${PROJECT_NAME})

set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER Libraries)
