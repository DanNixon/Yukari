project(PointCloudAlignment)

set(HEADERS)

set(SOURCES
  main.cpp)

add_executable(${PROJECT_NAME} ${HEADERS} ${SOURCES})

target_link_libraries(${PROJECT_NAME}
  YukariCommon)

set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER Executables/Processing)
