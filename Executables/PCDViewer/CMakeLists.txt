project(PCDViewer)

set(HEADERS)

set(SOURCES
  main.cpp)

add_executable(${PROJECT_NAME} ${HEADERS} ${SOURCES})

target_link_libraries(${PROJECT_NAME}
  ${PCL_LIBRARIES})

set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER Executables)
