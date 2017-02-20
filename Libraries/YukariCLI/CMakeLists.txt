project(YukariCLI)

set(HEADERS
  CLI.h
  Command.h
  CommandContainer.h
  SubCommand.h)

set(SOURCES
  CLI.cpp
  CommandContainer.cpp)

add_library(${PROJECT_NAME} ${HEADERS} ${SOURCES})

target_link_libraries(${PROJECT_NAME}
  YukariCommon)

include(AddTests)
AddTests(${PROJECT_NAME})

set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER Libraries)
