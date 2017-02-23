project(YukariCLI)

set(HEADERS
  CLI.h
  Command.h
  CommandContainer.h
  SubCommand.h)

set(SOURCES
  CLI.cpp
  CommandContainer.cpp)

set(LIB_DEPS
  YukariCommon)

set(TESTS
  test/CLITest.cpp)

add_library(${PROJECT_NAME} ${HEADERS} ${SOURCES})
target_link_libraries(${PROJECT_NAME} ${LIB_DEPS})

include(AddTests)
AddTests(${PROJECT_NAME} TESTS LIB_DEPS)

set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER Libraries)
