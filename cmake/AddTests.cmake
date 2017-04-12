function(AddTests PROJECT_UNDER_TEST TEST_FILES TEST_LIB_DEPS)
  set(test_name "${PROJECT_UNDER_TEST}Test")

  add_executable(${test_name} test/main.cpp ${${TEST_FILES}})

  target_link_libraries(${test_name} ${Boost_LIBRARIES} ${${TEST_LIB_DEPS}})

  set_target_properties(${test_name} PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/test)

  add_test(NAME ${test_name}
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/test
    COMMAND ${CMAKE_BINARY_DIR}/test/${test_name})

  set_target_properties(${test_name} PROPERTIES FOLDER Tests)
endfunction(AddTests)
