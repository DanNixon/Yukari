function(AddTests PROJECT_UNDER_TEST TEST_FILES TEST_LIB_DEPS)
  foreach(test_file ${${TEST_FILES}})
    get_filename_component(test_name "${test_file}" NAME_WE)

    add_executable(${test_name} "${test_file}")
    target_link_libraries(${test_name} ${Boost_LIBRARIES} ${PROJECT_UNDER_TEST} ${${TEST_LIB_DEPS}})

    set_target_properties(${test_name} PROPERTIES
      RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/test)

    add_test(NAME ${test_name}
      WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/test
      COMMAND ${CMAKE_BINARY_DIR}/test/${test_name})

    set_target_properties(${test_name} PROPERTIES FOLDER Tests)
  endforeach(test_file)
endfunction(AddTests)
