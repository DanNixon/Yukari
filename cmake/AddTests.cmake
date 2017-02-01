find_package ( Boost REQUIRED COMPONENTS unit_test_framework )

function ( AddTests _libs )
  include_directories ( ${Boost_INCLUDE_DIRS} )

  set ( test_dir "${CMAKE_CURRENT_SOURCE_DIR}/test")

  # Find test files
  file ( GLOB test_files
         RELATIVE "${test_dir}"
         "${test_dir}/*.cpp")


  # Generate test suites
  foreach ( test_file ${test_files} )
    get_filename_component ( test_name ${test_file} NAME_WE )

    add_executable ( ${test_name} "${test_dir}/${test_file}" )
    target_link_libraries ( ${test_name} ${Boost_LIBRARIES} ${_libs} )

    set_target_properties ( ${test_name} PROPERTIES
                            RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/test )

    add_test ( NAME ${test_name}
               WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/test
               COMMAND ${CMAKE_BINARY_DIR}/test/${test_name} )
  endforeach ( test_file )
endfunction ( AddTests )
