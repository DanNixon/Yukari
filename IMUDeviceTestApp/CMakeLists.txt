project(IMUDeviceTestApp)

add_executable(${PROJECT_NAME}
  ${CMAKE_CURRENT_SOURCE_DIR}/main.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/VTKIMUActorCallback.cpp)

target_link_libraries(${PROJECT_NAME}
  YukariCommon
  YukariIMU
  serial
  ${VTK_LIBRARIES})
  
set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER Executables)