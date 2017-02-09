project(IMUDeviceTestApp)

add_executable(${PROJECT_NAME}
  ${CMAKE_CURRENT_SOURCE_DIR}/main.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/VTKIMUActorCallback.cpp)

target_link_libraries(${PROJECT_NAME} LINK_PUBLIC
  YukariCommon
  YukariIMU
  serial
  ${VTK_LIBRARIES})
