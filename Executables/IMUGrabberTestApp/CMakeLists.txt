project(IMUGrabberTestApp)

set(HEADERS
  VTKIMUActorCallback.h
  VTKIMUCalibrationInteractionStyle.h)

set(SOURCES
  VTKIMUActorCallback.cpp
  VTKIMUCalibrationInteractionStyle.cpp
  main.cpp)

add_executable(${PROJECT_NAME} ${HEADERS} ${SOURCES})

target_link_libraries(${PROJECT_NAME}
  YukariCommon
  YukariIMU
  serial
  ${VTK_LIBRARIES})

set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER Executables)
