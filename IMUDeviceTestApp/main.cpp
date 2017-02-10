#include <cstdio>
#include <iomanip>
#include <iostream>
#include <string>

#ifdef _WIN32
#include <windows.h>
#else
#include <unistd.h>
#endif

#include <boost/qvm/all.hpp>
#include <serial/serial.h>
#include <vtkActor.h>
#include <vtkAxes.h>
#include <vtkCellArray.h>
#include <vtkCommand.h>
#include <vtkCubeSource.h>
#include <vtkFloatArray.h>
#include <vtkPointData.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkSmartPointer.h>

#include <YukariCommon/LoggingService.h>
#include <YukariIMU/MSPClient.h>
#include <YukariIMU/MSPGrabberAttitude.h>
#include <YukariIMU/MSPGrabberIMU.h>

#include "VTKIMUActorCallback.h"

using namespace boost::qvm;
using namespace Yukari::Common;
using namespace Yukari::IMU;
using namespace Yukari::Maths;

void doSleep(unsigned long milliseconds)
{
#ifdef _WIN32
  Sleep(milliseconds);
#else
  usleep(milliseconds * 1000);
#endif
}

int runRawData(const std::string &portName, unsigned int baud);
int runFrameGrabber(IGrabber *grabber);

int main(int argc, char **argv)
{
  auto logger = LoggingService::GetLogger("main");

  /* List all ports */
  std::vector<serial::PortInfo> ports = serial::list_ports();
  for (auto it = ports.begin(); it != ports.end(); ++it)
    printf("(%s, %s, %s)\n", it->port.c_str(), it->description.c_str(), it->hardware_id.c_str());

  if (argc != 4)
    return 1;

  const std::string portName(argv[2]);

  unsigned long baud = 0;
#if defined(WIN32) && !defined(__MINGW32__)
  sscanf_s(argv[3], "%lu", &baud);
#else
  sscanf(argv[3], "%lu", &baud);
#endif

  const std::string mode(argv[1]);
  if (mode == "raw")
    return runRawData(portName, baud);
  else if (mode == "attitude")
    return runFrameGrabber(new MSPGrabberAttitude(portName, baud));
  else if (mode == "imu")
    return runFrameGrabber(new MSPGrabberIMU(portName, baud));
  else
    return 2;
}

int runRawData(const std::string &portName, unsigned int baud)
{
  auto logger = LoggingService::GetLogger("runRawData");

  serial::Serial port(portName, baud);
  MSPClient msp(port);

  if (port.isOpen())
  {
    logger->info("Port is open.");
  }
  else
  {
    logger->error("Port failed to open.");
    return 2;
  }

  logger->info("Wait for board to wake...");
  doSleep(1000);
  logger->info("ok.");

  int16_t gyro[3];
  int16_t acc[3];
  int16_t mag[3];
  float att[3];

  while (true)
  {
    MSPClient::Payload p1, p2;
    bool ok = msp.requestData(MSPClient::RAW_IMU, p1) &&
              MSPClient::ParseRawIMUPayload(p1, gyro, acc, mag) &&
              msp.requestData(MSPClient::ATTITUDE, p2) && MSPClient::ParseAttitudePayload(p2, att);

    if (ok)
    {
      size_t i;
      for (i = 0; i < 3; i++)
        std::cout << std::setw(8) << gyro[i] << ' ';
      for (i = 0; i < 3; i++)
        std::cout << std::setw(8) << acc[i] << ' ';
      for (i = 0; i < 3; i++)
        std::cout << std::setw(8) << mag[i] << ' ';
      for (i = 0; i < 3; i++)
        std::cout << std::setw(8) << att[i] << ' ';
      std::cout << '\n';
    }
    else
      logger->warn("Failed to get MSP data!");

    p1.clear();
    p2.clear();

    doSleep(10);
  }

  return 0;
}

vtkPolyData *generateCube(Vector3 d)
{
  d /= 2.0f;

  static float x[8][3] = {{-d.x(), -d.y(), -d.z()}, {d.x(), -d.y(), -d.z()}, {d.x(), d.y(), -d.z()},
                          {-d.x(), d.y(), -d.z()},  {-d.x(), -d.y(), d.z()}, {d.x(), -d.y(), d.z()},
                          {d.x(), d.y(), d.z()},    {-d.x(), d.y(), d.z()}};

  static vtkIdType pts[6][4] = {{0, 1, 2, 3}, {4, 5, 6, 7}, {0, 1, 5, 4},
                                {1, 2, 6, 5}, {2, 3, 7, 6}, {3, 0, 4, 7}};

  vtkPolyData *cube = vtkPolyData::New();
  vtkPoints *points = vtkPoints::New();
  vtkCellArray *polys = vtkCellArray::New();
  vtkFloatArray *scalars = vtkFloatArray::New();

  size_t i;
  for (i = 0; i < 8; i++)
    points->InsertPoint(i, x[i]);
  for (i = 0; i < 6; i++)
    polys->InsertNextCell(4, pts[i]);
  for (i = 0; i < 8; i++)
    scalars->InsertTuple1(i, i < 4);

  cube->SetPoints(points);
  points->Delete();
  cube->SetPolys(polys);
  polys->Delete();
  cube->GetPointData()->SetScalars(scalars);
  scalars->Delete();

  return cube;
}

int runFrameGrabber(IGrabber *grabber)
{
  auto logger = LoggingService::GetLogger("runFrameGrabber");

  logger->info("Blue is front of device, red is back of device");

  grabber->open();
  if (grabber->isOpen())
  {
    logger->info("Grabber is open.");
  }
  else
  {
    logger->error("Grabber failed to open.");
    return 2;
  }

  logger->info("Wait for device to wake...");
  doSleep(1000);
  logger->info("ok.");

  vtkPolyData *cube = generateCube(Vector3(1.0f, 0.1f, 1.0f));

  vtkPolyDataMapper *cubeMapper = vtkPolyDataMapper::New();
  cubeMapper->SetScalarRange(0, 1);
#if VTK_MAJOR_VERSION > 6
  cubeMapper->SetInputData(cube);
#else
  cubeMapper->SetInput(cube);
#endif

  vtkActor *cubeActor = vtkActor::New();
  cubeActor->SetMapper(cubeMapper);

  vtkRenderer *renderer = vtkRenderer::New();
  vtkRenderWindow *renderWindow = vtkRenderWindow::New();
  renderWindow->AddRenderer(renderer);
  vtkRenderWindowInteractor *rendererInteractor = vtkRenderWindowInteractor::New();
  rendererInteractor->SetRenderWindow(renderWindow);

  rendererInteractor->Initialize();

  vtkSmartPointer<VTKIMUActorCallback> cb = vtkSmartPointer<VTKIMUActorCallback>::New();
  cb->setGrabber(grabber);
  cb->setActor(cubeActor);
  rendererInteractor->AddObserver(vtkCommand::TimerEvent, cb);
  rendererInteractor->CreateRepeatingTimer(10);

  renderer->AddActor(cubeActor);
  renderer->SetBackground(0.2, 0.4, 0.8);
  renderWindow->SetSize(300, 300);

  vtkAxes *axis = vtkAxes::New();
  vtkPolyDataMapper *axisMapper = vtkPolyDataMapper::New();
  axisMapper->SetInputConnection(axis->GetOutputPort());
  vtkActor *axisActor = vtkActor::New();
  axisActor->SetMapper(axisMapper);
  renderer->AddActor(axisActor);

  renderer->ResetCamera();
  renderWindow->Render();

  rendererInteractor->Start();

  cube->Delete();
  cubeMapper->Delete();
  cubeActor->Delete();
  renderer->Delete();
  renderWindow->Delete();
  rendererInteractor->Delete();

  return 0;
}
