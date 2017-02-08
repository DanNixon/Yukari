#include <cstdio>
#include <iomanip>
#include <iostream>
#include <string>

#ifdef _WIN32
#include <windows.h>
#else
#include <unistd.h>
#endif

#include <serial/serial.h>

#include <YukariIMU/MSPClient.h>
#include <YukariIMU/IGrabber.h>
#include <YukariIMU/MSPGrabberAttitude.h>
#include <YukariIMU/MSPGrabberIMU.h>

using namespace Yukari::IMU;

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
  serial::Serial port(portName, baud);
  MSPClient msp(port);

  std::cout << "Port open: " << port.isOpen() << '\n';
  if (!port.isOpen())
    return 3;

  std::cout << "Board wake...\n";
  doSleep(1000);
  std::cout << "ok.\n";

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
      std::cout << "fail\n";

    p1.clear();
    p2.clear();

    doSleep(10);
  }

  return 0;
}

int runFrameGrabber(IGrabber * grabber)
{
  grabber->open();
  std::cout << "Grabber open: " << grabber->isOpen() << '\n';
  if (!grabber->isOpen())
    return 3;

  std::cout << "Device wake...\n";
  doSleep(1000);
  std::cout << "ok.\n";

  while(true)
  {
    auto frame = grabber->grabFrame();
    std::cout << *frame << '\n';

    doSleep(10);
  }

  return 0;
}
