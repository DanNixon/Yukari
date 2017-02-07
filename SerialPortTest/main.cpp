#include <cstdio>
#include <iomanip>
#include <iostream>
#include <string>

// OS Specific sleep
#ifdef _WIN32
#include <windows.h>
#else
#include <unistd.h>
#endif

#include <YukariIMU/MSPClient.h>
#include <serial/serial.h>

using namespace Yukari::IMU;

void my_sleep(unsigned long milliseconds)
{
#ifdef _WIN32
  Sleep(milliseconds); // 100 ms
#else
  usleep(milliseconds * 1000); // 100 ms
#endif
}

void enumerate_ports()
{
  std::vector<serial::PortInfo> devices_found = serial::list_ports();

  std::vector<serial::PortInfo>::iterator iter = devices_found.begin();

  while (iter != devices_found.end())
  {
    serial::PortInfo device = *iter++;

    printf("(%s, %s, %s)\n", device.port.c_str(), device.description.c_str(),
           device.hardware_id.c_str());
  }
}

int main(int argc, char **argv)
{
  if (argc < 2)
  {
    return 1;
  }

  enumerate_ports();

  // Argument 1 is the serial port or enumerate flag
  std::string portName(argv[1]);

  if (portName == "-e")
  {
    enumerate_ports();
    return 0;
  }
  else if (argc < 3)
  {
    return 1;
  }

  // Argument 2 is the baudrate
  unsigned long baud = 0;
#if defined(WIN32) && !defined(__MINGW32__)
  sscanf_s(argv[2], "%lu", &baud);
#else
  sscanf(argv[2], "%lu", &baud);
#endif

  serial::Serial port(portName, baud);
  MSPClient msp(port);

  std::cout << "Port open: " << port.isOpen() << '\n';

  std::cout << "Board wake...\n";
  my_sleep(2000);
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
    my_sleep(10);
  }

  return 0;
}
