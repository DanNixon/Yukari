/***
 * This example expects the serial port has a loopback on it.
 *
 * Alternatively, you could use an Arduino:
 *
 * <pre>
 *  void setup() {
 *    Serial.begin(<insert your baudrate here>);
 *  }
 *
 *  void loop() {
 *    if (Serial.available()) {
 *      Serial.write(Serial.read());
 *    }
 *  }
 * </pre>
 */

#include <cstdio>
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

using std::string;
using std::exception;
using std::cout;
using std::cerr;
using std::endl;
using std::vector;
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
  vector<serial::PortInfo> devices_found = serial::list_ports();

  vector<serial::PortInfo>::iterator iter = devices_found.begin();

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

  // Argument 1 is the serial port or enumerate flag
  string portName(argv[1]);

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

  cout << "Port open: " << port.isOpen() << '\n';

  cout << "Board wake...\n";
  my_sleep(2000);
  cout << "ok.\n";

  while (true)
  {
    MSPClient::Payload p;
    bool ok = msp.requestData(MSPClient::RAW_IMU, p);

    if (ok)
    {
      std::cout << std::hex;
      for (auto it = p.begin(); it != p.end(); ++it)
        std::cout << *it << ' ';
      std::cout << '\n';
    }
    else
      std::cout << "fail\n";

    p.clear();
  }

  return 0;
}
