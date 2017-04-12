/** @file */

#include <chrono>
#include <iostream>
#include <string>

#include <boost/program_options.hpp>
#include <serial/serial.h>

#include <YukariCommon/LoggingService.h>
#include <YukariMSP/MSPClient.h>

using namespace Yukari::Common;
using namespace Yukari::MSP;
namespace po = boost::program_options;

int main(int argc, char **argv)
{
  auto logger = LoggingService::GetLogger("main");

  /* Init command line */
  po::options_description desc("Allowed options");
  po::variables_map args;

  // clang-format off
  desc.add_options()
    ("help", "Show brief usage message")
    ("port", po::value<std::string>(), "Serial port for IMU device")
    ("baud", po::value<int>()->default_value(115200), "Baud rate used for serial communication");
  // clang-format on

  /* Parse command line args */
  try
  {
    po::store(po::parse_command_line(argc, argv, desc), args);
  }
  catch (po::error const &e)
  {
    std::cerr << e.what() << '\n';
    return 1;
  }

  /* Show usage */
  if (args.count("help"))
  {
    std::cout << desc << "\n";
    return 1;
  }

  if (args.count("port") == 0)
  {
    /* List all ports */
    std::vector<serial::PortInfo> ports = serial::list_ports();
    for (auto it = ports.begin(); it != ports.end(); ++it)
      printf("(%s, %s, %s)\n", it->port.c_str(), it->description.c_str(), it->hardware_id.c_str());
  }
  else
  {
    /* Create MSP client */
    serial::Serial port(args["port"].as<std::string>(), args["baud"].as<int>());
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

    /* Send a test payload */
    {
      union {
        struct
        {
          int16_t w;
          int16_t i;
          int16_t j;
          int16_t k;
        } q;

        uint8_t d[8];
      } u;

      for (size_t i = 0; i < 100; i++)
      {
        MSPClient::Payload payload = {0x22, 0x33, 0x44, 0x55};
        bool good = msp.requestData(MSPClient::Y_ORIENTATION, payload);
        logger->info("MSP result: {}", good);
        logger->info("RX payload: {}", payload);

        std::copy(payload.begin(), payload.end(), u.d);

        logger->info("q.w = {}", u.q.w);
        logger->info("q.i = {}", u.q.i);
        logger->info("q.j = {}", u.q.j);
        logger->info("q.k = {}", u.q.k);

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
    }
  }

  return 0;
}
