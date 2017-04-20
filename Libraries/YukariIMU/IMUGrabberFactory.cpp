/** @file */

#include "IMUGrabberFactory.h"

#include <YukariCommon/MapHelpers.h>
#include <YukariCommon/StringParsers.h>

#include "DummyIMUGrabber.h"
#include "MSPGrabberAttitude.h"
#include "TeensyIMUDevice.h"

using namespace Yukari::Common;

namespace Yukari
{
namespace IMU
{
  IIMUGrabber::Ptr IMUGrabberFactory::Create(const std::string &fullCommand)
  {
    std::string type;
    std::map<std::string, std::string> params;
    if (!StringParsers::ParseCommand(fullCommand, type, params))
      return nullptr;

    return Create(type, params);
  }

  IIMUGrabber::Ptr IMUGrabberFactory::Create(const std::string &type,
                                             std::map<std::string, std::string> &parameters)
  {
    std::string lowerType = type;
    StringParsers::CleanString(lowerType);

    IIMUGrabber::Ptr grabber;
    if (lowerType == "dummy")
    {
      grabber = std::make_shared<DummyIMUGrabber>();
    }
    else if (lowerType == "attitude")
    {
      std::string port = MapHelpers::Get<std::string, std::string>(parameters, "port");
      int baud = std::stoi(MapHelpers::Get<std::string, std::string>(parameters, "baud", "115200"));

      grabber = std::make_shared<MSPGrabberAttitude>(port, baud);
    }
    else if (lowerType == "teensy")
    {
      std::string port = MapHelpers::Get<std::string, std::string>(parameters, "port");
      int baud = std::stoi(MapHelpers::Get<std::string, std::string>(parameters, "baud", "115200"));

      grabber = std::make_shared<TeensyIMUDevice>(port, baud);
    }

    return grabber;
  }
}
}
