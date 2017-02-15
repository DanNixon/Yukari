/** @file */

#include "CaptureFactory.h"

#include <YukariCloudCapture/CloudGrabberFactory.h>
#include <YukariCommon/LoggingService.h>
#include <YukariIMU/IMUGrabberFactory.h>

#include "SignalTrigger.h"
#include "TimelapseCaptureTrigger.h"

using namespace Yukari::CloudCapture;
using namespace Yukari::Common;
using namespace Yukari::IMU;

namespace Yukari
{
namespace CaptureApp
{
  CaptureController_sptr CaptureFactory::Create(Common::ConfigurationManager::Config config)
  {
    auto logger = LoggingService::GetLogger("CaptureFactory");

    CaptureController_sptr retVal = std::make_shared<CaptureController>();

    /* Get destination directory */
    retVal->setRootOutputDirectory(
        boost::filesystem::path(config.get<std::string>("capture.output_root_directory", ".")));

    /* Get cloud grabber */
    {
      std::string type = config.get<std::string>("capture.cloud.grabber");

      if (type == "openni")
      {
        // TODO: add devcie ID and modes
        std::string deviceID = config.get<std::string>("capture.cloud.device", "");
        retVal->setCloudGrabber(CloudGrabberFactory::Create(
            type, deviceID, pcl::io::OpenNI2Grabber::OpenNI_Default_Mode,
            pcl::io::OpenNI2Grabber::OpenNI_Default_Mode));
      }
      else
      {
        logger->error("Unknown cloud grabber type");
        return nullptr;
      }
    }

    /* Get IMU grabber */
    {
      std::string type = config.get<std::string>("capture.imu.grabber");
      std::string port = config.get<std::string>("capture.imu.port");
      int baud = config.get<int>("capture.imu.baud", 115200);

      auto imu = IMUGrabberFactory::Create(type, port, baud);
      if (!imu)
      {
        logger->error("Failed to create IMU grabber");
        return nullptr;
      }

      retVal->setIMUGrabber(imu);
    }

    /* Get exit triggers */
    {
      auto triggers = config.get_child_optional("capture.triggers.exit");

      if (triggers && !triggers->empty())
      {
        for (auto it = triggers->begin(); it != triggers->end(); ++it)
        {
          ITrigger_sptr trigger;

          /* Create trigger */
          std::string type = it->second.get<std::string>("type");
          if (type == "signal")
          {
            int signal = it->second.get<int>("signal");
            trigger = std::make_shared<SignalTrigger>(signal);
          }
          else
          {
            logger->warn("Unknown or unsupported trigger type");
          }

          /* Add trigger */
          if (trigger)
          {
            retVal->addExitTrigger(trigger);
            logger->trace("Added exit trigger");
          }
          else
          {
            logger->warn("Failed to create exit trigger");
          }
        }
      }
      else
      {
        logger->error("No exit triggers defined (at least one must be defined)");
        return nullptr;
      }
    }

    /* Get capture triggers */
    {
      auto triggers = config.get_child_optional("capture.triggers.capture");

      if (triggers && !triggers->empty())
      {
        for (auto it = triggers->begin(); it != triggers->end(); ++it)
        {
          ITrigger_sptr trigger;

          /* Create trigger */
          std::string type = it->second.get<std::string>("type");
          if (type == "signal")
          {
            int signal = it->second.get<int>("signal");
            trigger = std::make_shared<SignalTrigger>(signal);
          }
          if (type == "timelapse")
          {
            int seconds = it->second.get<int>("seconds");
            trigger = std::make_shared<TimelapseCaptureTrigger>(std::chrono::seconds(seconds));
          }
          else
          {
            logger->warn("Unknown or unsupported trigger type");
          }

          /* Add trigger */
          if (trigger)
          {
            retVal->addCaptureTrigger(trigger);
            logger->trace("Added capture trigger");
          }
          else
          {
            logger->warn("Failed to create capture trigger");
          }
        }
      }
      else
      {
        logger->error("No capture triggers defined (at least one must be defined)");
        return nullptr;
      }
    }

    logger->info("Created: {}", retVal);

    return retVal;
  }
}
}