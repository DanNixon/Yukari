/** @file */

#include "CaptureFactory.h"

#include <memory>

#include <YukariCaptureTriggers/SignalTrigger.h>
#include <YukariCaptureTriggers/TimelapseCaptureTrigger.h>
#include <YukariCloudCapture/CloudGrabberFactory.h>
#include <YukariCommon/LoggingService.h>
#include <YukariIMU/DummyIMUGrabber.h>
#include <YukariIMU/MSPGrabberAttitude.h>
#include <YukariIMU/TeensyIMUDevice.h>

#include "Types.h"

using namespace Yukari::CaptureTriggers;
using namespace Yukari::CloudCapture;
using namespace Yukari::Common;
using namespace Yukari::IMU;

namespace Yukari
{
namespace CaptureApp
{
  CaptureController_sptr CaptureFactory::Create(boost::program_options::variables_map config)
  {
    auto logger = LoggingService::Instance().getLogger("CaptureFactory");

    CaptureController_sptr retVal = std::make_shared<CaptureController>();

    /* Get destination directory */
    retVal->setRootOutputDirectory(
        boost::filesystem::path(config["dir"].as<std::string>()));

    /* Get cloud grabber */
    {
      /* Create cloud grabber */
      CloudGrabberPtr grabber = CloudGrabberFactory<pcl::PointXYZRGBA>::Create(config["cloudgrabber"].as<std::string>());
      if (!grabber)
      {
        logger->error("Failed to create cloud grabber");
        return nullptr;
      }

      retVal->setCloudGrabber(grabber);
    }

    /* Get IMU grabber */
    {
      /* Get data */
      std::string type = config["imugrabber"].as<std::string>();
      std::string port = config["imuport"].as<std::string>();
      int baud = config["imubaud"].as<int>();

      /* Create IMU grabber */
      IIMUGrabber_sptr imu;
      if (type == "dummy")
        imu = std::make_shared<DummyIMUGrabber>();
      else if (type == "attitude")
        imu = std::make_shared<MSPGrabberAttitude>(port, baud);
      else if (type == "teensy")
        imu = std::make_shared<TeensyIMUDevice>(port, baud);
      else
        logger->warn("Unknown IMU grabber type");

      if (!imu)
        logger->error("Failed to create IMU grabber");

      retVal->setIMUGrabber(imu);
    }

    /* Get start triggers */
    {
      auto triggers = config.get_child_optional("capture.triggers.start");

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
            retVal->addStartTrigger(trigger);
            logger->trace("Added start trigger");
          }
          else
          {
            logger->warn("Failed to create start trigger");
          }
        }
      }
    }

    /* Get stop triggers */
    {
      auto triggers = config.get_child_optional("capture.triggers.stop");

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
            retVal->addStopTrigger(trigger);
            logger->trace("Added stop trigger");
          }
          else
          {
            logger->warn("Failed to create stop trigger");
          }
        }
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
    }

    logger->debug("Created: {}", *retVal);

    return retVal;
  }
}
}
