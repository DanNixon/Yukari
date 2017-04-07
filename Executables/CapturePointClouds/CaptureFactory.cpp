/** @file */

#include "CaptureFactory.h"

#include <memory>

#include <YukariCaptureTriggers/SignalTrigger.h>
#include <YukariCaptureTriggers/TimelapseCaptureTrigger.h>
#include <YukariCloudCapture/DummyCloudGrabber.h>
#include <YukariCloudCapture/OpenNI2CloudGrabber.h>
#include <YukariCommon/LoggingService.h>
#include <YukariIMU/DummyIMUGrabber.h>
#include <YukariIMU/MSPGrabberAttitude.h>
#include <YukariIMU/TeensyIMUDevice.h>

using namespace Yukari::CaptureTriggers;
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
      std::string type = config.get<std::string>("capture.cloud.grabber", "dummy");

      /* Create cloud grabber */
      ICloudGrabber_sptr grabber;
      if (type == "dummy")
      {
        grabber = std::make_shared<DummyCloudGrabber>();
      }
      else if (type == "openni2")
      {
        // TODO: add modes
        std::string deviceID = config.get<std::string>("capture.cloud.device", "");
        grabber = std::make_shared<OpenNI2CloudGrabber>(
            deviceID, pcl::io::OpenNI2Grabber::OpenNI_Default_Mode,
            pcl::io::OpenNI2Grabber::OpenNI_Default_Mode);
      }
      else
      {
        logger->warn("Unknown cloud grabber type");
      }

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
      std::string type = config.get<std::string>("capture.imu.grabber", "dummy");
      std::string port = config.get<std::string>("capture.imu.port", "");
      int baud = config.get<int>("capture.imu.baud", 115200);

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
