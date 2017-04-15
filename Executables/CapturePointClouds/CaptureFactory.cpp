/** @file */

#include "CaptureFactory.h"

#include <memory>

#include <YukariCloudCapture/CloudGrabberFactory.h>
#include <YukariCommon/LoggingService.h>
#include <YukariIMU/IMUGrabberFactory.h>
#include <YukariTriggers/TriggerFactory.h>

#include "Types.h"

using namespace Yukari::CloudCapture;
using namespace Yukari::Common;
using namespace Yukari::IMU;
using namespace Yukari::Triggers;

namespace Yukari
{
namespace CaptureApp
{
  CaptureController_sptr CaptureFactory::Create(boost::program_options::variables_map config)
  {
    auto logger = LoggingService::Instance().getLogger("CaptureFactory");

    CaptureController_sptr retVal = std::make_shared<CaptureController>();

    /* Get destination directory */
    auto dir =
        boost::filesystem::absolute(boost::filesystem::path(config["dir"].as<std::string>()));
    logger->info("Absoluteoutput directory: {}", dir);
    retVal->setOutputDirectory(dir);

    /* Get cloud grabber */
    CloudGrabberPtr grabber =
        CloudGrabberFactory<pcl::PointXYZRGBA>::Create(config["cloudgrabber"].as<std::string>());
    if (!grabber)
    {
      logger->error("Failed to create cloud grabber");
      return nullptr;
    }
    retVal->setCloudGrabber(grabber);

    /* Get IMU grabber */
    IIMUGrabber_sptr imu = IMUGrabberFactory::Create(config["imugrabber"].as<std::string>());
    if (!imu)
      logger->error("Failed to create IMU grabber");
    retVal->setIMUGrabber(imu);

    /* Get transformation option */
    bool transformNow = config.count("transform") != 0;
    retVal->setTransformMode(transformNow ? CaptureController::TransformMode::TRANSFORM_NOW : CaptureController::TransformMode::SAVE_TRANSFORM);

    /* Get capture triggers */
    ITrigger_sptr trigger = TriggerFactory::Create(config["capturetrigger"].as<std::string>());
    if (trigger)
    {
      retVal->addCaptureTrigger(trigger);
      logger->trace("Added capture trigger");
    }
    else
    {
      logger->warn("Failed to create capture trigger");
    }

    logger->debug("Created: {}", *retVal);

    return retVal;
  }
}
}
