/** @file */

#include "CaptureFactory.h"

#include <memory>

#include <YukariCloudCapture/CloudGrabberFactory.h>
#include <YukariCommon/LoggingService.h>
#include <YukariIMU/IMUGrabberFactory.h>
#include <YukariTriggers/TriggerFactory.h>

#include "NDTIncrementalAlignment.h"
#include "TaskSaveRawCloud.h"
#include "TaskSaveRawIMUFrame.h"
#include "Types.h"

using namespace Yukari::CloudCapture;
using namespace Yukari::Common;
using namespace Yukari::Maths;
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
    logger->info("Absolute output directory: {}", dir);

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
    if (imu)
    {
      /* Set relative IMU transform from command line args */
      imu->setTransform(Transform(config));
    }
    else
    {
      logger->error("Failed to create IMU grabber");
    }
    retVal->setIMUGrabber(imu);

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

    /* Add post capture operations */
    // TODO
    retVal->addPostCaptureTask(std::make_shared<TaskSaveRawCloud>(dir / "raw_clouds"));
    retVal->addPostCaptureTask(
        std::make_shared<TaskSaveRawCloud>(dir / "transformed_clouds", true));
    retVal->addPostCaptureTask(std::make_shared<TaskSaveRawIMUFrame>(dir / "imu"));
    retVal->addPostCaptureTask(std::make_shared<NDTIncrementalAlignment>());

    logger->debug("Created: {}", *retVal);

    return retVal;
  }
}
}
