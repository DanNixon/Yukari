/** @file */

#include "CaptureFactory.h"

#include <memory>

#include <spdlog/sinks/file_sinks.h>

#include <YukariCloudCapture/CloudGrabberFactory.h>
#include <YukariCommon/LoggingService.h>
#include <YukariIMU/IMUGrabberFactory.h>
#include <YukariProcessing/FrameProcessingTaskFactory.h>
#include <YukariTriggers/SignalTrigger.h>
#include <YukariTriggers/TriggerFactory.h>

#include "Types.h"

using namespace Yukari::CloudCapture;
using namespace Yukari::Common;
using namespace Yukari::Maths;
using namespace Yukari::IMU;
using namespace Yukari::Triggers;
using namespace Yukari::Processing;

namespace Yukari
{
namespace Capture
{
  CaptureController::Ptr CaptureFactory::Create(boost::program_options::variables_map config)
  {
    auto logger = LoggingService::Instance().getLogger("CaptureFactory");

    CaptureController::Ptr retVal = std::make_shared<CaptureController>();

    /* Get destination directory */
    auto dir =
        boost::filesystem::absolute(boost::filesystem::path(config["dir"].as<std::string>()));
    logger->info("Absolute output directory: {}", dir);

    /* Output a log file in the output directory */
    boost::filesystem::path logFilename = dir / "log.txt";
    auto fileSink = std::make_shared<spdlog::sinks::simple_file_sink_mt>(logFilename.string());
    LoggingService::Instance().getSink()->addSink("file", fileSink);
    logger->debug("Added file sink");

    /* Get termination wait flag */
    bool forceExit = config.count("forceexit") > 0;
    logger->info("Forced exit: {}", forceExit);
    retVal->setForcedExit(forceExit);

    /* Get cloud grabber */
    CloudGrabberPtr grabber =
        CloudGrabberFactory<PointType>::Create(config["cloudgrabber"].as<std::string>());
    if (!grabber)
    {
      logger->error("Failed to create cloud grabber");
      return nullptr;
    }
    retVal->setCloudGrabber(grabber);

    /* Get IMU grabber */
    IIMUGrabber::Ptr imu;
    if (config.count("imugrabber"))
      imu = IMUGrabberFactory::Create(config["imugrabber"].as<std::string>());

    if (imu)
    {
      /* Set relative IMU transform from command line args */
      imu->setTransform(Transform(config));
      retVal->setIMUGrabber(imu);
    }
    else
    {
      logger->error("Failed to create IMU grabber");
    }

    /* Get capture trigger */
    {
      ITrigger::Ptr trigger = TriggerFactory::Create(config["capturetrigger"].as<std::string>());

      if (trigger)
        retVal->addCaptureTrigger(trigger);
      else
        logger->warn("Failed to create capture trigger");
    }

    /* Get exit trigger */
    {
      ITrigger::Ptr trigger = TriggerFactory::Create(config["exittrigger"].as<std::string>());

      if (trigger)
      {
        retVal->addExitTrigger(trigger);
      }
      else
      {
        logger->error("Failed to create exit trigger");
        return nullptr;
      }
    }

    /* Add post capture operations */
    std::vector<std::string> processingStages = config["process"].as<std::vector<std::string>>();
    if (processingStages.empty())
    {
      logger->error("No processing stages are defined, this capture will be pointless");
      return nullptr;
    }
    else
    {
      logger->debug("Num processing stages defined: {}", processingStages.size());

      for (auto it = processingStages.begin(); it != processingStages.end(); ++it)
      {
        auto task = FrameProcessingTaskFactory::Create(*it, dir);

        if (task)
          retVal->addPostCaptureTask(task);
        else
          logger->warn("Failed to create task: \"{}\"", *it);
      }
    }

    logger->debug("Created: {}", *retVal);

    return retVal;
  }
}
}
