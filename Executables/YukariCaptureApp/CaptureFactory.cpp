/** @file */

#include "CaptureFactory.h"

#include <YukariCloudCapture/CloudGrabberFactory.h>
#include <YukariIMU/IMUGrabberFactory.h>

#include "SignalTrigger.h"
#include "TimelapseCaptureTrigger.h"

using namespace Yukari::CloudCapture;
using namespace Yukari::IMU;

namespace Yukari
{
namespace CaptureApp
{
  CaptureController_sptr CaptureFactory::Create(Common::ConfigurationManager::Config config)
  {
    CaptureController_sptr retVal = std::make_shared<CaptureController>();

    // TODO
    // config.get<std::string>("hello.world", "nope");

    // TODO
    retVal->setCloudGrabber(
        CloudGrabberFactory::Create("openni2", "", pcl::io::OpenNI2Grabber::OpenNI_Default_Mode,
                                    pcl::io::OpenNI2Grabber::OpenNI_Default_Mode));
    retVal->setIMUGrabber(IMUGrabberFactory::Create("attitude", "COM4", 115200));
    retVal->setRootOutputDirectory(boost::filesystem::path("C:\\Users\\b2026369\\out"));
    retVal->addExitTrigger(std::make_shared<SignalTrigger>(SIGINT));
    retVal->addCaptureTrigger(std::make_shared<TimelapseCaptureTrigger>(std::chrono::seconds(1)));
    // END

    return retVal;
  }
}
}