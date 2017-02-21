#include <pcl/console/parse.h>
#include <pcl/console/print.h>

#include <YukariCloudCapture/DummyCloudGrabber.h>
#include <YukariCloudCapture/ICloudGrabber.h>
#include <YukariCloudCapture/OpenNI2CloudGrabber.h>

#include "CloudGrabberVisualisation.h"

int main(int argc, char **argv)
{
  /* Parse source type */
  std::string source;
  pcl::console::parse(argc, argv, "-source", source);

  /* Create cloud grabber */
  Yukari::CloudCapture::ICloudGrabber_sptr grabber;
  if (source == "dummy")
  {
    grabber = std::make_shared<Yukari::CloudCapture::DummyCloudGrabber>();
  }
  else if (source == "openni2")
  {
    unsigned mode;

    pcl::io::OpenNI2Grabber::Mode depthMode = pcl::io::OpenNI2Grabber::OpenNI_Default_Mode;
    if (pcl::console::parse(argc, argv, "-depthmode", mode) != -1)
      depthMode = pcl::io::OpenNI2Grabber::Mode(mode);

    pcl::io::OpenNI2Grabber::Mode imageMode = pcl::io::OpenNI2Grabber::OpenNI_Default_Mode;
    if (pcl::console::parse(argc, argv, "-imagemode", mode) != -1)
      imageMode = pcl::io::OpenNI2Grabber::Mode(mode);

    grabber = std::make_shared<Yukari::CloudCapture::OpenNI2CloudGrabber>("", depthMode, imageMode);
  }

  if (!grabber)
    return 1;

  Yukari::CloudGrabberTestApp::CloudGrabberVisualisation viewer(grabber);
  viewer.run();

  return 0;
}
