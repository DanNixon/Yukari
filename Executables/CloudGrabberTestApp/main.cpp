#include <pcl/console/parse.h>
#include <pcl/console/print.h>

#include <YukariCloudCapture/CloudGrabberFactory.h>
#include <YukariCloudCapture/ICloudGrabber.h>

#include "CloudGrabberVisualisation.h"

int main(int argc, char **argv)
{
  unsigned mode;

  pcl::io::OpenNI2Grabber::Mode depthMode = pcl::io::OpenNI2Grabber::OpenNI_Default_Mode;
  if (pcl::console::parse(argc, argv, "-depthmode", mode) != -1)
    depthMode = pcl::io::OpenNI2Grabber::Mode(mode);

  pcl::io::OpenNI2Grabber::Mode imageMode = pcl::io::OpenNI2Grabber::OpenNI_Default_Mode;
  if (pcl::console::parse(argc, argv, "-imagemode", mode) != -1)
    imageMode = pcl::io::OpenNI2Grabber::Mode(mode);

  Yukari::CloudCapture::ICloudGrabber_sptr grabber =
      Yukari::CloudCapture::CloudGrabberFactory::Create("openni2", "", depthMode, imageMode);
  Yukari::CloudGrabberTestApp::CloudGrabberVisualisation viewer(grabber);

  viewer.run();

  return 0;
}
