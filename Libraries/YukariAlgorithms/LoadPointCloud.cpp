/** @file */

#include "LoadPointCloud.h"

#include <YukariCloudCapture/ICloudGrabber.h>
#include <pcl/io/pcd_io.h>

using namespace Yukari::CloudCapture;
using namespace Yukari::Processing;

namespace Yukari
{
namespace Algorithms
{
  LoadPointCloud::LoadPointCloud()
      : m_logger(Common::LoggingService::GetLogger("LoadPointCloud"))
  {
    /* Add default validator */
    m_validator = [](const IAlgorithm &alg) {
      Property_sptr file = alg.getProperty(Processing::INPUT, "file");

      if (file->size() == 0)
        return "No files provided";

      return "";
    };
  }

  void LoadPointCloud::doExecute()
  {
    Property_sptr file = getProperty(Processing::INPUT, "file");
    size_t len = file->size();
    Property_sptr cloud = std::make_shared<Property>(len);

    for (size_t i = 0; i < len; i++)
    {
      const std::string filename = file->value<std::string>(i);

      m_logger->trace("Loading point cloud from file \"{}\"", filename);
      pcl::PointCloud<pcl::PointXYZRGBA>::Ptr c(new pcl::PointCloud<pcl::PointXYZRGBA>);
      if (pcl::io::loadPCDFile<pcl::PointXYZRGBA>(filename, *c) == -1)
      {
        m_logger->error("Failed to load point cloud ({})", filename);
      }
      else
      {
        m_logger->trace("Loaded point cloud ({})", filename);
        (*cloud)[i] = c;
      }
    }

    setProperty(Processing::OUTPUT, "cloud", cloud);
  }
}
}
