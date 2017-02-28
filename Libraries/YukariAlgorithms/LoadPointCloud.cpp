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
    m_validator = [](const PropertyContainer &inProps, const PropertyContainer &) {
      auto file = inProps.find("file");
      if (file == inProps.end())
        return "No file provided";

      if (file->second.size() == 0)
        return "No files provided";

      return "";
    };
  }

  void LoadPointCloud::execute()
  {
    Property file = getProperty(Processing::INPUT, "file");
    size_t len = file.size();
    Property cloud(len);

    for (size_t i = 0; i < len; i++)
    {
      const std::string filename = file.value<std::string>(i);

      m_logger->trace("Loading point cloud from file \"{}\"", filename);
      pcl::PointCloud<pcl::PointXYZRGBA>::Ptr c(new pcl::PointCloud<pcl::PointXYZRGBA>);
      if (pcl::io::loadPCDFile<pcl::PointXYZRGBA>(filename, *c) == -1)
      {
        m_logger->error("Failed to load point cloud ({})", filename);
      }
      else
      {
        m_logger->trace("Loaded point cloud ({})", filename);
        cloud[i] = c;
      }
    }

    setProperty(Processing::OUTPUT, "cloud", cloud);
  }
}
}
