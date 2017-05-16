/** @file */

#include "ITaskIncrementalAlignment.h"

#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/ndt.h>

#include <YukariCommon/MapHelpers.h>
#include <YukariCommon/StringParsers.h>

using namespace Yukari::Common;

namespace Yukari
{
namespace Processing
{
  ITaskIncrementalAlignment::ITaskIncrementalAlignment(const boost::filesystem::path &path,
                                                       std::map<std::string, std::string> &params)
      : ITaskAlignment(path, params)
      , m_logger(LoggingService::Instance().getLogger("ITaskIncrementalAlignment"))
      , m_saveTransforms(false)
      , m_saveClouds(false)
      , m_previousCloud()
      , m_previousCloudWorldTransform(Eigen::Matrix4f::Identity())
  {
    /* Parse save transforms option */
    {
      std::string paramStr = MapHelpers::Get<std::string, std::string>(params, "transform", "true");
      StringParsers::CleanString(paramStr);
      m_saveTransforms = paramStr == "true";
    }

    /* Parse save clouds option */
    {
      std::string paramStr = MapHelpers::Get<std::string, std::string>(params, "cloud", "true");
      StringParsers::CleanString(paramStr);
      m_saveClouds = paramStr == "true";
    }
  }

  int ITaskIncrementalAlignment::process(Task t)
  {
    if (!(t.cloud && t.imuFrame))
    {
      m_logger->error("Do not have both cloud and IMU frame");
      return 1;
    }

    /* Format frame number */
    std::stringstream ss;
    ss << std::setw(5) << std::setfill('0') << t.frameNumber;
    std::string frameNoStr = ss.str();

    if (!m_previousCloud)
    {
      /* Set initial transform */
      m_previousCloudWorldTransform = t.imuFrame->toCloudTransform();
    }
    else
    {
      doAlignment(t);
    }

    /* Set previous cloud */
    m_previousCloud = CloudPtr(new Cloud(*t.cloud));

    /* Transform the previous/target cloud by it's world position (for next frame) */
    pcl::transformPointCloud<PointT>(*m_previousCloud, *m_previousCloud,
                                     m_previousCloudWorldTransform);

    /* Save transformed cloud */
    if (m_saveClouds)
    {
      boost::filesystem::path cloudFilename = m_outputDirectory / (frameNoStr + "_cloud.pcd");
      m_logger->trace("Saving transformed point cloud for frame {}: {}", t.frameNumber,
                      cloudFilename);
      pcl::io::savePCDFileBinaryCompressed(cloudFilename.string(), *m_previousCloud);
    }

    /* Save transformation */
    if (m_saveTransforms)
    {
      IMU::IMUFrame transformFrame(m_previousCloudWorldTransform);
      boost::filesystem::path imuFilename = m_outputDirectory / (frameNoStr + "_transform.txt");
      transformFrame.save(imuFilename);
    }

    return 0;
  }
}
}
