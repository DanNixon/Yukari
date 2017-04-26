/** @file */

#pragma once

#include "PCLCloudGrabberWrapper.h"

#include <pcl/io/pcd_grabber.h>

#include <YukariCommon/FilesystemHelpers.h>
#include <YukariCommon/LoggingService.h>

namespace Yukari
{
namespace CloudCapture
{
  template <typename POINT_TYPE>
  class PCDFileCloudGrabber : public PCLCloudGrabberWrapper<POINT_TYPE>
  {
  public:
    PCDFileCloudGrabber(const boost::filesystem::path &root, const std::string &pattern, float fps)
        : PCLCloudGrabberWrapper(nullptr)
    {
      auto logger = Common::LoggingService::Instance().getLogger("PCDFileCloudGrabber");

      FilesystemHelpers::FindByRegex(root, pattern, m_filenames);
      logger->info("Found {} PCD files", m_filenames.size());

      setGrabber(std::make_shared<pcl::PCDGrabber<POINT_TYPE>>(m_filenames, fps, false));
    }

    virtual ~PCDFileCloudGrabber()
    {
    }

  private:
    Common::FilesystemHelpers::PathStringList m_filenames;
  };
}
}
