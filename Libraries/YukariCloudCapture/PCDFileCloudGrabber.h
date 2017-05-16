/** @file */

#pragma once

#include "PCLCloudGrabberWrapper.h"

#include <pcl/io/pcd_grabber.h>

#include <YukariCommon/FilesystemHelpers.h>
#include <YukariCommon/LoggingService.h>
#include <YukariTriggers/ProxyTrigger.h>

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
        , m_trigger(std::make_shared<Triggers::ProxyTrigger>())
    {
      auto logger = Common::LoggingService::Instance().getLogger("PCDFileCloudGrabber");

      FilesystemHelpers::FindByRegex(root, pattern, m_filenames);
      logger->info("Found {} PCD files", m_filenames.size());

      for (auto it = m_filenames.begin(); it != m_filenames.end(); ++it)
        logger->trace("File: {}", *it);

      setGrabber(std::make_shared<pcl::PCDGrabber<POINT_TYPE>>(m_filenames, fps, false));
    }

    virtual ~PCDFileCloudGrabber()
    {
    }

    virtual Triggers::ITrigger::Ptr trigger()
    {
      return m_trigger;
    }

  protected:
    virtual void onCloud(CloudConstPtr cloud) override
    {
      m_trigger->trigger();
    }

  private:
    Common::FilesystemHelpers::PathStringList m_filenames;
    std::shared_ptr<Triggers::ProxyTrigger> m_trigger;
  };
}
}
