/** @file */

#pragma once

#include <memory>

#include <YukariIMU/IMUFrame.h>

#include <boost/filesystem.hpp>

#include <YukariCommon/LoggingService.h>

#include "Types.h"

namespace Yukari
{
namespace CaptureApp
{
  class IPostCaptureTask
  {
  public:
    IPostCaptureTask(const boost::filesystem::path &path)
        : m_outputDirectory(path)
    {
      auto logger = Common::LoggingService::Instance().getLogger("IPostCaptureTask");

      /* Ensure capture directory exists */
      logger->info("Capture path: {}", m_outputDirectory);
      boost::filesystem::create_directories(m_outputDirectory);
      logger->debug("Capture root directory created");
    }

    virtual int onStart()
    {
      return 0;
    }

    virtual int process(size_t frameNumber, CloudConstPtr cloud,
                        IMU::IMUFrame_const_sptr imuFrame) = 0;

    virtual int onStop()
    {
      return 0;
    }

  protected:
    boost::filesystem::path m_outputDirectory;
  };

  typedef std::shared_ptr<IPostCaptureTask> IPostCaptureTask_sptr;
}
}
