/** @file */

#pragma once

#include <memory>

#include <YukariIMU/IMUFrame.h>

#include <boost/filesystem.hpp>

#include <YukariCommon/LoggingService.h>

namespace Yukari
{
namespace Processing
{
  template <typename POINT_TYPE> class IFrameProcessingTask
  {
  public:
    typedef pcl::PointCloud<POINT_TYPE> Cloud;
    typedef typename Cloud::Ptr CloudPtr;
    typedef typename Cloud::ConstPtr CloudConstPtr;

    typedef std::shared_ptr<IFrameProcessingTask<POINT_TYPE>> Ptr;

  public:
    IFrameProcessingTask(const boost::filesystem::path &path)
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
                        IMU::IMUFrame::ConstPtr imuFrame) = 0;

    virtual int onStop()
    {
      return 0;
    }

  protected:
    boost::filesystem::path m_outputDirectory;
  };
}
}
