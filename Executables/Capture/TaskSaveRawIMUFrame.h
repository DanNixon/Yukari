/** @file */

#pragma once

#include "IPostCaptureTask.h"

#include <boost/filesystem/path.hpp>
#include <pcl/io/pcd_io.h>

#include <YukariCommon/LoggingService.h>

namespace Yukari
{
namespace CaptureApp
{
  class TaskSaveRawIMUFrame : public IPostCaptureTask
  {
  public:
    TaskSaveRawIMUFrame(const boost::filesystem::path &path)
        : IPostCaptureTask(path)
        , m_logger(Common::LoggingService::Instance().getLogger("TaskSaveRawIMUFrame"))
    {
    }

    virtual int process(size_t frameNumber, CloudConstPtr cloud,
                        IMU::IMUFrame_const_sptr imuFrame) override
    {
      if (imuFrame)
      {
        /* Generate filename */
        boost::filesystem::path imuFilename =
            m_outputDirectory / (std::to_string(frameNumber) + "_imu.txt");

        /* Save IMU frame */
        m_logger->trace("Saving IMU frame for frame {}: {}", frameNumber, imuFilename);
        std::ofstream imuFile;
        imuFile.open(imuFilename.string());
        imuFile << *imuFrame << '\n';
        imuFile.close();
      }
      else
      {
        m_logger->error("No IMU frame cloud, cannot save");
        return 1;
      }

      return 0;
    }

  private:
    Common::LoggingService::Logger m_logger;
  };
}
}
