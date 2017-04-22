/** @file */

#pragma once

#include "IFrameProcessingTask.h"

#include <boost/filesystem/path.hpp>
#include <pcl/io/pcd_io.h>

#include <YukariCommon/LoggingService.h>

namespace Yukari
{
namespace Processing
{
  template <typename POINT_TYPE> class TaskSaveRawIMUFrame : public IFrameProcessingTask<POINT_TYPE>
  {
  public:
    TaskSaveRawIMUFrame(const boost::filesystem::path &path)
        : IFrameProcessingTask(path)
        , m_logger(Common::LoggingService::Instance().getLogger("TaskSaveRawIMUFrame"))
    {
    }

    virtual int process(Task t) override
    {
      if (t.imuFrame)
      {
        /* Generate filename */
        boost::filesystem::path imuFilename =
            m_outputDirectory / (std::to_string(t.frameNumber) + "_imu.txt");

        /* Save IMU frame */
        m_logger->trace("Saving IMU frame for frame {}: {}", t.frameNumber, imuFilename);
        std::ofstream imuFile;
        imuFile.open(imuFilename.string());
        imuFile << *t.imuFrame << '\n';
        imuFile.close();
      }
      else
      {
        m_logger->error("No IMU frame, cannot save");
        return 1;
      }

      return 0;
    }

  private:
    Common::LoggingService::Logger m_logger;
  };
}
}
