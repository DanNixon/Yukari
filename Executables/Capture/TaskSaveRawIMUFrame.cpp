/** @file */

#include "TaskSaveRawIMUFrame.h"

#include <pcl/io/pcd_io.h>

using namespace Yukari::Common;

namespace Yukari
{
namespace CaptureApp
{
  TaskSaveRawIMUFrame::TaskSaveRawIMUFrame(const boost::filesystem::path &path)
      : m_logger(LoggingService::Instance().getLogger("TaskSaveRawIMUFrame"))
      , m_outputDirectory(path)
  {
    /* Ensure output directory exists */
    m_logger->info("Save path: {}", m_outputDirectory);
    boost::filesystem::create_directories(m_outputDirectory);
    m_logger->debug("Directory created");
  }

  int TaskSaveRawIMUFrame::process(size_t frameNumber, CloudConstPtr cloud,
                                   IMU::IMUFrame_const_sptr imuFrame)
  {
    if (imuFrame)
    {
      boost::filesystem::path imuFilename =
          m_outputDirectory / (std::to_string(frameNumber) + "_imu.txt");

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
}
}
