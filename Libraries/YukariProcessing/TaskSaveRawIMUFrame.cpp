/** @file */

#include "TaskSaveRawIMUFrame.h"

#include <iomanip>
#include <sstream>

#include <pcl/io/pcd_io.h>

using namespace Yukari::Common;

namespace Yukari
{
namespace Processing
{
  TaskSaveRawIMUFrame::TaskSaveRawIMUFrame(const boost::filesystem::path &path)
      : IFrameProcessingTask(path)
      , m_logger(LoggingService::Instance().getLogger("TaskSaveRawIMUFrame"))
  {
  }

  int TaskSaveRawIMUFrame::process(Task t)
  {
    if (t.imuFrame)
    {
      /* Generate filename */
      std::stringstream ss;
      ss << std::setw(5) << std::setfill('0') << t.frameNumber;
      boost::filesystem::path imuFilename = m_outputDirectory / (ss.str() + "_imu.txt");

      /* Save IMU frame */
      t.imuFrame->save(imuFilename);
    }
    else
    {
      m_logger->error("No IMU frame, cannot save");
      return 1;
    }

    return 0;
  }
}
}