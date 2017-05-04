/** @file */

#include "TaskNDTWorldSegmentAlignment.h"

#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/ndt.h>

#include "CloudOperations.h"

using namespace Yukari::Common;

namespace Yukari
{
namespace Processing
{
  TaskNDTWorldSegmentAlignment::TaskNDTWorldSegmentAlignment(
      const boost::filesystem::path &path, std::map<std::string, std::string> &params)
      : ITaskAlignment(path, params)
      , m_logger(LoggingService::Instance().getLogger("TaskNDTWorldSegmentAlignment"))
  {
  }

  int TaskNDTWorldSegmentAlignment::process(Task t)
  {
    if (!(t.cloud && t.imuFrame))
    {
      m_logger->error("Do not have both cloud and IMU frame");
      return 1;
    }

    /* TODO */

    return 0;
  }

  int TaskNDTWorldSegmentAlignment::onStop()
  {
    /* TODO */

    return 0;
  }
}
}