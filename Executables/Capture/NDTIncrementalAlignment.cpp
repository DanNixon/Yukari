/** @file */

#include "NDTIncrementalAlignment.h"

using namespace Yukari::Common;

namespace Yukari
{
namespace CaptureApp
{
  NDTIncrementalAlignment::NDTIncrementalAlignment()
      : m_logger(LoggingService::Instance().getLogger("NDTIncrementalAlignment"))
  {
  }

  int NDTIncrementalAlignment::process(size_t frameNumber, CloudConstPtr cloud,
                                       IMU::IMUFrame_const_sptr imuFrame)
  {
    // TODO
    return 0;
  }
}
}
