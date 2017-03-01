/** @file */

#include "GenerateMeshFromPointCloud.h"

#include <YukariCloudCapture/ICloudGrabber.h>
#include <pcl/io/pcd_io.h>

using namespace Yukari::CloudCapture;
using namespace Yukari::Processing;

namespace Yukari
{
namespace Algorithms
{
  GenerateMeshFromPointCloud::GenerateMeshFromPointCloud()
      : m_logger(Common::LoggingService::GetLogger("GenerateMeshFromPointCloud"))
  {
    /* Add default validator */
    m_validator = [](const IAlgorithm &alg) {
      /* TODO */
      return "";
    };
  }

  void GenerateMeshFromPointCloud::doExecute()
  {
    /* TODO */
  }
}
}
