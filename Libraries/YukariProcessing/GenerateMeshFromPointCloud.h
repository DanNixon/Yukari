/** @file */

#pragma once

#include <pcl/PolygonMesh.h>

#include <YukariCloudCapture/ICloudGrabber.h>
#include <YukariCommon/LoggingService.h>

namespace Yukari
{
namespace Processing
{
  class GenerateMeshFromPointCloud
  {
  public:
    struct Parameters
    {
      // TODO
    };

  public:
    GenerateMeshFromPointCloud();

    pcl::PolygonMesh::Ptr
    estimateSingle(const Yukari::CloudCapture::ICloudGrabber::Cloud::ConstPtr cloud,
                   const Parameters &params);

  private:
    Common::LoggingService::Logger m_logger;
  };
}
}
