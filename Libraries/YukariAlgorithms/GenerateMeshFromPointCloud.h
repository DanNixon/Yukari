/** @file */

#pragma once

#include <YukariProcessing/IAlgorithm.h>

#include <pcl/PolygonMesh.h>

#include <YukariCloudCapture/ICloudGrabber.h>

namespace Yukari
{
namespace Algorithms
{
  class GenerateMeshFromPointCloud : public Processing::IAlgorithm
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

  protected:
    virtual void doExecute() override;

  private:
    Common::LoggingService::Logger m_logger;
  };
}
}
