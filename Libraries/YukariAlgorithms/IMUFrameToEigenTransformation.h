/** @file */

#pragma once

#include <YukariProcessing/IAlgorithm.h>

#include <Eigen/Geometry>
#include <YukariIMU/IMUFrame.h>

namespace Yukari
{
namespace Algorithms
{
  class IMUFrameToEigenTransformation : public Processing::IAlgorithm
  {
  public:
    static Eigen::Matrix4f Convert(const IMU::IMUFrame_sptr frame);

  public:
    IMUFrameToEigenTransformation();

    virtual void execute() override;

  private:
    Common::LoggingService::Logger m_logger;
  };
}
}
