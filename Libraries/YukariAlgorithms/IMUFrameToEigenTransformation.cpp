/** @file */

#include "IMUFrameToEigenTransformation.h"

using namespace Yukari::Processing;

namespace Yukari
{
namespace Algorithms
{
  Eigen::Matrix4f IMUFrameToEigenTransformation::Convert(const IMU::IMUFrame_sptr frame)
  {
    Eigen::Matrix4f out = Eigen::Matrix4f::Identity();
    out.block(0, 0, 3, 3) = frame->orientation().toEigen().toRotationMatrix();
    out.block(0, 3, 3, 1) = frame->position().toEigen();
    return out;
  }

  IMUFrameToEigenTransformation::IMUFrameToEigenTransformation()
      : m_logger(Common::LoggingService::GetLogger("IMUFrameToEigenTransformation"))
  {
    /* Add default validator */
    m_validator = [](const PropertyContainer &inProps, const PropertyContainer &) {
      auto frames = inProps.find("frames");
      if (frames == inProps.end())
        return "Frame property not provided";

      if (frames->second.size() == 0)
        return "Must provide at least one frame";

      return "";
    };
  }

  void IMUFrameToEigenTransformation::execute()
  {
    Property frames = getProperty(INPUT, "frames");
    size_t len = frames.size();
    Property out(len);
    m_logger->debug("Created output property with length {}", len);

    for (size_t i = 0; i < len; i++)
    {
      m_logger->trace("Processed IMU frame {}", i);
      out[i] = Convert(frames.value<IMU::IMUFrame_sptr>(i));
    }

    setProperty(OUTPUT, "transformation", out);
  }
}
}
