/** @file */

#pragma once

#include "ITaskAlignment.h"

#include <pcl/point_representation.h>

#include <YukariCommon/LoggingService.h>

namespace Yukari
{
namespace Processing
{
  class RegistrationPointRepresentation : public pcl::PointRepresentation<pcl::PointNormal>
  {
    using pcl::PointRepresentation<pcl::PointNormal>::nr_dimensions_;

  public:
    RegistrationPointRepresentation()
    {
      nr_dimensions_ = 4;
    }

    virtual void copyToFloatArray(const pcl::PointNormal &p, float *out) const
    {
      out[0] = p.x;
      out[1] = p.y;
      out[2] = p.z;
      out[3] = p.curvature;
    }
  };

  class TaskPairAlignment : public ITaskAlignment
  {
  public:
    TaskPairAlignment(const boost::filesystem::path &path,
                      std::map<std::string, std::string> &params);

    inline CloudPtr worldCloud()
    {
      return m_worldCloud;
    }

    virtual int process(Task t) override;
    virtual int onStop() override;

  private:
    Common::LoggingService::Logger m_logger;

    CloudPtr m_worldCloud;
  };
}
}
