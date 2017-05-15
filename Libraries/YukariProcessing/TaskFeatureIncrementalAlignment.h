/** @file */

#pragma once

#include "ITaskIncrementalAlignment.h"

#include <YukariCommon/LoggingService.h>

namespace Yukari
{
namespace Processing
{
  class TaskFeatureIncrementalAlignment : public ITaskIncrementalAlignment
  {
  public:
    typedef pcl::Normal NormalT;
    typedef pcl::PointCloud<NormalT> NormalCloud;
    typedef pcl::FPFHSignature33 FeatureT;
    typedef pcl::PointCloud<FeatureT> FeatureCloud;

    struct SingleCloudData
    {
      CloudPtr downsampled;
      NormalCloud::Ptr normals;
      FeatureCloud::Ptr features;
    };

  public:
    TaskFeatureIncrementalAlignment(const boost::filesystem::path &path,
                                    std::map<std::string, std::string> &params);

    virtual int process(Task t) override;

  protected:
    SingleCloudData preProcessSingleCloud(Task t);
    virtual void doAlignment(Task t) override;

  protected:
    SingleCloudData m_targetData;

  private:
    Common::LoggingService::Logger m_logger;
  };
}
}
