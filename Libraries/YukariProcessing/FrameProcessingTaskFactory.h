/** @file */

#pragma once

#include <map>
#include <string>

#include <YukariCommon/MapHelpers.h>
#include <YukariCommon/StringParsers.h>
#include <YukariProcessing/TaskAppendTransformedClouds.h>
#include <YukariProcessing/TaskNDTIncrementalAlignment.h>
#include <YukariProcessing/TaskNDTWorldAlignment.h>
#include <YukariProcessing/TaskNDTWorldSegmentAlignment.h>
#include <YukariProcessing/TaskSaveRawCloud.h>
#include <YukariProcessing/TaskSaveRawIMUFrame.h>

namespace Yukari
{
namespace Processing
{
  template <typename POINT_TYPE> class FrameProcessingTaskFactory
  {
  public:
    typedef typename IFrameProcessingTask<POINT_TYPE>::Ptr TaskPtr;

  public:
    static TaskPtr Create(const std::string &fullCommand,
                          const boost::filesystem::path &rootOutputDirectory)
    {
      std::string type;
      std::map<std::string, std::string> params;
      if (!StringParsers::ParseCommand(fullCommand, type, params))
        return nullptr;

      return Create(type, params, rootOutputDirectory);
    }

    static TaskPtr Create(const std::string &type, std::map<std::string, std::string> &parameters,
                          const boost::filesystem::path &rootOutputDirectory)
    {
      std::string lowerType = type;
      StringParsers::CleanString(lowerType);

      /* Get output directory */
      boost::filesystem::path outDir =
          rootOutputDirectory / MapHelpers::Get<std::string, std::string>(parameters, "out", ".");

      /* Create task */
      TaskPtr task;
      if (lowerType == "appendtransformed")
      {
        task = std::make_shared<TaskAppendTransformedClouds<POINT_TYPE>>(outDir);
      }
      else if (lowerType == "ndtincremental")
      {
        std::string saveTransformParam =
            MapHelpers::Get<std::string, std::string>(parameters, "transform", "true");
        StringParsers::CleanString(saveTransformParam);
        bool saveTransforms = saveTransformParam == "true";

        std::string saveCloudParam =
            MapHelpers::Get<std::string, std::string>(parameters, "cloud", "false");
        StringParsers::CleanString(saveCloudParam);
        bool saveClouds = saveCloudParam == "true";

        task = std::make_shared<TaskNDTIncrementalAlignment<POINT_TYPE>>(
            outDir, parameters, saveTransforms, saveClouds);
      }
      else if (lowerType == "ndtworld")
      {
        task = std::make_shared<TaskNDTWorldAlignment<POINT_TYPE>>(outDir, parameters);
      }
      else if (lowerType == "ndtworldsegment")
      {
        task = std::make_shared<TaskNDTWorldSegmentAlignment<POINT_TYPE>>(outDir, parameters);
      }
      else if (lowerType == "savecloud")
      {
        std::string transformParam =
            MapHelpers::Get<std::string, std::string>(parameters, "transform", "false");
        StringParsers::CleanString(transformParam);
        bool transform = transformParam == "true";

        task = std::make_shared<TaskSaveRawCloud<POINT_TYPE>>(outDir, transform);
      }
      else if (lowerType == "saveimu")
      {
        task = std::make_shared<TaskSaveRawIMUFrame<POINT_TYPE>>(outDir);
      }

      return task;
    }
  };
}
}
