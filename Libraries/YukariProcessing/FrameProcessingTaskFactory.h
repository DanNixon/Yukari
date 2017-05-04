/** @file */

#pragma once

#include <map>
#include <string>

#include <YukariCommon/MapHelpers.h>
#include <YukariCommon/StringParsers.h>
#include <YukariProcessing/TaskAppendTransformedClouds.h>
#include <YukariProcessing/TaskDownsampleCloud.h>
#include <YukariProcessing/TaskICPWorldAlignment.h>
#include <YukariProcessing/TaskNDTIncrementalAlignment.h>
#include <YukariProcessing/TaskNDTWorldAlignment.h>
#include <YukariProcessing/TaskNDTWorldSegmentAlignment.h>
#include <YukariProcessing/TaskPairAlignment.h>
#include <YukariProcessing/TaskSaveRawCloud.h>
#include <YukariProcessing/TaskSaveRawIMUFrame.h>

namespace Yukari
{
namespace Processing
{
  class FrameProcessingTaskFactory
  {
  public:
    static IFrameProcessingTask::Ptr Create(const std::string &fullCommand,
                          const boost::filesystem::path &rootOutputDirectory)
    {
      std::string type;
      std::map<std::string, std::string> params;
      if (!Common::StringParsers::ParseCommand(fullCommand, type, params))
        return nullptr;

      return Create(type, params, rootOutputDirectory);
    }

    static IFrameProcessingTask::Ptr Create(const std::string &type, std::map<std::string, std::string> &parameters,
                          const boost::filesystem::path &rootOutputDirectory)
    {
      std::string lowerType = type;
      Common::StringParsers::CleanString(lowerType);

      /* Get output directory */
      boost::filesystem::path outDir =
          rootOutputDirectory / Common::MapHelpers::Get<std::string, std::string>(parameters, "out", ".");

      /* Create task */
      IFrameProcessingTask::Ptr task;
      if (lowerType == "appendtransformed")
      {
        task = std::make_shared<TaskAppendTransformedClouds>(outDir);
      }
      else if (lowerType == "downsample")
      {
        task = std::make_shared<TaskDownsampleCloud>(outDir, parameters);
      }
      else if (lowerType == "icpworld")
      {
        task = std::make_shared<TaskICPWorldAlignment>(outDir, parameters);
      }
      else if (lowerType == "ndtincremental")
      {
        task = std::make_shared<TaskNDTIncrementalAlignment>(outDir, parameters);
      }
      else if (lowerType == "ndtworld")
      {
        task = std::make_shared<TaskNDTWorldAlignment>(outDir, parameters);
      }
      else if (lowerType == "ndtworldsegment")
      {
        task = std::make_shared<TaskNDTWorldSegmentAlignment>(outDir, parameters);
      }
      else if (lowerType == "pairalign")
      {
        task = std::make_shared<TaskPairAlignment>(outDir, parameters);
      }
      else if (lowerType == "savecloud")
      {
        std::string transformParam =
          Common::MapHelpers::Get<std::string, std::string>(parameters, "transform", "false");
        Common::StringParsers::CleanString(transformParam);
        bool transform = transformParam == "true";

        task = std::make_shared<TaskSaveRawCloud>(outDir, transform);
      }
      else if (lowerType == "saveimu")
      {
        task = std::make_shared<TaskSaveRawIMUFrame>(outDir);
      }

      return task;
    }
  };
}
}
