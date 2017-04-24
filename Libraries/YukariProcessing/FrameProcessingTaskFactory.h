/** @file */

#pragma once

#include <map>
#include <string>

#include <YukariCommon/MapHelpers.h>
#include <YukariCommon/StringParsers.h>
#include <YukariProcessing/TaskAppendTransformedClouds.h>
#include <YukariProcessing/TaskNDTIncrementalAlignment.h>
#include <YukariProcessing/TaskNDTWorldAlignment.h>
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
        task = std::make_shared<TaskNDTIncrementalAlignment<POINT_TYPE>>(outDir);
      }
      else if (lowerType == "ndtworld")
      {
        task = std::make_shared<TaskNDTWorldAlignment<POINT_TYPE>>(outDir);
      }
      else if (lowerType == "saveimu")
      {
        task = std::make_shared<TaskSaveRawIMUFrame<POINT_TYPE>>(outDir);
      }
      else if (lowerType == "savecloud")
      {
        std::string transformParam =
            MapHelpers::Get<std::string, std::string>(parameters, "transform", "false");
        StringParsers::CleanString(transformParam);
        bool transform = transformParam == "true";

        task = std::make_shared<TaskSaveRawCloud<POINT_TYPE>>(outDir, transform);
      }

      return task;
    }
  };
}
}
