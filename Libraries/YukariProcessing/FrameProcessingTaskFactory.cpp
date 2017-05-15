/** @file */

#include "FrameProcessingTaskFactory.h"

#include <YukariCommon/MapHelpers.h>
#include <YukariCommon/StringParsers.h>

#include "TaskAppendTransformedClouds.h"
#include "TaskDownsampleCloud.h"
#include "TaskFeatureIncrementalAlignment.h"
#include "TaskICPIncrementalAlignment.h"
#include "TaskICPWorldAlignment.h"
#include "TaskNDTICPIncrementalAlignment.h"
#include "TaskNDTICPWorldAlignment.h"
#include "TaskNDTIncrementalAlignment.h"
#include "TaskNDTWorldAlignment.h"
#include "TaskPairIncrementalAlignment.h"
#include "TaskPairWorldAlignment.h"
#include "TaskSaveRawCloud.h"
#include "TaskSaveRawIMUFrame.h"

using namespace Yukari::Common;

namespace Yukari
{
namespace Processing
{
  IFrameProcessingTask::Ptr
  FrameProcessingTaskFactory::Create(const std::string &fullCommand,
                                     const boost::filesystem::path &rootOutputDirectory)
  {
    std::string type;
    std::map<std::string, std::string> params;
    if (!StringParsers::ParseCommand(fullCommand, type, params))
      return nullptr;

    return Create(type, params, rootOutputDirectory);
  }

  IFrameProcessingTask::Ptr
  FrameProcessingTaskFactory::Create(const std::string &type,
                                     std::map<std::string, std::string> &parameters,
                                     const boost::filesystem::path &rootOutputDirectory)
  {
    std::string lowerType = type;
    StringParsers::CleanString(lowerType);

    /* Get output directory */
    boost::filesystem::path outDir =
        rootOutputDirectory / MapHelpers::Get<std::string, std::string>(parameters, "out", ".");

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
    else if (lowerType == "featureincremental")
    {
      task = std::make_shared<TaskFeatureIncrementalAlignment>(outDir, parameters);
    }
    else if (lowerType == "icpincremental")
    {
      task = std::make_shared<TaskICPIncrementalAlignment>(outDir, parameters);
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
    else if (lowerType == "ndticpincremental")
    {
      task = std::make_shared<TaskNDTICPIncrementalAlignment>(outDir, parameters);
    }
    else if (lowerType == "ndticpworld")
    {
      task = std::make_shared<TaskNDTICPWorldAlignment>(outDir, parameters);
    }
    else if (lowerType == "pairalign")
    {
      task = std::make_shared<TaskPairWorldAlignment>(outDir, parameters);
    }
    else if (lowerType == "pairalignincremental")
    {
      task = std::make_shared<TaskPairIncrementalAlignment>(outDir, parameters);
    }
    else if (lowerType == "savecloud")
    {
      std::string transformParam =
          MapHelpers::Get<std::string, std::string>(parameters, "transform", "false");
      StringParsers::CleanString(transformParam);
      bool transform = transformParam == "true";

      task = std::make_shared<TaskSaveRawCloud>(outDir, transform);
    }
    else if (lowerType == "saveimu")
    {
      task = std::make_shared<TaskSaveRawIMUFrame>(outDir);
    }

    return task;
  }
}
}
