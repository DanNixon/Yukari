/** @file */

#pragma once

#include <map>
#include <string>

#include <YukariCommon/LoggingService.h>
#include <YukariCommon/MapHelpers.h>
#include <YukariCommon/StringParsers.h>

#include "DummyCloudGrabber.h"
#include "ICloudGrabber.h"
#include "OpenNI2CloudGrabber.h"
#include "PCDFileCloudGrabber.h"

namespace Yukari
{
namespace CloudCapture
{
  template <typename POINT_TYPE> class CloudGrabberFactory
  {
  public:
    typedef typename ICloudGrabber<POINT_TYPE>::Ptr GrabberPtr;

  public:
    static GrabberPtr Create(const std::string &fullCommand)
    {
      std::string type;
      std::map<std::string, std::string> params;
      if (!StringParsers::ParseCommand(fullCommand, type, params))
        return nullptr;

      return Create(type, params);
    }

    static GrabberPtr Create(const std::string &type,
                             std::map<std::string, std::string> &parameters)
    {
      std::string lowerType = type;
      StringParsers::CleanString(lowerType);

      GrabberPtr grabber;
      if (lowerType == "dummy")
      {
        size_t width =
            std::stoi(MapHelpers::Get<std::string, std::string>(parameters, "width", "10"));
        size_t height =
            std::stoi(MapHelpers::Get<std::string, std::string>(parameters, "height", "10"));

        grabber = std::make_shared<DummyCloudGrabber<POINT_TYPE>>(width, height);
      }
      else if (lowerType == "pcdfile")
      {
        /* TODO */
        grabber = std::make_shared<PCDFileCloudGrabber<POINT_TYPE>>();
      }
      else if (lowerType == "openni2")
      {
        pcl::io::OpenNI2Grabber::Mode depthMode = pcl::io::OpenNI2Grabber::Mode(
            std::stoi(MapHelpers::Get<std::string, std::string>(parameters, "depthmode", "0")));
        pcl::io::OpenNI2Grabber::Mode imageMode = pcl::io::OpenNI2Grabber::Mode(
            std::stoi(MapHelpers::Get<std::string, std::string>(parameters, "imagemode", "0")));

        try
        {
          grabber = std::make_shared<OpenNI2CloudGrabber<POINT_TYPE>>(
              MapHelpers::Get<std::string, std::string>(parameters, "device"), depthMode,
              imageMode);
        }
        catch (const pcl::IOException &e)
        {
          grabber = nullptr;
          Common::LoggingService::Instance()
              .getLogger("CloudGrabberFactory")
              ->error("Cannot create OpenNI2 grabber, IO error: {}", e.what());
        }
      }

      return grabber;
    }
  };
}
}
