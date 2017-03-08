/** @file */

#include "AlgorithmFactory.h"

#include <boost/algorithm/string/case_conv.hpp>
#include <boost/algorithm/string/trim.hpp>

#include <YukariCommon/LoggingService.h>

/* Algorithm includes */
#include "Add.h"
#include "ApplyTransformationToCloud.h"
#include "ConcatenatePointClouds.h"
#include "GenerateMeshFromPointCloud.h"
#include "IMUFrameToEigenTransformation.h"
#include "LoadPointCloud.h"
#include "RemoveNaNFromPointCloud.h"
#include "SaveVTKMesh.h"
#include "TriangulatePointCloud.h"

using namespace Yukari::Common;
using namespace Yukari::Processing;

namespace Yukari
{
namespace Algorithms
{
  IAlgorithm_sptr AlgorithmFactory::Create(const std::string &name)
  {
    auto logger = LoggingService::GetLogger("AlgorithmFactory");
    logger->debug("Creating algorithm with name \"{}\"", name);

    /* Clean name */
    std::string cleanName(name);
    boost::algorithm::trim(cleanName);
    boost::algorithm::to_lower(cleanName);
    logger->debug("Clean algorithm name: \"{}\"", cleanName);

    /* Create algorithm */
    IAlgorithm_sptr alg;
    if (cleanName == "add")
      alg = std::make_shared<Add>();
    else if (cleanName == "applytransformationtocloud")
      alg = std::make_shared<ApplyTransformationToCloud>();
    else if (cleanName == "concatenatepointclouds")
      alg = std::make_shared<ConcatenatePointClouds>();
    else if (cleanName == "generatemeshfrompointcloud")
      alg = std::make_shared<GenerateMeshFromPointCloud>();
    else if (cleanName == "imuframetoeigentransformation")
      alg = std::make_shared<IMUFrameToEigenTransformation>();
    else if (cleanName == "loadpointcloud")
      alg = std::make_shared<LoadPointCloud>();
    else if (cleanName == "removenanfrompointcloud")
      alg = std::make_shared<RemoveNaNFromPointCloud>();
    else if (cleanName == "savevtkmesh")
      alg = std::make_shared<SaveVTKMesh>();
    else if (cleanName == "triangulatepointcloud")
      alg = std::make_shared<TriangulatePointCloud>();

    if (!alg)
      logger->error("Failed to create algorithm from name \"{}\"", name);

    return alg;
  }
}
}
