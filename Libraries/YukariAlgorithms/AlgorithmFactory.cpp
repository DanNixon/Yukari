/** @file */

#include "AlgorithmFactory.h"

#include <boost/algorithm/string/case_conv.hpp>
#include <boost/algorithm/string/trim.hpp>

#include "ApplyTransformationToCloud.h"
#include "ConcatenatePointClouds.h"
#include "GenerateMeshFromPointCloud.h"
#include "IMUFrameToEigenTransformation.h"
#include "LoadPointCloud.h"
#include "RemoveNaNFromPointCloud.h"
#include "SaveVTKMesh.h"
#include "TriangulatePointCloud.h"

using namespace Yukari::Processing;

namespace Yukari
{
namespace Algorithms
{
  IAlgorithm_sptr AlgorithmFactory::Create(std::string name)
  {
    /* Clean name */
    boost::algorithm::trim(name);
    boost::algorithm::to_lower(name);

    /* Create algorithm */
    IAlgorithm_sptr alg;
    if (name == "applytransformationtocloud")
      alg = std::make_shared<ApplyTransformationToCloud>();
    else if (name == "concatenatepointclouds")
      alg = std::make_shared<ConcatenatePointClouds>();
    else if (name == "generatemeshfrompointcloud")
      alg = std::make_shared<GenerateMeshFromPointCloud>();
    else if (name == "imuframetoeigentransformation")
      alg = std::make_shared<IMUFrameToEigenTransformation>();
    else if (name == "loadpointcloud")
      alg = std::make_shared<LoadPointCloud>();
    else if (name == "removenanfrompointcloud")
      alg = std::make_shared<RemoveNaNFromPointCloud>();
    else if (name == "savevtkmesh")
      alg = std::make_shared<SaveVTKMesh>();
    else if (name == "triangulatepointcloud")
      alg = std::make_shared<TriangulatePointCloud>();

    return alg;
  }

  IAlgorithm_sptr AlgorithmFactory::Create(const std::string &name,
                                           std::vector<std::string>::const_iterator begin,
                                           std::vector<std::string>::const_iterator end)
  {
    IAlgorithm_sptr alg = Create(name);

    // TODO

    return alg;
  }
}
}
