/** @file */

#include "SaveVTKMesh.h"

#include <YukariCloudCapture/ICloudGrabber.h>
#include <pcl/io/vtk_io.h>

using namespace Yukari::CloudCapture;
using namespace Yukari::Processing;

namespace Yukari
{
namespace Algorithms
{
  SaveVTKMesh::SaveVTKMesh()
      : m_logger(Common::LoggingService::GetLogger("SaveVTKMesh"))
  {
    /* Add default validator */
    m_validator = [](const IAlgorithm &alg) {
      Property_sptr mesh = alg.getProperty(Processing::INPUT, "mesh");
      Property_sptr filename = alg.getProperty(Processing::INPUT, "filename");

      if (mesh->size() != filename->size())
        return "Must provide equal numbers of meshes and filenames";

      return "";
    };
  }

  void SaveVTKMesh::doExecute()
  {
    Property_sptr mesh = getProperty(Processing::INPUT, "mesh");
    Property_sptr filename = getProperty(Processing::INPUT, "filename");

    size_t len = mesh->size();

    for (size_t i = 0; i < len; i++)
    {
      pcl::PolygonMesh::Ptr m = mesh->value<pcl::PolygonMesh::Ptr>(i);
      const std::string f = filename->value<std::string>(i);

      m_logger->trace("Savng mesh to file \"{}\"", f);
      pcl::io::saveVTKFile(f, *m);
    }
  }
}
}
