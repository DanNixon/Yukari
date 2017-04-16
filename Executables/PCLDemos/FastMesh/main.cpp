/*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2011, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of Willow Garage, Inc. nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Modified from: https://github.com/PointCloudLibrary/pcl/blob/master/apps/src/openni_fast_mesh.cpp
*/

#include <thread>
#include <chrono>
#include <mutex>

#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <pcl/io/openni2_grabber.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/surface/organized_fast_mesh.h>
#include <pcl/visualization/cloud_viewer.h>

using namespace pcl;
using namespace pcl::visualization;

template <typename PointType> class OpenNIFastMesh
{
public:
  typedef pcl::PointCloud<PointType> Cloud;
  typedef typename Cloud::Ptr CloudPtr;
  typedef typename Cloud::ConstPtr CloudConstPtr;

  OpenNIFastMesh()
  {
    m_ofm.setTrianglePixelSize(1);
    m_ofm.setTriangulationType(pcl::OrganizedFastMesh<PointType>::TRIANGLE_ADAPTIVE_CUT);
  }

  void cloud_cb(CloudConstPtr cloud)
  {
    // Prepare input
    m_ofm.setInputCloud(cloud);

    // Store the results in a temporary object
    pcl::PolygonMeshPtr mesh(new pcl::PolygonMesh);
    m_ofm.reconstruct(*mesh);

    // Lock and copy
    {
      std::lock_guard<std::mutex> lock(m_mtx);

      m_mesh.swap(mesh);
      m_cloud.swap(cloud);
    }
  }

  void run(int argc, char **argv)
  {
    pcl::Grabber *interface = new pcl::io::OpenNI2Grabber();

    boost::function<void(const CloudConstPtr &)> f =
      boost::bind(&OpenNIFastMesh::cloud_cb, this, _1);
    boost::signals2::connection c = interface->registerCallback(f);

    m_view.reset(new pcl::visualization::PCLVisualizer(argc, argv, "PCL OpenNI Mesh Viewer"));

    interface->start();

    CloudConstPtr cloud;
    pcl::PolygonMeshPtr mesh;

    while (!m_view->wasStopped())
    {
      m_view->spinOnce();

      if (!m_cloud || !m_mtx.try_lock())
      {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        continue;
      }

      cloud.swap (m_cloud);
      mesh.swap (m_mesh);
      
      m_mtx.unlock();

      //if (!m_view->updatePolygonMesh<PointType>(cloud, mesh->polygons, "surface"))
      //{
        m_view->removePolygonMesh ("surface");
        m_view->addPolygonMesh<PointType>(cloud, mesh->polygons, "surface");
      //}
    }

    interface->stop();
  }

  pcl::OrganizedFastMesh<PointType> m_ofm;

  std::mutex m_mtx;

  CloudConstPtr m_cloud;
  pcl::PolygonMesh::Ptr m_mesh;

  std::shared_ptr<pcl::visualization::PCLVisualizer> m_view;
};

int main(int argc, char **argv)
{
  OpenNIFastMesh<pcl::PointXYZRGBA> v;
  v.run(argc, argv);

  return 0;
}