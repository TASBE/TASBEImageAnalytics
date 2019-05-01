/*
  Copyright (C) 2011 - 2019, Raytheon BBN Technologies and contributors listed
  in the AUTHORS file in TASBE Flow Analytics distribution's top directory.

  This file is part of the TASBE Flow Analytics package, and is distributed
  under the terms of the GNU General Public License, with a linking
  exception, as described in the file LICENSE in the TASBE Image Analysis
  package distribution's top directory.
*/

/*
 * This file was modified from an original version within PCL.
 *
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2012, Willow Garage, Inc.
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
 */

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/hdl_grabber.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/parse.h>

using namespace std;
using namespace pcl::console;
using namespace pcl::visualization;

class SimpleHDLViewer
{
  public:
    typedef pcl::PointCloud<pcl::PointXYZI> Cloud;
    typedef typename Cloud::ConstPtr CloudConstPtr;

    SimpleHDLViewer (pcl::Grabber& grabber,
        pcl::visualization::PointCloudColorHandler<pcl::PointXYZI> &handler) :
        cloud_viewer_ (new pcl::visualization::PCLVisualizer ("PCL HDL Cloud")),
        grabber_ (grabber),
        handler_ (handler)
    {
    }

    void cloud_callback (const CloudConstPtr& cloud)
    {
      boost::mutex::scoped_lock lock (cloud_mutex_);
      cloud_ = cloud;
      cout << "Timestamp: " << (cloud->header.stamp & 0x00000000ffffffffl) << endl;
    }

    void run ()
    {
      cloud_viewer_->addCoordinateSystem (3.0, "coord1", 0);
      cloud_viewer_->setBackgroundColor (0, 0, 0);
      cloud_viewer_->initCameraParameters ();
      cloud_viewer_->setCameraPosition (0.0, 0.0, 30.0, 0.0, 1.0, 0.0, 0);
      cloud_viewer_->setCameraClipDistances (0.0, 50.0);

      boost::function<void (const CloudConstPtr&)> cloud_cb = boost::bind (
          &SimpleHDLViewer::cloud_callback, this, _1);
      boost::signals2::connection cloud_connection = grabber_.registerCallback (
          cloud_cb);

      grabber_.start ();

      while (!cloud_viewer_->wasStopped ())
      {
        CloudConstPtr cloud;

        // See if we can get a cloud
        if (cloud_mutex_.try_lock ())
        {
          cloud_.swap (cloud);
          cloud_mutex_.unlock ();
        }

        if (cloud)
        {
          handler_.setInputCloud (cloud);
          if (!cloud_viewer_->updatePointCloud (cloud, handler_, "HDL"))
            cloud_viewer_->addPointCloud (cloud, handler_, "HDL");

          cloud_viewer_->spinOnce ();
        }

        if (!grabber_.isRunning ())
          cloud_viewer_->spin ();

        boost::this_thread::sleep (boost::posix_time::microseconds (100));
      }

      grabber_.stop ();

      cloud_connection.disconnect ();
    }

    boost::shared_ptr<pcl::visualization::PCLVisualizer> cloud_viewer_;

    pcl::Grabber& grabber_;
    boost::mutex cloud_mutex_;

    CloudConstPtr cloud_;
    pcl::visualization::PointCloudColorHandler<pcl::PointXYZI> &handler_;
};

int main (int argc, char ** argv)
{
  std::string hdlCalibration, pcapFile;

  parse_argument (argc, argv, "-calibrationFile", hdlCalibration);
  parse_argument (argc, argv, "-pcapFile", pcapFile);

  pcl::HDLGrabber grabber (hdlCalibration, pcapFile);

  pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> color_handler ("intensity");

  SimpleHDLViewer v (grabber, color_handler);
  v.run ();
  return (0);
}
