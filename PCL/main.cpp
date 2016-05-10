/*
Copyright 2016 Chris Papenfuﬂ

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/

// Disable Error C4996 that occur when using Boost.Signals2.
#ifdef _DEBUG
#define _SCL_SECURE_NO_WARNINGS
#endif

#include <iostream>
#include "kinect_pcl_grabber.h"
#include "blob-utils.h"
#include <pcl/visualization/pcl_visualizer.h>
#include <vector>
#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>

//typedef pcl::PointXYZRGBA PointType;
typedef pcl::PointXYZ PointType;
bool buildBack = false;
bool active = false;

pcl::PassThrough<PointType> filterZ, filterY, filterX;

// PCL Visualizer
boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(
	new pcl::visualization::PCLVisualizer("Point Cloud Viewer"));

std::string cloudId = "cloud";

bool updated = false;

int main(int argc, char* argv[])
{
	viewer->setCameraPosition(0.0, 0.0, -2.5, 0.0, 0.0, 0.0);
	boost::function<void(const pcl::visualization::KeyboardEvent&)> keyPressedFunc =
		[](const pcl::visualization::KeyboardEvent& cb) {
		if (cb.getKeySym() == "a" && cb.keyDown()) {
#ifdef Region
			initClustering_RegionGrowing();
#endif
#ifdef Plane
			initClustering_PlaneSegmentation();
#endif
			if (active) {
				active = false;
			}
			else {
				active = true;
			}
		}
		else if (cb.getKeySym() == "b" && cb.keyDown()) {
			if (bakCloud->size() > 0) {
				viewer->removePointCloud("bak");
			}
			else {
				bakCloud->clear();
				buildBack = true;
			}
		}
	};
	viewer->registerKeyboardCallback(keyPressedFunc);

	// Point Cloud
	pcl::PointCloud<PointType>::Ptr cloud;

	//configure Filter
	filterZ.setFilterFieldName("z");
	filterZ.setFilterLimits(1., 3.5);
	filterX.setFilterFieldName("x");
	filterX.setFilterLimits(1., 1);
	filterY.setFilterFieldName("y");
	filterY.setFilterLimits(-.5, .5);

	boost::mutex mutex;
	boost::function<void(pcl::PointCloud<PointType>::Ptr&, pcl::PointCloud<PointType>::Ptr&)> funcRange =
		[](pcl::PointCloud<PointType>::Ptr& minRange, pcl::PointCloud<PointType>::Ptr& maxRange) {
		minRange->points[0].x = -0.2;
		maxRange->points[0].x = 0.2;
		minRange->points[0].z = 1.;
		maxRange->points[0].z = 2.5;
		//minRange->points[0].y = -.3;
		//maxRange->points[0].y = .0;
	};

	// Retrieved Point Cloud Callback Function
	boost::function<void(const pcl::PointCloud<PointType>::Ptr&)> function =
		[&cloud, &mutex](const pcl::PointCloud<PointType>::Ptr& ptr) {
		if (ptr->points.size() == 0) return;
		boost::mutex::scoped_lock lock(mutex);
		//cloud = ptr;
		if (buildBack) {
			buildBackground(ptr);
			pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(bakCloud, 0, 0, 255);
			if (!viewer->updatePointCloud(bakCloud, single_color, "bak")) {
				viewer->addPointCloud(bakCloud, single_color, "bak");
			}
			buildBack = false;
		}
		filterBackground(ptr);
		if (active) {
			cout << "active!" << endl;

#ifdef Region
			pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = findCluster_RegionGrowing(ptr);
			if (!viewer->updatePointCloud(colored_cloud, cloudId)) {
				viewer->addPointCloud(colored_cloud, cloudId);
			}
#endif
#ifdef Plane
			pcl::ModelCoefficients::ConstPtr plane = findCluster_PlaneSegmentation(ptr);
			if (plane->values.size() == 4) {
				if (viewer->contains("plane_1")) {
					viewer->removeShape("plane_1", 0);
				}
				viewer->addPlane(*plane, "plane_1", 0);
				viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.9, 0.1, 0.1 /*R,G,B*/, "plane_1", 0);
				viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.6, "plane_1", 0);
				viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "plane_1", 0);
			}
#endif
		}
		else {
			pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(ptr, active ? 255 : 0, active ? 0 : 255, 0);
			if (!viewer->updatePointCloud(ptr, single_color, cloudId)) {
				viewer->addPointCloud(ptr, single_color, cloudId);
			}
		}
		updated = true;
	};

	// Kinect2Grabber
	boost::shared_ptr<pcl::Grabber> grabber = boost::make_shared<pcl::Kinect2Grabber>();


	// Register Callback Function
	boost::signals2::connection connection = grabber->registerCallback(function);
	boost::signals2::connection connectionRange = grabber->registerCallback(funcRange);

	// Start Grabber
	grabber->start();


	while (!viewer->wasStopped()) {
		boost::mutex::scoped_try_lock lock(mutex);
		if (updated && lock.owns_lock()) {
			updated = false;
			viewer->spinOnce();
		}
	}

	// Stop Grabber
	grabber->stop();

	return 0;
}