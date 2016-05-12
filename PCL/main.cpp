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

#ifdef R200_GRABBER
#include "grabber\r200_pcl_grabber.h"
#endif
#ifdef F200_GRABBER
#include "grabber\f200_pcl_grabber.h"
#endif
#ifdef KINECT2_GRABBER
#include "grabber\kinect_pcl_grabber.h"
#endif
#ifdef LEAP_GRABBER
#include "grabber\leap_pcl_grabber.h"
#endif
#ifdef DUO_GRABBER
#include "grabber\duo_pcl_grabber.h"
#endif
#include "detection-utils.h"
#include <pcl/visualization/pcl_visualizer.h>
#include <vector>
#include <pcl/filters/filter.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/visualization/cloud_viewer.h>

//typedef pcl::PointXYZRGBA PointType;
typedef pcl::PointXYZ PointType;
bool buildBack = false;
bool active = false;

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
				if (viewer->contains("cube")) {
					viewer->removeShape("cube");
				}
				if (viewer->contains("plane")) {
					viewer->removeShape("plane");
				}
				if (viewer->contains(cloudId + "_plane")) {
					viewer->removePointCloud(cloudId + "_plane");
				}
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

	boost::mutex mutex;
	boost::function<void(pcl::PointCloud<PointType>::Ptr&, pcl::PointCloud<PointType>::Ptr&)> funcRange =
		[](pcl::PointCloud<PointType>::Ptr& minRange, pcl::PointCloud<PointType>::Ptr& maxRange) {
#ifdef KINECT2_GRABBER
		minRange->points[0].x = -0.2;
		maxRange->points[0].x = 0.2;
		minRange->points[0].z = 1.;
		maxRange->points[0].z = 2.;
#endif // KINECT2_GRABBER
#ifndef KINECT2_GRABBER
		//minRange->points[0].x = -0.2;
		//maxRange->points[0].x = 0.2;
		minRange->points[0].z = .1;
		maxRange->points[0].z = 1.;
#endif
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
			pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
			pcl::ModelCoefficients::ConstPtr plane = findCluster_PlaneSegmentation(ptr, inliers);
			if (plane->values.size() == 4) {
				if (viewer->contains("plane")) {
					viewer->removeShape("plane", 0);
				}
				viewer->addPlane(*plane, "plane", 0);
				viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.9, 0.1, 0.1, "plane", 0);//R,G,B
				viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.6, "plane", 0);
				viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "plane", 0);
			}
			pcl::PointCloud<PointType>::Ptr planeCloud(new pcl::PointCloud<PointType>());
			pcl::copyPointCloud(*ptr, *inliers, *planeCloud);
			pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(planeCloud, 122, 122, 122);
			if (!viewer->updatePointCloud(planeCloud, single_color, cloudId + "_plane")) {
				viewer->addPointCloud(planeCloud, single_color, cloudId + "_plane");
			}
			float dist = calcDistance(plane);
			if (viewer->contains("cube")) {
				viewer->removeShape("cube");
			}
			std::cout << dist << " m " << std::endl;
			if (!viewer->updateText(std::to_string(dist), 0, 50, "dist")) {
				viewer->addText(std::to_string(dist), 0, 50, "dist");
			}

			float middlPnt[] = {
				startPoint[0] + dist * measureVector[0],// X
				startPoint[1] + dist * measureVector[1],// Y
				startPoint[2] + dist * measureVector[2],// Z
			};
			viewer->addCube(middlPnt[0] - .05, middlPnt[0] + .05, middlPnt[1] - .05, middlPnt[1] + .05, middlPnt[2] - .05, middlPnt[2] + .05);
			viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.9, 0.9, 0.1, "cube", 0);//R,G,B
			viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.6, "cube", 0);
			viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "cube", 0);
#endif
		}
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(ptr, 0, 255, 0);
		std::vector<int> indices;
		pcl::removeNaNFromPointCloud(*ptr, *ptr, indices);
		if (ptr->size() > 100 && !viewer->updatePointCloud(ptr, single_color, cloudId)) {
			viewer->addPointCloud(ptr, single_color, cloudId);
		}
		updated = true;
	};

	// Kinect2Grabber
	boost::shared_ptr<pcl::Grabber> grabber = boost::make_shared<
#ifdef R200_GRABBER
		pcl::R200Grabber
#endif
#ifdef F200_GRABBER
		pcl::F200Grabber
#endif
#ifdef LEAP_GRABBER
		pcl::LEAPGrabber
#endif
#ifdef DUO_GRABBER
		pcl::DUOGrabber
#endif
#ifdef KINECT2_GRABBER
		pcl::Kinect2Grabber
#endif
	>();


	// Register Callback Function
	boost::signals2::connection connection = grabber->registerCallback(function);
	boost::signals2::connection connectionRange = grabber->registerCallback(funcRange);

	// Start Grabber
	grabber->start();
	viewer->addCoordinateSystem(3., 0, 0, 10, 1);

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