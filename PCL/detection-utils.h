#pragma once
/*
Copyright 2016 Chris Papenfu�

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

#include <iostream>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <vector>

typedef pcl::PointXYZ PointType;
pcl::PointCloud<PointType>::Ptr bakCloud(new pcl::PointCloud<PointType>());
pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;

/*
Kinect on Kinect-package 40 cm
R200 on F200 Package 12 cm
F200 on F200 Package 14 cm

*/
static const float startPoint[] = { 0., -0.14, 0. };//40 cm distance from sensor to ground
static const float measureVector[] = { 0., 0., 1. };

#define Plane
#ifdef Plane
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/sac_model_perpendicular_plane.h>
#include <pcl/common/angles.h>
// Create the segmentation object
pcl::SACSegmentation<pcl::PointXYZ> seg;

pcl::ModelCoefficients::Ptr oldCoefficients(new pcl::ModelCoefficients);

void initClustering_PlaneSegmentation() {
	seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);//senkrecht auf definierter Achse
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setMaxIterations(1000);
	seg.setDistanceThreshold(0.005);//max Mittlere Abstand der Punkte zur Ebene
	seg.setAxis(Eigen::Vector3f(0, 0, 1));//definierte Achse
	seg.setEpsAngle(pcl::deg2rad(10.));//Winkel in dem die Ebene liegen kann
}

pcl::ModelCoefficients::ConstPtr findCluster_PlaneSegmentation(pcl::PointCloud<PointType>::Ptr cloud, pcl::PointIndices::Ptr inliers) {
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	seg.setInputCloud(cloud);
	seg.segment(*inliers, *coefficients);
	if (inliers->indices.size() > 0) {
		oldCoefficients = coefficients;
	}
	return oldCoefficients;
}

float calcDistance(pcl::ModelCoefficients::ConstPtr plane) {
	float tmp = (plane->values[0] * measureVector[0]) + (plane->values[1] * measureVector[1]) + (plane->values[2] * measureVector[2]);
	if (tmp > 0) {
		float dist = ((-plane->values[3] - plane->values[0] * startPoint[0] - plane->values[1] * startPoint[1] - plane->values[2] * startPoint[2]) / tmp);

		if (dist > 0) {
			float intersectionPnt[] = {
				startPoint[0] + dist * measureVector[0],// X
				startPoint[1] + dist * measureVector[1],// Y
				startPoint[2] + dist * measureVector[2],// Z
			};
			return dist;
		}
	}
	return -1.;
	/*var tmp = (CollisionPlane.X * pointingVec[0] + CollisionPlane.Y * pointingVec[1] + CollisionPlane.Z * pointingVec[2]);
	if (tmp != 0)
	{

		var vecDist = (-CollisionPlane.W - CollisionPlane.X * handPos.X - CollisionPlane.Y * handPos.Y - CollisionPlane.Z * handPos.Y) /
			tmp;
		if (vecDist > 0)
		{
			isCollision = true;
			return (handVec + vecDist * pointingVec).GetSpacePoint();
		}
	}*/
}

#endif

#ifdef Region
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/region_growing_rgb.h>
pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
pcl::RegionGrowing<PointType, pcl::Normal> reg;
void initClustering_RegionGrowing() {
	pcl::search::Search <PointType>::Ptr tree = boost::shared_ptr<pcl::search::Search<PointType> >(new pcl::search::KdTree<PointType>);

	normal_estimator.setSearchMethod(tree);
	normal_estimator.setKSearch(20);

	reg.setMinClusterSize(50);
	reg.setMaxClusterSize(100000);
	reg.setSearchMethod(tree);
	reg.setNumberOfNeighbours(30);
	reg.setSmoothnessThreshold(7.0 / 180.0 * M_PI);
	reg.setCurvatureThreshold(1.0);
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr findCluster_RegionGrowing(pcl::PointCloud<PointType>::Ptr cloud) {
	pcl::PointCloud <pcl::Normal>::Ptr normals(new pcl::PointCloud <pcl::Normal>);
	normal_estimator.setInputCloud(cloud);
	normal_estimator.compute(*normals);

	reg.setInputCloud(cloud);
	reg.setInputNormals(normals);

	std::vector <pcl::PointIndices> clusters;
	reg.extract(clusters);
	return reg.getColoredCloud();
}
#endif

void buildBackground(pcl::PointCloud<PointType>::Ptr cloud) {
	bakCloud->resize(cloud->size());
	pcl::copyPointCloud(*cloud, *bakCloud);

	kdtree.setInputCloud(bakCloud);
}

void filterBackground(pcl::PointCloud<PointType>::Ptr cloud) {
	if (bakCloud->size() > 0) {
		int K = 5;
		std::vector<int> pointIdxNKNSearch(K);
		std::vector<int> indices;
		std::vector<float> pointNKNSquaredDistance(K);

		pcl::PointXYZ* pt = &cloud->points[0];
		size_t range = cloud->points.size();

		for (size_t i = 0; i < range; i++, pt++)
		{
			if (pt->x != pt->x || pt->y != pt->y || pt->z != pt->z) continue;
			if (kdtree.nearestKSearch(*pt, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0) {
				bool isShort = false;
				for (size_t d = 0; d < K && !isShort; d++)
				{
					if (pointNKNSquaredDistance[d] < .0001) {
						isShort = true;
					}
				}
				if (isShort) {
					pt->x = NAN;
					pt->y = NAN;
					pt->z = NAN;
				}
			}

		}
	}
}

