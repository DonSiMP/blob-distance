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

typedef pcl::PointXYZ PointType;
pcl::PointCloud<PointType>::Ptr bakCloud(new pcl::PointCloud<PointType>());
pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;

#define Plane
#ifdef Plane
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
// Create the segmentation object
pcl::SACSegmentation<pcl::PointXYZ> seg;

pcl::ModelCoefficients::Ptr oldCoefficients(new pcl::ModelCoefficients);

void initClustering_PlaneSegmentation() {
	// Optional
	seg.setOptimizeCoefficients(true);
	// Mandatory
	seg.setModelType(pcl::SACMODEL_PLANE);
	//seg.setMethodType(pcl::SAC_PROSAC);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setDistanceThreshold(0.001);
}

pcl::ModelCoefficients::ConstPtr findCluster_PlaneSegmentation(pcl::PointCloud<PointType>::Ptr cloud) {
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	seg.setInputCloud(cloud);
	seg.segment(*inliers, *coefficients);
	if (inliers->indices.size() > 0) {
		oldCoefficients = coefficients;
	}
	return oldCoefficients;
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

