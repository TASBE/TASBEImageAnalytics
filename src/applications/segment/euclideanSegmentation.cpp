/*
 *
 */

#include <iostream>
#include <fstream>
#include <stdio.h>
#include <sstream>
#include <string>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <pcl/common/time.h>
#include <pcl/common/impl/common.hpp>

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/impl/extract_clusters.hpp>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "core/MicroscopeProperties.h"

using namespace std;
using namespace pcl;
using namespace cv;

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> Cloud;
typedef typename Cloud::Ptr CloudPtr;

typedef pcl::PointCloud<PointNormal> PointCloudWithNormals;

Cloud::ConstPtr cloud_(new Cloud);
Cloud::Ptr prevCloud;
Cloud::Ptr totalCloud;

void printUsage() {
	cout << "This program takes an input point cloud and a location to " << endl
			<< "microscope properties, and then segments the point " << endl
			<< " cloud using Euclidean cluster extraction.  Outputs " << endl
			<< " are saved in the output director."
			<< endl << endl
			<< "Usage: " << endl
			<< "<inputCloud> <Properties XML> <output_dir>"
			<< endl;
}

/**
 *
 */
int main(const int argc, const char **argv) {
	if (argc != 4) {
		cout << "Expected 3 inputs, received " << (argc - 1) << endl << endl;
		printUsage();
		return 0;
	}

	string inFile = argv[1];
	string outPath = argv[3];

	MicroscopeProperties scopeProps;
	scopeProps.readFromXML(argv[2]);

	cout << "Processing " << inFile << endl;
	cout << "Outputting to " << outPath << endl;
	cout << scopeProps << endl;

	CloudPtr cloud (new Cloud);
	io::loadPLYFile (inFile.c_str(), *cloud);

	cout << "Num input points: " << cloud->size() << endl;

	PointT minVals, maxVals;
	getMinMax3D(*cloud, minVals, maxVals);
	cout << "Min Vals: " << minVals << endl;
	cout << "Max Vals: " << maxVals << endl;
	//int numCols = (int)(maxVals.x / pixelSize);
	//int numRows = (int)(maxVals.y / pixelSize);
	int numCols = scopeProps.imageWidth;
	int numRows = scopeProps.imageHeight;

	cout << "Computing kdTree..." << endl;
	StopWatch time;
	// Creating the KdTree object for the search method of the extraction
	search::KdTree<PointT>::Ptr tree(new search::KdTree<PointT>);
	tree->setInputCloud(cloud);
	double runTime = time.getTime();
	cout << "\t Computed kdTree in " << runTime << "ms!" << endl;

	cout << "Segmenting..." << endl;
	time.reset();
	std::vector<PointIndices> cluster_indices;
	EuclideanClusterExtraction<PointT> ec;
	ec.setClusterTolerance(10);
	ec.setMinClusterSize(5);
	//ec.setMaxClusterSize(25000);
	ec.setSearchMethod(tree);
	ec.setInputCloud(cloud);
	ec.extract(cluster_indices);
	runTime = time.getTime();
	int numClusters = cluster_indices.size();
	int currCluster = 0;
	int maskScale = 255 / numClusters;
	cout << "\tSegmented cloud in " << runTime << "ms!" << endl;
	cout << "\tNum clusters: " << numClusters << endl;
	cout << "\tCluster Sizes: " << numClusters << endl;
	Mat segImage(numRows, numCols, CV_8UC1, Scalar(0));
	for (auto it = cluster_indices.begin(); it != cluster_indices.end(); ++it) {
		CloudPtr cloud_cluster(new Cloud);
		for (auto pit = it->indices.begin(); pit != it->indices.end(); ++pit) {
			PointT & pt = cloud->points[*pit];
			cloud_cluster->points.push_back(pt);
			int col = (int)(pt.x / scopeProps.pixelWidth);
			int row = (int)(pt.y / scopeProps.pixelHeight);
			segImage.at<uchar>(row,col) = (uchar)((currCluster + 1) * maskScale);
		}
		cloud_cluster->width = cloud_cluster->points.size();
		cloud_cluster->height = 1;
		cloud_cluster->is_dense = true;

		std::stringstream ss;
		ss << outPath << "/" << "cloud_cluster_" << currCluster << ".ply";
		io::savePLYFile(ss.str(), *cloud_cluster, false);

		cout << "\t\tCluster " << currCluster << ", num pts: " << cloud_cluster->points.size() << endl;
		currCluster++;
	}
	Mat colorClusterMap;
	applyColorMap(segImage, colorClusterMap, COLORMAP_JET);
	std::stringstream ss;
	ss << outPath << "/" << "segMask.png";
	imwrite(ss.str(), colorClusterMap);
}




