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

#include <boost/filesystem.hpp>

#include "../../../include/core/SegParams.h"
#include "core/MicroscopeProperties.h"

using namespace std;
using namespace pcl;
using namespace cv;

namespace fs = boost::filesystem;

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> Cloud;
typedef typename Cloud::Ptr CloudPtr;

typedef pcl::PointCloud<PointNormal> PointCloudWithNormals;

void printUsage() {
	cout << "This program performs segmentation on a 3D point " << endl
			<< "cloud. It takes one parameter, a path to a " << endl
			<< "configuration ini file."
			<< endl << endl
			<< "Usage: " << endl
			<< "<pathToIni>"
			<< endl << endl
			<< SegParams::getDescription()
			<< endl;
}

/**
 *
 */
std::vector<PointIndices> segment(CloudPtr cloud,
		SegParams & segParams) {

	// Creating the KdTree object for the search method of the extraction
	search::KdTree<PointT>::Ptr tree(new search::KdTree<PointT>);
	tree->setInputCloud(cloud);

	std::vector<PointIndices> cluster_indices;

	if (!segParams.hasParam(SegParams::SEG_TYPE)
			|| segParams.getValue(SegParams::SEG_TYPE)
					== SegParams::ST_EUCLIDEAN) {

		EuclideanClusterExtraction<PointT> ec;
		if (segParams.hasParam(SegParams::EUC_CLUSTER_TOLERANCE)) {
			ec.setClusterTolerance(
					segParams.getDouble(SegParams::EUC_CLUSTER_TOLERANCE));
		} else {
			ec.setClusterTolerance(10);
		}
		if (segParams.hasParam(SegParams::MIN_CLUSTER_SIZE)) {
			ec.setMinClusterSize(
					segParams.getDouble(SegParams::MIN_CLUSTER_SIZE));
		} else {
			ec.setMinClusterSize(5);
		}
		if (segParams.hasParam(SegParams::MAX_CLUSTER_SIZE)) {
			ec.setMaxClusterSize(
					segParams.getDouble(SegParams::MAX_CLUSTER_SIZE));
		}

		ec.setSearchMethod(tree);
		ec.setInputCloud(cloud);
		ec.extract(cluster_indices);
	} else if (segParams.getValue(SegParams::SEG_TYPE)
					== SegParams::ST_VOXEL) {

	} else {
		cout << "Unknown segmentation type: "
				<< segParams.getValue(SegParams::SEG_TYPE) << endl;
		cout << "Accepted segmentation types: " << endl;
		cout << SegParams::ST_EUCLIDEAN << endl;
		cout << SegParams::ST_VOXEL << endl;
	}

	return cluster_indices;
}


/**
 *
 */
int main(const int argc, const char **argv) {
	if (argc != 2) {
		cout << "Expected 1 inputs, received " << (argc - 1) << endl << endl;
		printUsage();
		return 0;
	}

	SegParams segParams;
	if (!segParams.loadParameters(argv[1])) {
		cout << "Failed loading segmentation parameters!" << endl;
		return 0;
	}
	cout << segParams << endl;

	if (!segParams.hasParam(SegParams::INPUT_CLOUD)) {
		cout << "Input cloud not specified!" << endl;
		printUsage();
		return 0;
	}
	if (!segParams.hasParam(SegParams::OUTPUT_DIR)) {
		cout << "Output dir not specified!" << endl;
		printUsage();
		return 0;
	}
	if (!segParams.hasParam(SegParams::SCOPE_PROPERTIES)) {
		cout << "Microscope properties not specified!" << endl;
		printUsage();
		return 0;
	}

	string inFile = segParams.getValue(SegParams::INPUT_CLOUD);
	string outPath = segParams.getValue(SegParams::OUTPUT_DIR);
	string runName;
	if (segParams.hasParam(SegParams::RUN_NAME)) {
		runName = segParams.getValue(SegParams::RUN_NAME);
	}

	// Ensure output dir exists
	fs::path boostOutPath(outPath);
	if (!fs::exists(boostOutPath)) {
		fs::create_directory(boostOutPath);
	}

	MicroscopeProperties scopeProps;
	if (!scopeProps.readFromXML(
			segParams.getValue(SegParams::SCOPE_PROPERTIES))) {
		cout << "Failed loading microscope properties!" << endl;
		return 0;
	}

	cout << scopeProps << endl;

	CloudPtr cloud(new Cloud);
	io::loadPLYFile (inFile.c_str(), *cloud);

	cout << "Num input points: " << cloud->size() << endl;

	PointT minVals, maxVals;
	getMinMax3D(*cloud, minVals, maxVals);
	cout << "Min Vals: " << minVals << endl;
	cout << "Max Vals: " << maxVals << endl;
	int numCols = scopeProps.imageWidth;
	int numRows = scopeProps.imageHeight;

	cout << "Segmenting..." << endl;
	StopWatch time;
	std::vector<PointIndices> cluster_indices = segment(cloud, segParams);
	double runTime = time.getTime();

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
			segImage.at<uchar>(row, col) = (uchar) ((currCluster + 1)
					* maskScale);
		}
		cloud_cluster->width = cloud_cluster->points.size();
		cloud_cluster->height = 1;
		cloud_cluster->is_dense = true;

		std::stringstream ss;
		ss << outPath << "/" << runName << "_cloud_cluster_" << currCluster << ".ply";
		io::savePLYFile(ss.str(), *cloud_cluster, false);

		cout << "\t\tCluster " << currCluster << ", num pts: "
				<< cloud_cluster->points.size() << endl;
		currCluster++;
	}
	Mat colorClusterMap;
	applyColorMap(segImage, colorClusterMap, COLORMAP_JET);
	std::stringstream ss;
	ss << outPath << "/" << runName << "_segMask.png";
	imwrite(ss.str(), colorClusterMap);
}




