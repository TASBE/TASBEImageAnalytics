/*
 *
 */

#include <iostream>
#include <fstream>
#include <stdio.h>
#include <sstream>
#include <string>
#include <algorithm>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/common/time.h>
#include <pcl/common/impl/common.hpp>

#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/impl/extract_clusters.hpp>
#include <pcl/segmentation/cpc_segmentation.h>
#include <pcl/segmentation/impl/cpc_segmentation.hpp>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>

#include "../../../include/core/SegParams.h"
#include "core/MicroscopeProperties.h"

using namespace std;
using namespace pcl;
using namespace cv;

namespace fs = boost::filesystem;

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> Cloud;
typedef Cloud::Ptr CloudPtr;

typedef pcl::PointCloud<PointNormal> PointCloudWithNormals;

enum chan {RED = 0, GREEN = 1, BLUE = 2, NUM=3};

enum DistCol {RED_DIST = 0, GREEN_DIST = 1};
string chanNames[3] = {"Red", "Green", "Blue"};

enum DistTypes {NN = 0, AVG = 1, EXP = 2};

int NUM_COLORS = 8;
// Red, Green, Blue,
Vec3b colors[8] = {
	Vec3b(  0,   0,   0), // 000 - 0 - Black
	Vec3b(  0,   0, 255), // 001 - 1 - Blue
	Vec3b(  0, 255,   0), // 010 - 2 - Green
	Vec3b(  0, 255, 255), // 011 - 3 - Blue + Green => Teal
	Vec3b(255,   0,   0), // 100 - 4 - Red
	Vec3b(255,   0, 255), // 101 - 5 - Red + Blue =>
	Vec3b(255, 255,   0), // 110 - 6 - Red + Green =>
	Vec3b(255, 255, 255)  // 111 - 7 - Red + Green + Blue => White
};


void printUsage() {
	cout << "This program processes the 3D point cloud to compute "
			<< "statistics about the relative positions of senders and "
			<< "receivers.  This version doesn't require segmentation " << endl
			<< "and computes values on each point."
			<< "It takes three parameters, each a path to a segmentation "
			<< "parameters configuration ini file for each channel."
			<< "Optionally, a distance algorithm can be specified."
			<< "One of:" << endl
			<< "NN   - Distance between the nearest neighbor" << endl
			<< "AVG  - NN dist averaged over 5 neighbors, default" << endl
			<< "EXP  - sum of Gaussian over 5 neighbors, sigma = 1" << endl
			<< endl << endl
			<< "Usage: " << endl
			<< "<redIni> <greenIni> <blueIni> [distType]"
			<< endl;
}

/**
 *
 */
double intraCloudDistance(Cloud c1, Cloud c2) {
	vector<double> distances;
	for (int p1 = 0; p1 < c1.size(); p1++) {
		for (int p2 = 0; p2 < c2.size(); p2++) {
			double xDiff = c1[p1].x - c2[p2].x;
			double yDiff = c1[p1].y - c2[p2].y;
			double zDiff = c1[p1].z - c2[p2].z;
			distances.push_back(
					sqrt(xDiff * xDiff + yDiff * yDiff + zDiff * zDiff));
		}
	}

	double result = -1;
	// Implement min distance
	for (int i = 0; i < distances.size(); i++) {
		if (result == -1 || distances[i] < result) {
			result = distances[i];
		}
	}
	return result;
}

/**
 *
 */
int main(const int argc, const char **argv) {
	if (argc != 4 && argc != 5) {
		cout << "Expected 3 or 4 inputs, received " << (argc - 1) << endl << endl;
		printUsage();
		return 0;
	}

	int distType = AVG;
	if (argc >= 5) {
		string dt = argv[4];
		if (boost::iequals(dt, "AVG")) {
			distType = AVG;
		} else if (boost::iequals(dt, "NN")) {
			distType = NN;
		} else if (boost::iequals(dt, "EXP")) {
			distType = EXP;
		} else {
			cerr << "Unexpected distance type! Value: " << dt << endl;
			return -1;
		}
	}
	cout << "Distance type is: " << distType << endl;

	// Parse input params
	SegParams chanParams[chan::NUM];
	CloudPtr chanClouds[3];
	string outPath;
	std::stringstream ss;
	for (int i = 0; i < chan::NUM; i++) {
		// Load params
		if (!chanParams[i].loadParameters(argv[i + 1])) {
			cout << "Failed loading " << chanNames[i] << " channel parameters!"
					<< endl;
			return 0;
		}
		cout << chanNames[i] << " Params: " << endl << chanParams[i] << endl;
		// Check state
		if (!chanParams[i].hasParam(SegParams::OUTPUT_DIR)) {
			cout << "Output dir not specified for channel " << chanNames[i]
					<< "!" << endl;
			printUsage();
			return 0;
		}
		if (!chanParams[i].hasParam(SegParams::SCOPE_PROPERTIES)) {
			cout << "Microscope properties not specified for channel "
					<< chanNames[i] << "!" << endl;
			printUsage();
			return 0;
		}

		outPath = chanParams[i].getValue(SegParams::OUTPUT_DIR);

		// Load cloud
		chanClouds[i].reset(new Cloud());
		io::loadPLYFile (chanParams[i].getValue(SegParams::INPUT_CLOUD), *chanClouds[i]);
		int origPts = chanClouds[i]->size();

		RadiusOutlierRemoval<PointT> outlierRemove;
		outlierRemove.setInputCloud(chanClouds[i]);
		outlierRemove.setRadiusSearch(10);
		outlierRemove.setMinNeighborsInRadius (3);
		outlierRemove.filter (*chanClouds[i]);

		cout << chanNames[i] << " cloud path: " << chanParams[i].getValue(SegParams::INPUT_CLOUD) << endl;
		cout << chanNames[i] << " cloud orig num pts: " << origPts << endl;
		cout << chanNames[i] << " cloud num pts: " << chanClouds[i]->size() << endl;
		cout << endl;
	}

	// Ensure output dir exists
	fs::path boostCloudOutPath(outPath);
	fs::path boostParentOutPath = boostCloudOutPath.parent_path();
	fs::path boostSynNotchOutPath = boostParentOutPath;
	boostSynNotchOutPath += "/SynNotchNoSegOutput";
	if (!fs::exists(boostSynNotchOutPath)) {
		fs::create_directory(boostSynNotchOutPath);
	}

	MicroscopeProperties scopeProps;
	if (!scopeProps.readFromXML(
			chanParams[chan::RED].getValue(SegParams::SCOPE_PROPERTIES))) {
		cout << "Failed loading microscope properties!" << endl;
		return 0;
	}

	cout << "Pixel Height: " << scopeProps.pixelHeight << "µm" << endl;
	cout << "Pixel Width: " << scopeProps.pixelWidth << "µm" << endl;
	cout << "Pixel Depth: " << scopeProps.pixelDepth << "µm" << endl;
	cout << "Point Volume: " << (scopeProps.pixelDepth * scopeProps.pixelHeight * scopeProps.pixelWidth) << "µm^3" << endl;


	// Visualization options for data
	PointCloud<PointT> combinedCloud;
	// Single image plane projection of point clouds
	Mat clusterProjection(scopeProps.imageHeight, scopeProps.imageWidth, CV_8UC3, Scalar(0,0,0));
	Mat clusterIntermediary(scopeProps.imageHeight, scopeProps.imageWidth, CV_8UC1, Scalar(0));
	//for (int i = chan::NUM - 1; i >= 0; i--) {
	for (int i = 0; i < chan::NUM; i++) {
		for (const PointT & pt: *chanClouds[i]) {
			combinedCloud.push_back(pt);
			int row = (int)(pt.y / scopeProps.pixelHeight + 0.5);
			int col = (int)(pt.x / scopeProps.pixelWidth + 0.5);
			clusterIntermediary.at<uchar>(row, col) |= 1 << i;
		}
	}

	for (int row = 0; row < clusterIntermediary.rows; row++) {
		for (int col = 0; col < clusterIntermediary.cols; col++) {
			uchar colorIdx = clusterIntermediary.at<uchar>(row,col);
			assert(colorIdx < NUM_COLORS);
			clusterProjection.at<Vec3b>(row,col) = colors[colorIdx];
		}
	}

	ss.str(""); ss.clear();
	ss << boostSynNotchOutPath.string() << "/allPointsProj.png";
	imwrite(ss.str(), clusterProjection);
	ss.str(""); ss.clear();
	ss << boostSynNotchOutPath.string() << "/allPoints.ply";
	io::savePLYFile(ss.str(), combinedCloud, false);
	combinedCloud.clear();
	clusterProjection.release();


	// Compute distances
	search::KdTree<PointT>::Ptr redSearch(new search::KdTree<PointT>);
	search::KdTree<PointT>::Ptr greenSearch(new search::KdTree<PointT>);
	if (chanClouds[chan::RED]->size() > 0) {
	redSearch->setInputCloud(chanClouds[chan::RED]);
	} else {
		cerr << "Found red cloud with no points!" << endl;
	}

	if (chanClouds[chan::GREEN]->size() > 0) {
		greenSearch->setInputCloud(chanClouds[chan::GREEN]);
	} else {
		cerr << "Found green cloud with no points!" << endl;
	}
	Mat blueDists(chanClouds[chan::BLUE]->size(), 2, CV_64FC1, Scalar(0));
	int numNeighbors = 5;
	int matIdx = 0;
	double sqrDistSum;
	for (const PointT & bluePt : *chanClouds[chan::BLUE]) {
		vector<int> nearestIdx;
		vector<float> nearestSqrDists;

		// Red Distances
		if (chanClouds[chan::RED]->size() > 0) {
			redSearch->nearestKSearch(bluePt, numNeighbors, nearestIdx, nearestSqrDists);
			sqrDistSum = 0;
			for (const int & sqrDist : nearestSqrDists) {
				sqrDistSum += sqrt(sqrDist);
			}
			sqrDistSum /= nearestSqrDists.size();
			if (distType == AVG) {
				blueDists.at<double>(matIdx, RED_DIST) = sqrDistSum;
			} else if (distType == NN) {
				blueDists.at<double>(matIdx, RED_DIST) = sqrt(nearestSqrDists[0]);
			} else if (distType == EXP) {
				double dist = 0;
				double eps = -0.5;
				for (const double & sqrDst : nearestSqrDists) {
					dist += exp(eps * sqrDst);
				}
				blueDists.at<double>(matIdx, RED_DIST) = dist / nearestSqrDists.size();
			}
		}

		// Green Distances
		if (chanClouds[chan::GREEN]->size() > 0) {
			greenSearch->nearestKSearch(bluePt, numNeighbors, nearestIdx, nearestSqrDists);

			sqrDistSum = 0;
			for (const int & sqrDist : nearestSqrDists) {
				sqrDistSum += sqrt(sqrDist);
			}
			sqrDistSum /= nearestSqrDists.size();
			if (distType == AVG) {
				blueDists.at<double>(matIdx, GREEN_DIST) = sqrDistSum;
			} else if (distType == NN) {
				blueDists.at<double>(matIdx, GREEN_DIST) = sqrt(nearestSqrDists[0]);
			} else if (distType == EXP) {
				double dist = 0;
				double eps = -0.5;
				for (const double & sqrDst : nearestSqrDists) {
					dist += exp(eps * sqrDst);
				}
				blueDists.at<double>(matIdx, GREEN_DIST) = dist / nearestSqrDists.size();
			}
		}

		// Update Mat Index
		matIdx++;
	}

	ofstream csvOut;
	ss.str(""); ss.clear();
	ss << boostSynNotchOutPath.string() << "/distances.csv";
	csvOut.open(ss.str());
	csvOut << cv::format(blueDists, cv::Formatter::FMT_CSV) << std::endl;
	csvOut.close();
	cout << endl << "Done!" << endl;
}




