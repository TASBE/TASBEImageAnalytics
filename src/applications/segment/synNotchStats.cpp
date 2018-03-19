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
#include <pcl/common/time.h>
#include <pcl/common/impl/common.hpp>

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
typedef Cloud::Ptr CloudPtr;

typedef pcl::PointCloud<PointNormal> PointCloudWithNormals;

enum chan {RED = 0, GREEN = 1, BLUE = 2, NUM=3};
string chanNames[3] = {"Red", "Green", "Blue"};
Vec3b colors[3] = {Vec3b(0,0,255), Vec3b(0,255,0), Vec3b(255,0,0)};

void printUsage() {
	cout << "This program processes the 3D point cloud segmentation to compute "
			<< "statistics about the relative positions of senders and "
			<< "receivers." << endl
			<< "It takes three parameters, each a path to a segmentation "
			<< "parameters configuration ini file for each channel."
			<< endl << endl
			<< "Usage: " << endl
			<< "<redIni> <greenIni> <blueIni>"
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
	if (argc != 4) {
		cout << "Expected 3 inputs, received " << (argc - 1) << endl << endl;
		printUsage();
		return 0;
	}

	SegParams chanParams[chan::NUM];
	PointCloud<PointXYZRGBL> chanLabeledClouds[3];
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
		string runName;
		if (chanParams[i].hasParam(SegParams::RUN_NAME)) {
			runName = chanParams[i].getValue(SegParams::RUN_NAME);
		}
		// Load cloud of labeled points
		ss.str(""); ss.clear();
		ss << outPath << "/" << runName << "_clusters" << ".ply";
		io::loadPLYFile (ss.str(), chanLabeledClouds[i]);
		cout << chanNames[i] << " cloud path: " << ss.str() << endl;
		cout << chanNames[i] << " cloud num pts: " << chanLabeledClouds[i].size() << endl;
		cout << endl;
	}

	// Ensure output dir exists
	fs::path boostCloudOutPath(outPath);
	fs::path boostParentOutPath = boostCloudOutPath.parent_path();
	fs::path boostSynNotchOutPath = boostParentOutPath;
	boostSynNotchOutPath += "/SynNotchOutput";
	if (!fs::exists(boostSynNotchOutPath)) {
		fs::create_directory(boostSynNotchOutPath);
	}

	MicroscopeProperties scopeProps;
	if (!scopeProps.readFromXML(
			chanParams[chan::RED].getValue(SegParams::SCOPE_PROPERTIES))) {
		cout << "Failed loading microscope properties!" << endl;
		return 0;
	}

	// Single image plane projection of cluster locations
	Mat clusterProjection(scopeProps.imageHeight, scopeProps.imageWidth, CV_8UC3, Scalar(0,0,0));

	// Separate labeled clouds into individual object clouds
	map<uint32_t, Cloud> clusterClouds[chan::NUM];
	for (int i = chan::NUM - 1; i >= 0; i--) {
		for (int p = 0; p < chanLabeledClouds[i].size(); p++) {
			const PointXYZRGBL & lpt = chanLabeledClouds[i][p];
			PointT pt(lpt.r, lpt.g, lpt.b);
			pt.x = lpt.x; pt.y = lpt.y; pt.z = lpt.z;
			clusterClouds[i][lpt.label].push_back(pt);
			int row = (int)(pt.y / scopeProps.pixelHeight + 0.5);
			int col = (int)(pt.x / scopeProps.pixelWidth + 0.5);
			clusterProjection.at<Vec3b>(row, col) = colors[i];
		}
	}

	/*
	// Debugging: log info about clusters
	for (int i = 0; i < chan::NUM; i++) {
		cout << chanNames[i] << " Clusters: " << endl;
		for (auto const & clIt : clusterClouds[i]) {
			cout << "\t Id " << clIt.first << ": " << clIt.second.size()
					<< " points" << endl;
		}
		cout << endl;
	}
	*/

	// Compute distances
	std::ofstream snStats;
	ss.str(""); ss.clear();
	ss << boostSynNotchOutPath.string() << "/synNotchStats.csv";
	snStats.open(ss.str().c_str());
	snStats << "blue clusterId, blue volume, integrated intensity, size,"
			"closest red dist, closest green dist, "
			"closest red id, volume, integrated intensity, size,"
			"closest green id, "
			"volume, integrated intensity, size" << endl;
	set<uint32_t> usedIds[chan::NUM];
	double ptVol = scopeProps.pixelDepth * scopeProps.pixelHeight * scopeProps.pixelWidth;
	for (auto const & blueCl : clusterClouds[chan::BLUE]) {
		double closestRedDist = -1;
		uint32_t closestRedId;
		uint32_t closestRedSize;
		uint32_t blueIntensity = 0;
		for (auto const & pt : blueCl.second.points) {
			blueIntensity += pt.b;
		}
		for (auto const & redCl : clusterClouds[chan::RED]) {
			double rbDist = intraCloudDistance(redCl.second, blueCl.second);
			if (closestRedDist == -1 || rbDist < closestRedDist) {
				closestRedDist = rbDist;
				closestRedId = redCl.first;
				closestRedSize = redCl.second.size();
			}
		}
		double closestGreenDist = -1;
		uint32_t closestGreenId;
		uint32_t closestGreenSize;
		for (auto const & greenCl : clusterClouds[chan::GREEN]) {
			double bgDist = intraCloudDistance(blueCl.second, greenCl.second);
			if (closestGreenDist == -1 || bgDist < closestGreenDist) {
				closestGreenDist = bgDist;
				closestGreenId = greenCl.first;
				closestGreenSize = greenCl.second.size();
			}
		}
		uint32_t greenIntensity = 0;
		for (auto const & pt : clusterClouds[chan::GREEN][closestGreenId].points) {
			greenIntensity += pt.g;
		}
		usedIds[chan::GREEN].insert(closestGreenId);
		usedIds[chan::BLUE].insert(blueCl.first);
		if (closestRedDist < 1) { // Matched a Red
			uint32_t redIntensity = 0;
			for (auto const & pt : clusterClouds[chan::RED][closestRedId].points) {
				redIntensity += pt.g;
			}
			usedIds[chan::RED].insert(closestRedId);
			int blueSize = blueCl.second.size();
			snStats << blueCl.first << ", " << blueSize * ptVol << ", "
					<< blueIntensity << ", " << blueCl.second.size() << ", "
					<< closestRedDist << ", " << closestGreenDist << ", "
					<< closestRedId << ", " << closestRedSize * ptVol << ", "
					<< redIntensity << ", " << closestRedSize << ", "
					<< closestGreenId << ", " << closestGreenSize * ptVol << ", "
					<< greenIntensity << ", " << closestGreenSize
					<< endl;
		} else { // Did not match a red
			snStats << blueCl.first << ", " << blueCl.second.size() * ptVol << ", "
					<< blueIntensity << ", " << blueCl.second.size() << ", "
					<< ", , " << closestGreenDist
					<< ", , , , " << closestGreenId << ", "
					<< closestGreenSize * ptVol  << ", "
					<< greenIntensity << ", " << closestGreenSize
					<< endl;
		}
	}

	vector<uint32_t> unmatchedReds;
	for (auto const & redCl : clusterClouds[chan::RED]) {
		if (usedIds[chan::RED].count(redCl.first) < 1) {
			unmatchedReds.push_back(redCl.first);
			// Make sure the projection displays this ID
			usedIds[chan::RED].insert(redCl.first);
		}
	}
	if (unmatchedReds.size() > 0) {
		std::ofstream unmatchedOut;
		ss.str(""); ss.clear();
		ss << boostSynNotchOutPath.string() << "/unmatchedRed.csv";
		unmatchedOut.open(ss.str().c_str());
		unmatchedOut << "Unmatched Red ID" << endl;
		for (auto id : unmatchedReds) {
			unmatchedOut << id << endl;
		}
	}

	// Add cluster labels to projection
	int fontFace = cv::FONT_HERSHEY_COMPLEX_SMALL;
	int fontScale = 1;
	int fontThickness = 1;
	for (int i = chan::NUM - 1; i >= 0; i--) {
		for (auto const & clIt : clusterClouds[i]) {
			// Only display used labels
			if (usedIds[i].count(clIt.first) < 1) {
				continue;
			}
			PointT minVals, maxVals;
			getMinMax3D(clIt.second, minVals, maxVals);
			ss.str(""); ss.clear();
			ss << clIt.first;
			int yBaseLine;
			Size txtSz = getTextSize(ss.str(), fontFace, fontScale,
					fontThickness, &yBaseLine);
			Point txtLoc = Point(
					(int) (minVals.x / scopeProps.pixelWidth + 0.5),
					(int) (minVals.y / scopeProps.pixelHeight + 0.5));
			putText(clusterProjection, ss.str(), txtLoc, fontFace, fontScale,
					colors[i], 2);
		}
		cout << endl;
	}

	// Save cluster loc image projection
	ss.str(""); ss.clear();
	ss << boostSynNotchOutPath.string() << "/allSegMask.png";
	imwrite(ss.str(), clusterProjection);

	cout << endl << "Done!" << endl;
}




