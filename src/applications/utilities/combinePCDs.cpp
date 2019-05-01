/*
  Copyright (C) 2011 - 2019, Raytheon BBN Technologies and contributors listed
  in the AUTHORS file in TASBE Flow Analytics distribution's top directory.

  This file is part of the TASBE Flow Analytics package, and is distributed
  under the terms of the GNU General Public License, with a linking
  exception, as described in the file LICENSE in the TASBE Image Analysis
  package distribution's top directory.
*/

#include <iostream>
#include <fstream>
#include <stdio.h>
#include <sstream>
#include <string>
#include <vector>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <pcl/common/time.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/conditional_removal.h>

using namespace std;
using namespace pcl;

typedef PointXYZI PointT;
typedef PointCloud<PointT> Cloud;
typedef typename Cloud::Ptr CloudPtr;
typedef typename Cloud::ConstPtr CloudConstPtr;

// Filename defines
const string OUTPUT_FILE = "output";

int cloudCount = 0;
int maxClouds = 100000;
int startCloud = 0;

CloudConstPtr maxCloud(new Cloud);

///////////////////////

void printUsage() {
    cout << "This program combines and registered two PCD files" << endl
            << "Usage: " << endl
            << "<pcd1> <pcd2>" << endl;
}

/**
 *
 */
int main(const int argc, const char **argv) {
    StopWatch time;

    string pcd1File, pcd2File;
    CloudPtr cloud1(new Cloud);
    CloudPtr cloud1Trimmed(new Cloud);
    CloudPtr cloud2(new Cloud);
    CloudPtr cloud2Trimmed(new Cloud);
    CloudPtr xformedCloud(new Cloud);
    CloudPtr cloudOut(new Cloud);
    CloudPtr ndtOut(new Cloud);

    if ((argc != 3)) {
	    printUsage();
	    return 0;
	}
	pcd1File = argv[1];
	pcd2File = argv[2];

	/********************* Load PCD Files *************************************/
	int rv1, rv2;
	rv1 = pcl::io::loadPCDFile(pcd1File, *cloud1);
	rv2 = pcl::io::loadPCDFile(pcd2File, *cloud2);

	if (rv1 == -1) {
	    cerr << "Error reading first PCD file! Path: " << pcd1File << endl;
	    return 0;
	}
	if (rv2 == -1) {
        cerr << "Error reading second PCD file! Path: " << pcd2File << endl;
        return 0;
    }

	/********************* Filter out ground points ***************************/
	// The arcs created by the Lidar on the ground can cause ICP to fail badly
	ConditionAnd<PointT>::Ptr rangeCond(new ConditionAnd<PointT>());
	    rangeCond->addComparison(
	            FieldComparison<PointT>::ConstPtr(
	                    new FieldComparison<PointT>("z", ComparisonOps::GT, 0.1)));
    ConditionalRemoval<PointT> condRem;
    condRem.setCondition(rangeCond);

    condRem.setInputCloud(cloud1);
    condRem.filter(*cloud1Trimmed);

    condRem.setInputCloud(cloud2);
    condRem.filter(*cloud2Trimmed);

    /********************* Apply Initial Guess Transformation *****************/
	//const Eigen::Affine3f xform = getTransformation(-4.707,-5.479,-.125,0,0,0);
	const Eigen::Affine3f xform = getTransformation(0,0,0,0,0,0);
	transformPointCloud(*cloud2Trimmed, *xformedCloud, xform);

	cout << "Running ICP..." << endl;
	StopWatch icpTime;
	IterativeClosestPointNonLinear<PointT, PointT> icp;
    icp.setInputSource(xformedCloud);
    icp.setInputTarget(cloud1Trimmed);

    icp.setMaximumIterations(500);
    icp.setTransformationEpsilon(1E-9);
    icp.setEuclideanFitnessEpsilon(0.1);

    /********************* Apply ICP ******************************************/
    Cloud icpCloud;
    icp.align(icpCloud);
    cout << "ICP Done! Time: " << icpTime.getTime() << endl;
    cout << "Has converged: " << icp.hasConverged() << " score: "
            << icp.getFitnessScore() << endl;
    cout << icp.getFinalTransformation() << endl;

    // ICP was performed on the trimmed clouds, transform the un-trimmed cloud
    string outFile;
    transformPointCloud(*cloud2, *xformedCloud, xform);

    // Save the original clouds, with just the initial guess xform applied
    outFile += OUTPUT_FILE + "_guessOnly.pcd";
    cloudOut->clear();
    (*cloudOut) += *cloud1;
    (*cloudOut) += *xformedCloud;
    cout << outFile << " has " << cloudOut->size() << " points" << endl;
    pcl::io::savePCDFile(outFile, *cloudOut);

    // Apply ICP XForm
    transformPointCloud(*xformedCloud, icpCloud, icp.getFinalTransformation());

	// Combine both clouds
    outFile = OUTPUT_FILE + "_nl_icp.pcd";
    cloudOut->clear();
	(*cloudOut) += *cloud1;
	(*cloudOut) += icpCloud;
	cout << outFile << " has " << cloudOut->size() << " points" << endl;
	pcl::io::savePCDFile(outFile, *cloudOut);

	cout << endl;
	cout << "Running NDT!" << endl;
	StopWatch ndtTime;
	// Initializing Normal Distributions Transform (NDT).
    pcl::NormalDistributionsTransform<PointT, PointT> ndt;

    // Setting scale dependent NDT parameters
    // Setting minimum transformation difference for termination condition.
    ndt.setTransformationEpsilon(1E-6);
    // Setting maximum step size for More-Thuente line search.
    ndt.setStepSize(0.02);
    //Setting Resolution of NDT grid structure (VoxelGridCovariance).
    ndt.setResolution(0.5);

    // Setting max number of registration iterations.
    ndt.setMaximumIterations(50);

    transformPointCloud(*cloud2Trimmed, *xformedCloud, xform);

    // Setting point cloud to be aligned.
    ndt.setInputSource(xformedCloud);
    // Setting point cloud to be aligned to.
    ndt.setInputTarget(cloud1Trimmed);

    Eigen::Matrix4f init_guess = Eigen::Matrix4f::Identity();

    // Calculating rigid transform to align the input cloud to the target cloud
    ndt.align(*ndtOut, icp.getFinalTransformation());

    cout << "NDT Done! Time: " << ndtTime.getTime() << endl;
    cout << "Has converged: " << ndt.hasConverged() << " score: "
            << ndt.getFitnessScore() << endl;
    cout << ndt.getFinalTransformation() << endl;

    // Apply NDT XForm
    transformPointCloud(*cloud2, *xformedCloud, xform);
    transformPointCloud(*xformedCloud, *ndtOut, ndt.getFinalTransformation());

    // Combine both clouds
    outFile = OUTPUT_FILE + "_ndt.pcd";
    cloudOut->clear();
    (*cloudOut) += *cloud1;
    (*cloudOut) += *ndtOut;
    cout << outFile << " has " << cloudOut->size() << " points" << endl;
    pcl::io::savePCDFile(outFile, *cloudOut);

    cout << "Run time: " << time.getTime() << " (" << time.getTimeSeconds()
            << " seconds)" << endl;
}
