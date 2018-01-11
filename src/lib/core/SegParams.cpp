/*
 * SegmentationParameters.h
 *
 *  Created on: Jan 10, 2018
 *      Author: nwalczak
 */

#include "core/SegParams.h"

#include <boost/filesystem.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>

const std::string SegParams::SEG_SECTION = "SegmentationParameters";

const std::string SegParams::SEG_TYPE = "SegType";
const std::string SegParams::EUC_CLUSTER_TOLERANCE = "EucClusterTolerance";
const std::string SegParams::MIN_CLUSTER_SIZE = "MinClusterSize";
const std::string SegParams::MAX_CLUSTER_SIZE = "MaxClusterSize";

const std::string SegParams::ST_EUCLIDEAN = "Euclidean";
const std::string SegParams::ST_VOXEL = "Voxel";


bool SegParams::loadParameters(std::string pathToIni) {
	boost::filesystem::path boostPath(pathToIni);
	if (!boost::filesystem::exists(boostPath)) {
		std::cout << "Segmentation params ini doesn't exist! Path: "
				<< pathToIni << std::endl;
		return false;
	}
	inputFile = boostPath.filename().c_str();

	boost::property_tree::ptree pt;
	boost::property_tree::ini_parser::read_ini(pathToIni.c_str(), pt);

	for (auto& section : pt) {
		// Ignore sections other than the one we care about
		if (section.first != SEG_SECTION) {
			continue;
		}
		for (auto& key : section.second) {
			params[key.first] = key.second.get_value<std::string>();
		}
	}
	return true;
}

bool SegParams::hasParam(std::string key) {
	return params.find(key) != params.end();
}

std::string SegParams::getValue(std::string key) {
	return params[key];
}

int SegParams::getInt(std::string key) {
	return atoi(params[key].c_str());
}

int SegParams::getDouble(std::string key) {
	return atof(params[key].c_str());
}

