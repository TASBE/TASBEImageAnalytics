/*
 * SegmentationParameters.h
 *
 *  Created on: Jan 10, 2018
 *      Author: nwalczak
 */

#include "core/SegParams.h"

#include <sstream>

#include <boost/filesystem.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>

const std::string SegParams::SEG_SECTION = "SegmentationParameters";
const std::string SegParams::INPUT_SECTION = "InputParameters";

const std::string SegParams::INPUT_CLOUD = "InputCloud";
const std::string SegParams::INPUT_BRIGHTFIELD = "InputBrightField";
const std::string SegParams::INPUT_YFP = "InputYFP";
const std::string SegParams::INPUT_BFP = "InputBFP";
const std::string SegParams::OUTPUT_DIR = "OutputDir";
const std::string SegParams::SCOPE_PROPERTIES = "MicroscopeProperties";

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
		if (section.first != SEG_SECTION && section.first != INPUT_SECTION) {
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

std::string SegParams::getDescription() {
	std::stringstream ss;

	ss << "Supported parameters for segmentation ini:" << std::endl;
	ss << INPUT_SECTION << std::endl;

	ss << "\t" << INPUT_CLOUD << std::endl;
	ss << "\t" << INPUT_BRIGHTFIELD << std::endl;
	ss << "\t" << INPUT_YFP << std::endl;
	ss << "\t" << INPUT_BFP << std::endl;
	ss << "\t" << OUTPUT_DIR << std::endl;
	ss << "\t" << SCOPE_PROPERTIES << std::endl;

	ss << SEG_SECTION << std::endl;
	ss << "\t" << EUC_CLUSTER_TOLERANCE << std::endl;
	ss << "\t" << MIN_CLUSTER_SIZE << std::endl;
	ss << "\t" << MAX_CLUSTER_SIZE << std::endl;

	ss << "\t" << SEG_TYPE << std::endl;
	ss << "\t\t" << ST_EUCLIDEAN << std::endl;
	ss << "\t\t" << ST_VOXEL << std::endl;

	return ss.str();
}
