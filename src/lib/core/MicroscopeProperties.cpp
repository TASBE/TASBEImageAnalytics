#include <core/MicroscopeProperties.h>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <boost/filesystem.hpp>

#include <iostream>

namespace pt = boost::property_tree;

const std::string MicroscopeProperties::INI_CFG_SECTION = "Config";

const std::string MicroscopeProperties::INI_PIXEL_WIDTH = "pixelWidth";
const std::string MicroscopeProperties::INI_PIXEL_HEIGHT = "pixelHeight";
const std::string MicroscopeProperties::INI_PIXEL_DEPTH = "pixelDepth";
const std::string MicroscopeProperties::INI_IMAGE_WIDTH = "imageWidth";
const std::string MicroscopeProperties::INI_IMAGE_HEIGHT = "imageHeight";

/**
 *
 */
bool MicroscopeProperties::readFromXML(const std::string & propXMLPath) {

	boost::filesystem::path bPath(propXMLPath);
	fileName = bPath.filename().c_str();
    // Create empty property tree object
    pt::ptree tree;

    // Parse the XML into the property tree.
    pt::read_xml(propXMLPath, tree);

    // Get dimension information from Dimensions tag
	for (pt::ptree::value_type &v : tree.get_child(
			"Data.Image.ImageDescription.Dimensions")) {
    	std::string dimId = v.second.get<std::string>("<xmlattr>.DimID");
    	double size = v.second.get<double>("<xmlattr>.Voxel");
    	int numElements = v.second.get<int>("<xmlattr>.NumberOfElements");
    	std::string unit = v.second.get<std::string>("<xmlattr>.Unit");
    	size *= getMultiplier(unit);
    	if (dimId == "X") {
    		this->pixelWidth = size;
    		this->imageWidth = numElements;
    	} else if (dimId == "Y") {
    		this->pixelHeight = size;
    		this->imageHeight = numElements;
    	} else if (dimId == "Z") {
    		this->pixelDepth = size;
    	}
    }
	return true;
}

bool MicroscopeProperties::readFromINI(const std::string & propINIPath) {
	boost::filesystem::path boostPath(propINIPath);
	if (!boost::filesystem::exists(boostPath)) {
		std::cout << "Segmentation params ini doesn't exist! Path: "
				<< propINIPath << std::endl;
		return false;
	}

	boost::property_tree::ptree pt;
	boost::property_tree::ini_parser::read_ini(propINIPath.c_str(), pt);

	for (auto& section : pt) {
		// Ignore sections other than the one we care about
		if (section.first != INI_CFG_SECTION) {
			continue;
		}
		for (auto& key : section.second) {
			if (key.first.compare(INI_PIXEL_HEIGHT) == 0) {
				this->pixelHeight = atof(key.second.get_value<std::string>().c_str());
			} else if (key.first.compare(INI_PIXEL_WIDTH) == 0) {
				this->pixelWidth = atof(key.second.get_value<std::string>().c_str());
			} else if (key.first.compare(INI_PIXEL_DEPTH) == 0) {
				this->pixelDepth = atof(key.second.get_value<std::string>().c_str());
			} else if (key.first.compare(INI_IMAGE_HEIGHT) == 0) {
				this->imageHeight = atof(key.second.get_value<std::string>().c_str());
			} else if (key.first.compare(INI_IMAGE_WIDTH) == 0) {
				this->imageWidth = atof(key.second.get_value<std::string>().c_str());
			}
		}
	}
	return true;
}


/**
 *
 */
double MicroscopeProperties::getMultiplier(std::string unit) {
	double mult = 1.0;
	if (unit == "m") {
		mult = 1e6;
	} else if (unit == "cm") {
		mult = 1e4;
	} else if (unit == "mm") {
		mult = 1e3;
	} else if (unit == "nm") {
		mult = 1e-3;
	}
	return mult;
}

