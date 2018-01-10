#include <core/MicroscopeProperties.h>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/filesystem.hpp>

namespace pt = boost::property_tree;

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

