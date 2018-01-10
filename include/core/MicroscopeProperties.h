/*
 * MicrscopeProperties.h
 *
 *  Created on: Jan 10, 2018
 *      Author: nwalczak
 */

#ifndef INCLUDE_CORE_MICROSCOPEPROPERTIES_H_
#define INCLUDE_CORE_MICROSCOPEPROPERTIES_H_

#include <string>
#include <ostream>

struct MicroscopeProperties {
	double pixelWidth;
	double pixelHeight;
	double pixelDepth;
	double imageWidth;
	double imageHeight;

	std::string fileName;

	bool readFromXML(const std::string & propXMLPath);

	friend std::ostream& operator << (std::ostream& os, const MicroscopeProperties& p) {
		os << "Microscope properties for " << p.fileName << std::endl;
		os << "\timageWidth: " << p.imageWidth << std::endl;
		os << "\timageHeight: " << p.imageHeight << std::endl;
		os << "\tpixelWidth: " << p.pixelWidth << std::endl;
		os << "\tpixelHeight: " << p.pixelHeight << std::endl;
		os << "\tpixelDepth: " << p.pixelDepth << std::endl;
		return os;
	}
};


#endif /* INCLUDE_CORE_MICROSCOPEPROPERTIES_H_ */
