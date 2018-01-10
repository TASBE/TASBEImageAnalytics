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
#include <iomanip>

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
		os << "\t" << std::left << std::setw(15) << "imageWidth: "
				<< std::left << std::setw(15) << p.imageWidth << std::endl;
		os << "\t" << std::left << std::setw(15) << "imageHeight: "
				<< std::left << std::setw(15) << p.imageHeight << std::endl;
		os << "\t" << std::left << std::setw(15) << "pixelWidth: "
				<< std::left << std::setw(15) << p.pixelWidth << std::endl;
		os << "\t" << std::left << std::setw(15) << "pixelHeight: "
				<< std::left << std::setw(15) << p.pixelHeight << std::endl;
		os << "\t" << std::left << std::setw(15) << "pixelDepth: "
				<< std::left << std::setw(15) << p.pixelDepth << std::endl;
		return os;
	}
};


#endif /* INCLUDE_CORE_MICROSCOPEPROPERTIES_H_ */
