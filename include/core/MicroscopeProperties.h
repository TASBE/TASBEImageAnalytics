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

	static const std::string INI_CFG_SECTION;

	static const std::string INI_PIXEL_WIDTH;
	static const std::string INI_PIXEL_HEIGHT;
	static const std::string INI_PIXEL_DEPTH;
	static const std::string INI_IMAGE_WIDTH;
	static const std::string INI_IMAGE_HEIGHT;

	std::string fileName;

	bool readFromXML(const std::string & propXMLPath);

	bool readFromINI(const std::string & propINIPath);

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

	/**
	 * Get a multiplier to convert the given unit into micrometers.
	 */
	double getMultiplier(std::string unit);
};


#endif /* INCLUDE_CORE_MICROSCOPEPROPERTIES_H_ */
