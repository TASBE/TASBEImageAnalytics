/*
 * SegmentationParameters.h
 *
 *  Created on: Jan 10, 2018
 *      Author: nwalczak
 */

#ifndef INCLUDE_CORE_SEGPARAMS_H_
#define INCLUDE_CORE_SEGPARAMS_H_

#include <map>
#include <string>
#include <iostream>
#include <iomanip>

struct SegParams {
	static const std::string SEG_SECTION;
	static const std::string INPUT_SECTION;

	static const std::string RUN_NAME;
	static const std::string INPUT_CLOUD;
	static const std::string INPUT_BRIGHTFIELD;
	static const std::string INPUT_YFP;
	static const std::string INPUT_BFP;
	static const std::string OUTPUT_DIR;
	static const std::string SCOPE_PROPERTIES;

	static const std::string SEG_TYPE;
	static const std::string EUC_CLUSTER_TOLERANCE;
	static const std::string MIN_CLUSTER_SIZE;
	static const std::string MAX_CLUSTER_SIZE;

	static const std::string ST_EUCLIDEAN;
	static const std::string ST_VOXEL;

	// Store all of the known parameters
	std::map<std::string,std::string> params;

	// Keep track of the file that loaded the parameters
	std::string inputFile;

	/**
	 * Load parameters from a given ini file.
	 */
	bool loadParameters(std::string pathToIni);

	/**
	 * Return true if the supplied key has a stored value, else false.
	 */
	bool hasParam(std::string key);

	/**
	 * Get the value for a specified string, as a string. Does not check for
	 * the existence of the key, so if the key isn't stored a default value
	 * will be returned.
	 */
	std::string getValue(std::string key);

	/**
	 * Get the value for a specified string, as an int. Does not check for
	 * the existence of the key, so if the key isn't stored a default value
	 * will be returned.  Also does not check if the conversion is possible.
	 */
	int getInt(std::string key);

	/**
	 * Get the value for a specified string, as a double. Does not check for
	 * the existence of the key, so if the key isn't stored a default value
	 * will be returned.  Also does not check if the conversion is possible.
	 */
	int getDouble(std::string key);

	/**
	 * Return a description of this class.
	 */
	static std::string getDescription();

	friend std::ostream& operator << (std::ostream& os, const SegParams& p) {
		os << "Segmentation parameters for " << p.inputFile << std::endl;
		for (auto it = p.params.begin(); it != p.params.end(); it++) {
			os << "\t" << std::left << std::setw(25) << it->first + ": "
					<< std::left << std::setw(25) << it->second << std::endl;
		}

		return (os);
	}
};



#endif /* INCLUDE_CORE_SEGPARAMS_H_ */
