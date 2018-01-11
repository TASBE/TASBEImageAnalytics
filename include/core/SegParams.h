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

	static const std::string EUC_CLUSTER_TOLERANCE;
	static const std::string MIN_CLUSTER_SIZE;
	static const std::string MAX_CLUSTER_SIZE;

	std::map<std::string,std::string> params;

	std::string inputFile;

	bool loadParameters(std::string pathToIni);

	bool hasParam(std::string key);

	std::string getValue(std::string key);

	int getInt(std::string key);

	int getDouble(std::string key);

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
