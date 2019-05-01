/*
  Copyright (C) 2011 - 2019, Raytheon BBN Technologies and contributors listed
  in the AUTHORS file in TASBE Flow Analytics distribution's top directory.

  This file is part of the TASBE Flow Analytics package, and is distributed
  under the terms of the GNU General Public License, with a linking
  exception, as described in the file LICENSE in the TASBE Image Analysis
  package distribution's top directory.
*/

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/ximgproc/edge_filter.hpp"
#include "opencv2/ximgproc/slic.hpp"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>

using namespace cv;
using namespace std;

/** @function main */
int main(int argc, char** argv) {

	if (argc != 3) {
		cout << "Expected 2 arguments, received " << (argc - 1) << endl;
		return -1;
	}

	/// Load source image and convert it to gray
	Mat src = imread(argv[1], 1);

	string baseOutput = argv[2];

	/// Convert image to gray and blur it
	Mat srcScaled;
	src.convertTo(srcScaled, CV_8UC1);

	Mat dst;

	ximgproc::bilateralTextureFilter(srcScaled, dst);

	Ptr<ximgproc::SuperpixelSLIC> slicAlg = ximgproc::createSuperpixelSLIC(srcScaled,
			ximgproc::MSLIC);

	slicAlg->iterate();

	Mat slicMask;
	slicAlg->getLabelContourMask(slicMask);
	Mat output = dst.clone();
	srcScaled.copyTo(output, slicMask);

	imwrite(baseOutput + "_bilat.png", dst);
	imwrite(baseOutput + "_slic.png", output);
	imwrite(baseOutput + "_slicMask.png", slicMask);


	return (0);
}
