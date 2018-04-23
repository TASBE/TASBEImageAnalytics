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
