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
#include <iostream>
#include <stdio.h>
#include <stdlib.h>

using namespace cv;
using namespace std;

Mat src;
Mat blurred;
int thresh = 100;
int max_thresh = 255;
int origBlurr;
RNG rng(12345);

/// Function header
void thresh_callback(int, void*);

void src_callback(int, void*);

/** @function main */
int main(int argc, char** argv) {
	/// Load source image and convert it to gray
	src = imread(argv[1], IMREAD_ANYDEPTH);

	cout << "Bit depth: " << src.depth() * 8 << endl;
	if (src.depth() > 1) {
		/*
		double minVal, maxVal;
		minMaxLoc(src, &minVal, &maxVal);
		cout << "Min Val: " << minVal << ", Max Val: " << maxVal << endl;

		Mat full, partial;
		double gain = 255/(maxVal - minVal);
		src.convertTo(full, CV_8UC1, 255/(pow(2,16) - 1), 0);
		src.convertTo(partial, CV_8UC1, gain, - minVal * gain);

		minMaxLoc(full, &minVal, &maxVal);
		cout << "full Min Val: " << minVal << ", full Max Val: " << maxVal << endl;
		minMaxLoc(partial, &minVal, &maxVal);
		cout << "partial Min Val: " << minVal << ", partial Max Val: " << maxVal << endl;

		namedWindow("Full", CV_WINDOW_AUTOSIZE);
		imshow("Full", full);
		namedWindow("Partial", CV_WINDOW_AUTOSIZE);
		imshow("Partial", partial);
		waitKey(0);
		*/

		double minVal, maxVal;
		minMaxLoc(src, &minVal, &maxVal);

		Mat eightBit;
		double gain = 255/(maxVal - minVal);
		src.convertTo(eightBit, CV_8UC1, gain, - minVal * gain);
		src = eightBit;
	}

	/// Convert image to gray and blur it
	if (src.channels() > 1) {
		Mat src_gray;
		cvtColor(src, src_gray, CV_BGR2GRAY);
		src = src_gray;
	}


	ximgproc::bilateralTextureFilter(src, blurred);

	/// Create Window
	namedWindow("Source", CV_WINDOW_AUTOSIZE);
	imshow("Source", src);

	createTrackbar(" Canny thresh:", "Source", &thresh, max_thresh,
			thresh_callback);
	createTrackbar(" Orig/Blurred", "Source", &origBlurr, 1, src_callback);
	thresh_callback(0, 0);

	waitKey(0);
	return (0);
}

/** @function thresh_callback */
void thresh_callback(int, void*) {
	Mat canny_output;
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;

	/// Detect edges using canny
	Canny(blurred, canny_output, thresh, thresh * 2, 3);
	/// Find contours
	findContours(canny_output, contours, hierarchy, CV_RETR_EXTERNAL,
			CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

	/// Draw contours
	Mat drawing = Mat::zeros(canny_output.size(), CV_8UC3);
	for (int i = 0; i < contours.size(); i++) {
		Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255),
				rng.uniform(0, 255));
		drawContours(drawing, contours, i, color, 2, 8, hierarchy, 0, Point());
	}

	/// Show in a window
	namedWindow("Contours", CV_WINDOW_AUTOSIZE);
	imshow("Contours", drawing);

	namedWindow("Edges", CV_WINDOW_AUTOSIZE);
	imshow("Edges", canny_output);
}

void src_callback(int, void*) {

	if (origBlurr) {
		imshow("Source", blurred);
	} else {
		imshow("Source", src);
	}
}
