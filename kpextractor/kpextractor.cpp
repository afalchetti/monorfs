// kpextractor.cpp
// Keypoint extractor
// Part of MonoRFS
//
// Copyright (c) 2015, Angelo Falchetti
// All rights reserved.
// 
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * The names of its contributors may not be used to endorse or promote products
//       derived from this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL ANGELO FALCHETTI BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "latch.hpp"

using namespace std;
using namespace cv;
using namespace cv::xfeatures2d;

extern "C" {
double* getkeypointslatch(char* imgstream, int rows, int cols, bool calcdescriptors, int* length, char** descstream)
{
	Mat image(rows, cols, CV_8UC1, imgstream);

	vector<KeyPoint> kp;
	Mat        descriptors;
	double*    kpstream;
	Ptr<LATCH> detector = LATCH::create();
	Mat        mask  = Mat::ones(image.size(), image.type());
	const int  dsize = 32;

	if (calcdescriptors) {
		(*detector)(image, mask, kp, descriptors);
	}
	else {
		(*detector)(image, mask, kp, cv::noArray());
	}

	*length = kp.size();

	 if (calcdescriptors) {
		kpstream    = new double[2 * (*length)];
		*descstream = new char[dsize * (*length)];

		for (int i = 0, h = 0, m = 0; i < (*length); i++) {
			kpstream[h++] = kp[i].pt.x;
			kpstream[h++] = kp[i].pt.y;

			for (int k = 0; k < dsize; k++) {
				(*descstream)[m++] = descriptors.at<char>(i, k);
			}
		}
	}
	else {
		kpstream    = new double[2 * (*length)];
		*descstream = 0;

		for (int i = 0, h = 0; i < (*length); i++) {
			kpstream[h++] = kp[i].pt.x;
			kpstream[h++] = kp[i].pt.y;
		}
	}

	return kpstream;
}

double* getkeypointsorb(char* imgstream, int rows, int cols, bool calcdescriptors, int* length, char** descstream)
{
	Mat image(rows, cols, CV_8UC1, imgstream);

	vector<KeyPoint> kp;
	Mat       descriptors;
	double*   kpstream;
	ORB       orbdetect(200);
	Mat       mask  = Mat::ones(image.size(), image.type());
	const int dsize = 32;

	if (calcdescriptors) {
		orbdetect(image, mask, kp, descriptors);
	}
	else {
		orbdetect(image, mask, kp, cv::noArray());
	}

	*length = kp.size();

	 if (calcdescriptors) {
		kpstream    = new double[2 * (*length)];
		*descstream = new char[dsize * (*length)];

		for (int i = 0, h = 0, m = 0; i < (*length); i++) {
			kpstream[h++] = kp[i].pt.x;
			kpstream[h++] = kp[i].pt.y;

			for (int k = 0; k < dsize; k++) {
				(*descstream)[m++] = descriptors.at<char>(i, k);
			}
		}
	}
	else {
		kpstream    = new double[2 * (*length)];
		*descstream = 0;

		for (int i = 0, h = 0; i < (*length); i++) {
			kpstream[h++] = kp[i].pt.x;
			kpstream[h++] = kp[i].pt.y;
		}
	}

	return kpstream;
}

double* getkeypoints(char* imgstream, int rows, int cols, bool calcdescriptors, int* length, char** descstream)
{
	return getkeypointslatch(imgstream, rows, cols, calcdescriptors, length, descstream);
}

void deletearrayd(double* array)
{
	delete[] array;
}

void deletearrayc(char* array)
{
	delete[] array;
}

}