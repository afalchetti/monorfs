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

using namespace std;
using namespace cv;

extern "C" {
double* getkeypointsorb(char* imgstream, int rows, int cols, int* length)
{
	Mat image(rows, cols, CV_8UC1, imgstream);

	vector<KeyPoint> kp;
	ORB orbdetect(150);
	Mat mask = Mat::ones(image.size(), image.type());
	orbdetect(image, mask, kp, cv::noArray());

	*length = kp.size();

	double* kpstream = new double[2 * (*length)];

	for (int i = 0, h = 0; i < (*length); i++, h += 2) {
		kpstream[h+0] = kp[i].pt.x;
		kpstream[h+1] = kp[i].pt.y;
	}

	return kpstream;
}

void deletearray(double* array)
{
	delete[] array;
}

//int main (int argc, char *argv[])
//{
//	VideoCapture reader("room.mp4");
//
//	if (!reader.isOpened()) {
//		return -1;
//	}
//
//	Mat gray;
//	namedWindow("kp", WINDOW_AUTOSIZE);
//	int i = 0;
//	while (true) {
//		Mat frame;
//		reader >> frame;
//
//		cvtColor(frame, gray, CV_BGR2GRAY);
//		vector<KeyPoint> keypoints = getKeyPointsFAST(gray);
//
//		drawKeypoints(gray, keypoints, gray);
//
//		imshow("kp", gray);
//
//		if (i == 10) {
//			waitKey(0);
//		}
//		else {
//			if (waitKey(30) >= 0) { break; }
//		}
//
//		i++;
//	}
//
//	return 0;
//}
}

