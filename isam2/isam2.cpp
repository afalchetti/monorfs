// isam2.cpp
// iSAM2 SLAM solving navigator
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

#include <gtsam/geometry/Point3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include "PixelRangeFactor.h"

#include <vector>

using namespace std;
using namespace gtsam;
using namespace boost;
using namespace monorfs;

extern "C" {
typedef boost::shared_ptr<noiseModel::Gaussian> Noise;

class ISAM2Navigator {
public:
	int            t;
	int            tlength;
	int            msize;
	vector<double> trajectory;
	vector<double> mapmodel;
	vector<double> tmarginals;
	vector<double> mmarginals;
	ISAM2          isam;
	Values         estimate;
	double         focal;
	Noise          measurementnoise;
	Noise          motionnoise;

	ISAM2Navigator(double focal, Noise measurementnoise, Noise motionnoise)
		: t(0), tlength(1), msize(0),
		  trajectory(vector<double>{0.0, 0.0, 0.0}), mapmodel(vector<double>()),
		  tmarginals(vector<double>(36)), mmarginals(vector<double>()),
		  isam(), estimate(), focal(focal),
		  measurementnoise(measurementnoise), motionnoise(motionnoise) {}
};

// write a pose state into a list;
// an element offset sets the distance
// from the top of the list in "pose units"
void marshalpose(double* list, const Pose3& state, const int n)
{
	int               offset   = 7 * n;
	const Point3&     position = state.translation();
	const Quaternion& rotation = state.rotation().toQuaternion();

	list[offset + 0] = position.x();
	list[offset + 1] = position.y();
	list[offset + 2] = position.z();
	list[offset + 3] = rotation.w();
	list[offset + 4] = rotation.x();
	list[offset + 5] = rotation.y();
	list[offset + 6] = rotation.z();
}

// write a point state into a list;
// an element offset sets the distance
// from the top of the list in "point units"
void marshalpoint(double* list, const Point3& state, const int n)
{
	int offset = 3 * n;

	list[offset + 0] = state.x();
	list[offset + 1] = state.y();
	list[offset + 2] = state.z();
}

// write a covariance marginal matrix into a list
// at a given offset in "covariance units"
// note that all marshaled marginals to the same list __must__
// be square and have the same size, otherwise there could be index
// bound errors (sigsev) and the offset will be wrong
void marshalmarginal(double* list, const Matrix& covariance, const int n)
{
	int side   = covariance.rows();
	int offset = n * side * side;

	for (int i = 0; i < side; ++i) {
	for (int k = 0; k < side; ++k) {
		list[offset + side * i + k] = covariance(i, k);
	}
	}
}

// read a pose state from a compact double list
// given an offset in "pose units"
Pose3 unmarshalpose(const double* list, const int n)
{
	int offset = 7 * n;  // state + covariance
	return Pose3(Rot3(Quaternion(list[offset + 3],
	                             list[offset + 4],
	                             list[offset + 5],
	                             list[offset + 6])),
	             Point3(list[offset + 0],
	                    list[offset + 1],
	                    list[offset + 2]));
}

// get the covariance matrix from a gaussian noise model
Matrix getcovariance(Noise noise)
{
	return (noise->R().transpose() * noise->R()).inverse();
}

// create a new isam2 slam solver with
// all its necessary context
// the motion noise format is [sx, sy, sz, syaw, spitch, sroll] (in local coords)
// the measurement noise format is [sx, sy, srange]
ISAM2Navigator* newnavigator(double* initstate, double* measurementnoise, double* motionnoise, double focal)
{
	Vector3 measurementsigma;
	Vector6 motionsigma;

	measurementsigma << measurementnoise[0], measurementnoise[1], measurementnoise[2];
	motionsigma << motionnoise[0], motionnoise[1], motionnoise[2],
	               motionnoise[3], motionnoise[4], motionnoise[5];
	
	Pose3 initpose = Pose3(Rot3(Quaternion(initstate[3], initstate[4], initstate[5], initstate[6])),
	                       Point3(initstate[0], initstate[1], initstate[2]));

	noiseModel::Diagonal::shared_ptr posenoise =
	    noiseModel::Diagonal::Sigmas((Vector(6) << Vector3::Constant(0),Vector3::Constant(0)));

	ISAM2Navigator* navigator = new ISAM2Navigator(focal,
	                                               noiseModel::Diagonal::Sigmas(measurementsigma),
	                                               noiseModel::Diagonal::Sigmas(motionsigma));

	marshalmarginal(&navigator->tmarginals[0], getcovariance(posenoise), 0);
	navigator->estimate.insert(Symbol('x', 0), initpose);

	NonlinearFactorGraph graph;
	graph.push_back(PriorFactor<Pose3>(Symbol('x', 0), initpose, posenoise));

	navigator->isam.update(graph, navigator->estimate);
	++navigator->t;

	return navigator;
}

// delete and clean up the navigator context
void deletenavigator(ISAM2Navigator* navigator)
{
	delete navigator;
}

// estimate a landmark from an estimated pose and the raw measurement
Point3 landmarkestimate(double px, double py, double range, Pose3 pose, double focal)
{
	double localz = range * focal / sqrt(px*px + py*py + focal*focal);
	double localx = localz * px / focal;
	double localy = localz * py / focal;

	return pose.translation() + pose.rotation() * Point3(localx, localy, localz);
}

// update the navigator estimate using new information:
// odometry and  measurement lists from the last step;
// 'odometry' format is an array with [dx, dy, dz, dyaw, dpitch, droll]
// in loca coordinates;
// 'measurements' format must be a 3*n double array with the
// x-y-z coordinates for each one of them;
// each measurement must be associated to a labeled (int) landmark;
void update(ISAM2Navigator* navigator, double* odometry, double* measurements, int* labels, int nmeasurements)
{
	NonlinearFactorGraph graph;
	Values               newestimates;

	// add new estimate for new pose (estimate using last pose)
	Pose3 pestimate = navigator->estimate.at<Pose3>(Symbol('x', navigator->t - 1));

	newestimates.insert(Symbol('x', navigator->t), pestimate);

	// add new measurements
	for (int i = 0, k = 0; i < nmeasurements; ++i, k += 3) {
		int    l     = labels[i];
		double px    = measurements[k + 0];
		double py    = measurements[k + 1];
		double range = measurements[k + 2];

		graph.push_back(PixelRangeFactor(Symbol('x', navigator->t), Symbol('l', l),
		                                 px, py, range,
		                                 navigator->measurementnoise, navigator->focal));

		if (!navigator->estimate.exists(Symbol('l', l))) {
			newestimates.insert(Symbol('l', l),
			                    landmarkestimate(px, py, range, pestimate, navigator->focal));
		}
	}

	if (odometry != nullptr) {
		// note that dorientation is shifted since the coordinate
		// system is not the same in this framework: roll doesn't
		// really roll the vehicle, but changes the pitch, etc.
		Pose3 delta(Rot3::ypr(odometry[5], odometry[3], odometry[4]),
	                Point3(odometry[0], odometry[1], odometry[2]));
	    
		graph.push_back(BetweenFactor<Pose3>(Symbol('x', navigator->t - 1),
		                                     Symbol('x', navigator->t),
		                                     delta, navigator->motionnoise));
	}

	// isam2 magic bayes tree update
	navigator->isam.update(graph, newestimates);
	navigator->estimate = navigator->isam.calculateEstimate();

	// update the public estimates (interop)
	navigator->tlength = navigator->tlength + 1;
	navigator->msize   = navigator->estimate.size() - navigator->tlength;

	navigator->trajectory.resize(7 * navigator->tlength);
	navigator->mapmodel  .resize(3 * navigator->msize);
	navigator->tmarginals.resize((7 * 7) * navigator->tlength);
	navigator->mmarginals.resize((3 * 3) * navigator->msize);

	for (auto item = navigator->estimate.begin(); item != navigator->estimate.end(); ++item) {
		Symbol key(item->key);

		Matrix covariance = navigator->isam.marginalCovariance(key);

		if (key.chr() == 'x') {  // pose
			marshalpose(&navigator->trajectory[0], static_cast<Pose3&>(item->value), key.index());
			marshalmarginal(&navigator->tmarginals[0], covariance, key.index());
		}
		else if (key.chr() == 'l') {  // landmark
			marshalpoint(&navigator->mapmodel[0], static_cast<Point3&>(item->value), key.index());
			marshalmarginal(&navigator->mmarginals[0], covariance, key.index());
		}
	}

	++navigator->t;
}

// get the trajectory estimate held in the navigator
double* gettrajectory(ISAM2Navigator* navigator, int* length)
{
	*length = navigator->tlength;
	return &navigator->trajectory[0];
}

// get the map model held in the navigator
double* getmapmodel(ISAM2Navigator* navigator, int* length)
{
	*length = navigator->msize;
	return &navigator->mapmodel[0];
}

// get the trajectory estimate covariance matrices held in the navigator
double* trajectorymarginals(ISAM2Navigator* navigator, int* length)
{
	*length = navigator->tlength;
	return &navigator->tmarginals[0];
}

// get the map model covariances held in the navigator
double* mapmarginals(ISAM2Navigator* navigator, int* length)
{
	*length = navigator->msize;
	return &navigator->mmarginals[0];
}
}
