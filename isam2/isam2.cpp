// isam2.cpp
// ISAM2 SLAM solving navigator
// Part of MonoRFS
//
// Copyright (C) 2015 Angelo Falchetti
// All rights reserved.
// Any use, reproduction, distribution or copy of this
// work via any medium is strictly forbidden without
// express written consent from the author.

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
typedef boost::shared_ptr<noiseModel::Base> Noise;

typedef struct {
	int                  t;
	int                  tlength;
	int                  msize;
	vector<double>       trajectory;
	vector<double>       mapmodel;
	vector<int>          mlabels;
	ISAM2                isam;
	NonlinearFactorGraph graph;
	Values               estimate;
	double               focal;
	Noise                measurementnoise;
	Noise                motionnoise;
} ISAM2Navigator;

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
	int     offset   = 3 * n;

	list[offset + 0] = state.x();
	list[offset + 1] = state.y();
	list[offset + 2] = state.z();
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

// create a new isam2 slam solver with
// all its necessary context
// the motion noise format is [syaw, spitch, sroll, sx, sy, sz] (in local coords)
// the measurement noise format is [sx, sy, srange]
ISAM2Navigator* newnavigator(double* measurementnoise, double* motionnoise, double focal)
{
	Vector3 measurementsigma;
	Vector6 motionsigma;

	measurementsigma << measurementnoise[0], measurementnoise[1], measurementnoise[2];
	motionsigma << motionnoise[0], motionnoise[1], motionnoise[2],
	               motionnoise[3], motionnoise[4], motionnoise[5];
	
	ISAM2Navigator* navigator =
	    new ISAM2Navigator{0, 1, 0, vector<double>{0.0, 0.0, 0.0}, vector<double>(), vector<int>(),
	                       ISAM2(), NonlinearFactorGraph(), Values(), focal,
	                       noiseModel::Diagonal::Sigmas(measurementsigma),
	                       noiseModel::Diagonal::Sigmas(motionsigma)};
	
	Pose3 initpose = Pose3(Rot3(Quaternion(1, 0, 0, 0)), Point3(0, 0, 0));
	noiseModel::Diagonal::shared_ptr posenoise =
	    noiseModel::Diagonal::Sigmas((Vector(6) << Vector3::Constant(0),Vector3::Constant(0)));
	
	navigator->estimate.insert(Symbol('x', 0), initpose);
	navigator->graph.push_back(PriorFactor<Pose3>(Symbol('x', 0), initpose, posenoise));

	return navigator;
}

// delete and clean up the navigator context
void deletenavigator(ISAM2Navigator* navigator)
{
	delete navigator;
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
	// add new measurements
	for (int i = 0, k = 0; i < nmeasurements; ++i, k += 4) {
		int    l     = measurements[k];
		double px    = measurements[k + 1];
		double py    = measurements[k + 2];
		double range = measurements[k + 3];

		navigator->graph.push_back(PixelRangeFactor(Symbol('x', navigator->t), Symbol('l', l),
		                                            px, py, range,
		                                            navigator->measurementnoise, navigator->focal));
	}

	if (odometry != nullptr) {
		Pose3 delta(Rot3::ypr(odometry[3], odometry[4], odometry[5]),
	                Point3(odometry[0], odometry[1], odometry[2]));
	    
		navigator->graph.push_back(BetweenFactor<Pose3>(Symbol('x', navigator->t),
		                                                Symbol('x', navigator->t - 1),
		                                                delta, navigator->motionnoise));
	}

	// add new estimate for new pose (estimate using last pose)
	Pose3 pestimate = unmarshalpose(&navigator->trajectory[0], navigator->tlength - 1);
	navigator->estimate.insert(Symbol('x', navigator->t), pestimate);

	// isam2 magic bayes tree update
	navigator->isam.update(navigator->graph, navigator->estimate);
	Values current = navigator->isam.calculateEstimate();

	// update the public estimates (interop)
	navigator->tlength = navigator->tlength + 1;
	navigator->msize   = current.size() - navigator->tlength;

	navigator->trajectory.resize(7 * navigator->tlength);  // mean + covariance
	navigator->mapmodel  .resize(3 * navigator->msize);
	navigator->mlabels   .resize(3 * navigator->msize);

	for (auto item = current.begin(); item != current.end(); ++item) {
		Symbol key(item->key);

		if (key.chr() == 'x') {  // pose
			marshalpose(&navigator->trajectory[0], static_cast<Pose3&>(item->value), key.index());
		}
		else if (key.chr() == 'l') {  // landmark
			marshalpoint(&navigator->mapmodel[0], static_cast<Point3&>(item->value), key.index());
		}
	}

	// clear for next round (note that graph + estimate
	// indicate only the __new__ landmarks and estimates)
	navigator->graph.resize(0);
	navigator->estimate.clear();

	++navigator->t;
}

// get a the trajectory estimate held in the navigator
double* gettrajectory(ISAM2Navigator* navigator, int* length)
{
	*length = navigator->tlength;
	return &navigator->trajectory[0];
}

// get a the map model held in the navigator
double* getmapmodel(ISAM2Navigator* navigator, int* length)
{
	*length = navigator->msize;
	return &navigator->mapmodel[0];
}
}