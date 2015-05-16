// PixelRangeFactor.h
// RGB-D measurement factor, pixel coordinates + range
// Part of MonoRFS
//
// Copyright (C) 2015 Angelo Falchetti
// All rights reserved.
// Any use, reproduction, distribution or copy of this
// work via any medium is strictly forbidden without
// express written consent from the author.

#ifndef PIXELRANGEFACTOR_H
#define PIXELRANGEFACTOR_H

#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <tuple>
#include <string>
#include <cmath>

namespace monorfs {
using namespace std;
using namespace gtsam;

class PixelRangeFactor : public NoiseModelFactor2<Pose3, Point3>
{
private:
	double px;
	double py;
	double range;
	double focal;
	
public:
	// create default PixelRangeFactor instance
	PixelRangeFactor();
	
	// create PixelRangeFactor instance given appropiate keys for the
	// relevant pose and landmark, the actual pixel-range measurement
	// a noise model and a camera focal length parameter
	PixelRangeFactor(Key poseKey, Key pointKey,
	                 const double px, const double py, const double range,
	                 const SharedNoiseModel& model, const double focal);
	
	// get a string representation of the PixelRangeFactor
	string tostring(const KeyFormatter& keyformat = DefaultKeyFormatter) const;
	
	// compare with another Factor component-by-component
	virtual bool equals(const NonlinearFactor& thatfactor, double eps) const;
	
	// calculate h(m) - z, using the RGBD equation
	// h(m) = [|m-x|, (f x/z)L, (f y/z)L]
	// 
	// where |a-b| is the euclidean distance between a and b and
	// (.)L is performed on the local axis
	Vector evaluateError(const Pose3& pose, const Point3& point,
	                            boost::optional<Matrix&> H1, boost::optional<Matrix&> H2) const;
	
	// get the internal measurement data as the tuple (px, py, range)
	const tuple<double, double, double> measured() const;
};
}

#endif