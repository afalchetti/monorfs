// PixelRangeFactor.h
// RGB-D measurement factor, pixel coordinates + range
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
