// PixelRangeFactor.cpp
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

#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <tuple>
#include <string>
#include <cmath>
#include "PixelRangeFactor.h"

using namespace std;
using namespace gtsam;

namespace monorfs {

// create default PixelRangeFactor instance
PixelRangeFactor::PixelRangeFactor() {}

// create PixelRangeFactor instance given appropiate keys for the
// relevant pose and landmark, the actual pixel-range measurement
// a noise model and a camera focal length parameter
PixelRangeFactor::PixelRangeFactor(Key poseKey, Key landmarkKey,
	                               const double px, const double py, const double range,
	                               const SharedNoiseModel& model, const double focal)
		: NoiseModelFactor2<Pose3, Point3>(model, poseKey, landmarkKey),
		  px(px), py(py), range(range), focal(focal) {}

// get a string representation of the PixelRangeFactor
string PixelRangeFactor::tostring(const KeyFormatter& keyformat) const
{
	return "PixelRangeFactor(" + keyformat(key1()) + "," + keyformat(key2()) + "): " +
	       "[" + to_string(px) + ", " + to_string(py) + ", " + to_string(range) + "]";
}

// compare with another Factor component-by-component
bool PixelRangeFactor::equals(const NonlinearFactor& thatfactor, double eps) const
{
	const PixelRangeFactor* that = dynamic_cast<const PixelRangeFactor*>(&thatfactor);
	return that != nullptr && Base::equals(*that, eps) &&
	       fabs(this->px    - that->px)    < eps &&
	       fabs(this->py    - that->py)    < eps &&
	       fabs(this->range - that->range) < eps;
}

// calculate h(p, m) - z, using the RGBD equation
// h(p, m) = [|m-p|, (f x/z)L, (f y/z)L]
// 
// where |a-b| is the euclidean distance between a and b and
// (.)L is performed on the local axis
Vector PixelRangeFactor::evaluateError(const Pose3& pose, const Point3& landmark,
	                                   boost::optional<Matrix&> H1, boost::optional<Matrix&> H2) const
{
	Matrix jlocalpose, jlocalpoint;
	Point3 local = pose.transform_to(landmark, jlocalpose, jlocalpoint);

	double erange = local.norm();
	double epx    = focal * local.x() / local.z();
	double epy    = focal * local.y() / local.z();

	// jacobians
	Matrix3 jprojection;
	
	if (H1 || H2) {
		// the jacobian of the homography projection
		double mag = sqrt(local.x() * local.x() + local.y() * local.y() + local.z() * local.z());
		jprojection << focal / local.z(), 0,                     -focal * local.x() / (local.z() * local.z()),
		               0,                     focal / local.z(), -focal * local.y() / (local.z() * local.z()),
		               local.x() / mag,       local.y() / mag,    local.z() / mag;
	}

	if (H1) {
		*H1 = jprojection * jlocalpose;
	}

	if (H2) {
		*H2 = jprojection * jlocalpoint;
	}

	return Vector3(epx - px, epy - py, erange - range);
}

// get the internal measurement data as the tuple (px, py, range)
const tuple<double, double, double> PixelRangeFactor::measured() const
{
	return make_tuple(px, py, range);
}
}
