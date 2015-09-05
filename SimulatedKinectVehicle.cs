// SimulatedVehicle.cs
// Vehicle motion and measurement model using a simulated environment
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

using System;
using System.Collections.Generic;

using AForge;
using Accord.Math;
using Accord.Statistics.Distributions.Univariate;

using Microsoft.Xna.Framework;

using U  = monorfs.Util;
using ME = monorfs.MatrixExtensions;

namespace monorfs
{
/// <summary>
/// Simulation Kinect vehicle model.
/// Similar to SimulatedVehicle, but adds
/// a better visibility model for Kinect.
/// </summary>
public class SimulatedKinectVehicle : SimulatedVehicle
{
	/// <summary>
	/// Horizontal resolution of the real vehicle.
	/// </summary>
	private float resx;

	/// <summary>
	/// Vertical resolution of the real vehicle.
	/// </summary>
	private float resy;

	/// <summary>
	/// Kinect real vehicle depth map accesor.
	/// </summary>
	private Func<float[][]> getdepth;

	/// <summary>
	/// Copy constructor. Perform a deep copy of another simulated vehicle.
	/// </summary>
	/// <param name="that">Copied simulated vehicle.</param>
	/// <param name="copytrajectory">If true, the vehicle historic trajectory is copied. Relatively heavy operation.</param>
	public SimulatedKinectVehicle(SimulatedKinectVehicle that, bool copytrajectory = false)
		: base(that, copytrajectory)
	{
		this.resx     = that.resx;
		this.resy     = that.resy;
		this.getdepth = that.getdepth;
	}

	/// <summary>
	/// Perform a deep copy of another general vehicle,
	/// scaling the covariance matrices and setting a new detection probability.
	/// </summary>
	/// <param name="that">Copied vehicle.</param>
	/// <param name="motioncovmultiplier">Scalar multiplier for the motion covariance matrix.</param>
	/// <param name="measurecovmultiplier">Scalar multiplier for the measurement covariance matrix.</param>
	/// <param name="pdetection">Probability of detection.</param>
	/// <param name="clutter">Clutter density.</param>
	/// <param name="copytrajectory">If true, the vehicle historic trajectory is copied. Relatively heavy operation.</param>
	public SimulatedKinectVehicle(KinectVehicle that, double motioncovmultiplier, double measurecovmultiplier, double pdetection, double clutter, bool copytrajectory = false)
		: base(that, motioncovmultiplier, measurecovmultiplier, pdetection, clutter, copytrajectory)
	{
		this.resx = that.ResX;
		this.resy = that.ResY;

		this.getdepth = () => that.DepthFrame;
	}

	/// <summary>
	/// Find if a given ladmark is visible from the current pose of the vehicle
	/// using pixel-range coordinates to express the landmark.
	/// </summary>
	/// <param name="measurement">Queried landmark in pixel-range coordinates.</param>
	/// <returns>True if the landmark is visible; false otherwise.</returns>
	public override bool VisibleM(double[] measurement)
	{
		int   x     = (int) (measurement[0] + resx / 2);
		int   y     = (int) (measurement[1] + resy / 2);
		float range = (float) measurement[2];

		if (!base.VisibleM(measurement)) {
			return false;
		}

		float[][] depth = getdepth();

		// the point is inbounds (redundant with base implementation, but
		// floating point errors could make it necessary
		if (y < 0 || y >= depth.Length || x < 0 || x >= depth[0].Length) {
			return false;
		}

		// the point is in front of the depth map (i.e. not occluded)
		return range <= depth[x][y];
	}
}
}
