// TrackVehicle.cs
// Vehicle motion and measurement model used to track other vehicles
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

using Accord.Math;

using Microsoft.Xna.Framework;

namespace monorfs
{
/// <summary>
/// Simulation Kinect vehicle model.
/// Similar to SimulatedVehicle, but adds
/// a better visibility model for Kinect.
/// </summary>
public class TrackVehicle<MeasurerT, PoseT, MeasurementT> : SimulatedVehicle<MeasurerT, PoseT, MeasurementT>
	where PoseT        : IPose<PoseT>, new()
	where MeasurementT : IMeasurement<MeasurementT>, new()
	where MeasurerT    : IMeasurer<MeasurerT, PoseT, MeasurementT>, new()
{
	/// <summary>
	/// Construct a TrackVehicle at the origin with default properties.
	/// </summary>
	public TrackVehicle() : base() {}

	/// <summary>
	/// Copy constructor. Perform a deep copy of another tracker vehicle.
	/// </summary>
	/// <param name="that">Copied simulated vehicle.</param>
	/// <param name="copytrajectory">If true, the vehicle historic trajectory is copied. Relatively heavy operation.</param>
	public TrackVehicle(SimulatedVehicle<MeasurerT, PoseT, MeasurementT> that,
	                    bool copytrajectory = false)
		: base(that, copytrajectory) {}

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
	public TrackVehicle(Vehicle<MeasurerT, PoseT, MeasurementT> that,
	                    double motioncovmultiplier, double measurecovmultiplier,
	                    double pdetection, double clutter, bool copytrajectory = false)
		: base(that, motioncovmultiplier, measurecovmultiplier, pdetection, clutter, copytrajectory) {}


	/// <summary>
	/// Apply the motion model to the vehicle. It corresponds to a
	/// 3D odometry model following the equation:
	/// 
	/// x = x + q dx q* + N(0, Q)
	/// o = dq o dq* + N(0, Q')
	/// 
	/// where q is the midrotation quaternion (halfway between the old and new orientations) and N(a, b) is a normal function
	/// with mean 'a' and covariance matrix 'b'.
	/// </summary>
	/// <param name="time">Provides a snapshot of timing values.</param>
	/// <param name="reading">Odometry reading (dx, dy, dz, dpitch, dyaw, droll).</param>
	public void UpdateNoisy(GameTime time, double[] reading)
	{
		Update(time, reading);
		
		// no input, static friction makes the robot stay put (if there is any static friction)
		if (!(PerfectStill && reading.IsEqual(0))) {
			double[] noise = time.ElapsedGameTime.TotalSeconds.Multiply(
			                     Util.RandomGaussianVector(new double[OdoSize],
			                                            MotionCovariance));
			Pose = Pose.AddOdometry(noise);
		}

		WayPoints[WayPoints.Count - 1] = Tuple.Create(time.TotalGameTime.TotalSeconds, Util.SClone(Pose.State));
	}
}
}
