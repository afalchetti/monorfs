﻿// SimulatedVehicle.cs
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
/// Simulation vehicle model.
/// It uses a 3d odometry motion model (yaw-pitch-roll) and
/// a pixel-range measurement model.
/// </summary>
public class SimulatedVehicle : Vehicle
{
	/// <summary>
	/// Internal probability of detection.
	/// </summary>
	private readonly double detectionProbability = Config.DetectionProbability;

	/// <summary>
	/// Amount of expected clutter (spuriousness) on the measurement process.
	/// </summary>
	public readonly double ClutterDensity = Config.ClutterDensity;

	/// <summary>
	/// Probability of detection.
	/// </summary>
	public double PD { get { return detectionProbability; } }

	/// <summary>
	/// Clutter density integral over the whole measurement space, i.e.
	/// the expected number of clutter measurements.
	/// </summary>
	public readonly double ClutterCount;

	/// <summary>
	/// Defines if this vehicle remains perfectly still when no input commands are given.
	/// If the robot is well located on the ground this makes sense, but if it is a handheld
	/// device, it is not so easy to maintain it steady.
	/// </summary>
	public static bool PerfectStill { get { return Config.PerfectStill; } }

	/// <summary>
	/// Poisson distributed random generator with parameter lambda = ClutterCount.
	/// </summary>
	/// <remarks>The type is general distribution to allow for zero clutter (dirac distribution)</remarks>
	private UnivariateDiscreteDistribution clutterGen;

	/// <summary>
	/// Construct a new Vehicle from a "zero initial state", i.e. location = {0, 0, 0},
	/// theta = 0 and rotation axis = {1, 0, 0} with no landmarks.
	/// </summary>
	public SimulatedVehicle()
		: this(new double[3] {0, 0, 0}, 0, new double[3] {1, 0, 0}, new List<double[]>()) {}

	/// <summary>
	/// Construct a new Vehicle object from its initial state.
	/// </summary>
	/// <param name="location">Spatial coordinates.</param>
	/// <param name="theta">Orientation angle.</param>
	/// <param name="axis">Orientation rotation axis.</param>
	/// <param name="landmarks">Landmark 3d locations against which the measurements are performed.</param>
	public SimulatedVehicle(double[] location, double theta, double[] axis, List<double[]> landmarks)
		: this(location, theta, axis, landmarks, 575.8156,
		       new Rectangle(-640 / 2, -480 / 2, 640, 480),
		       new Range(0.1f, 10f)) {}

	/// <summary>
	/// Construct a new Vehicle object from its initial state.
	/// </summary>
	/// <param name="location">Spatial coordinates.</param>
	/// <param name="theta">Orientation angle.</param>
	/// <param name="axis">Orientation rotation axis.</param>
	/// <param name="landmarks">Landmark 3d locations against which the measurements are performed.</param>
	/// <param name="focal">Focal lenghth.</param>
	/// <param name="film">Film area.</param>
	/// <param name="clip">Range clipping area.</param>
	public SimulatedVehicle(double[] location, double theta, double[] axis, List<double[]> landmarks,
	                        double focal, Rectangle film, Range clip)
		: base(location, theta, axis, focal, film, clip)
	{
		ClutterCount = ClutterDensity * FilmArea.Height * FilmArea.Width * RangeClip.Length;

		if (ClutterCount > 0) {
			clutterGen = new PoissonDistribution(ClutterCount);
		}
		else {
			clutterGen = new DegenerateDistribution(0);
		}

		Landmarks     = landmarks;
		SidebarWidth  = 1;
		SidebarHeight = 1;

		HasDataAssociation = true;
		DataAssociation    = new List<int>();
	}

	/// <summary>
	/// Copy constructor. Perform a deep copy of another simulated vehicle.
	/// </summary>
	/// <param name="that">Copied simulated vehicle.</param>
	/// <param name="copytrajectory">If true, the vehicle historic trajectory is copied. Relatively heavy operation.</param>
	public SimulatedVehicle(SimulatedVehicle that, bool copytrajectory = false)
		: this((Vehicle) that, copytrajectory)
	{
		this.detectionProbability = that.detectionProbability;
		this.ClutterDensity       = that.ClutterDensity;
		this.ClutterCount         = that.ClutterCount;
		this.clutterGen           = that.clutterGen;
	}

	/// <summary>
	/// Perform a deep copy of another general vehicle.
	/// </summary>
	/// <param name="that">Copied general vehicle.</param>
	/// <param name="copytrajectory">If true, the vehicle historic trajectory is copied. Relatively heavy operation.</param>
	public SimulatedVehicle(Vehicle that, bool copytrajectory = false)
		: this(that, 1, 1, Config.DetectionProbability, Config.ClutterDensity, copytrajectory) {}

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
	public SimulatedVehicle(Vehicle that, double motioncovmultiplier, double measurecovmultiplier, double pdetection, double clutter, bool copytrajectory = false)
		: base(that, copytrajectory)
	{
		this.detectionProbability = pdetection;
		this.ClutterDensity       = clutter;
		this.ClutterCount         = this.ClutterDensity * this.FilmArea.Height * this.FilmArea.Width * this.RangeClip.Length;

		this.MotionCovariance      = motioncovmultiplier.Multiply(that.MotionCovariance);
		this.MeasurementCovariance = measurecovmultiplier.Multiply(that.MeasurementCovariance);

		if (this.ClutterCount > 0) {
			this.clutterGen = new PoissonDistribution(this.ClutterCount);
		}
		else {
			this.clutterGen = new DegenerateDistribution(0);
		}
	}

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
	/// <param name="dx">Moved distance from odometry in the local vertical movement-perpendicular direction since last timestep.</param>
	/// <param name="dy">Moved distance from odometry in the local horizontal movement-perpendicular direction since last timestep.</param>
	/// <param name="dz">Moved distance from odometry in the local depth movement-parallel direction since last timestep.</param>
	/// <param name="dyaw">Angle variation from odometry in the yaw coordinate since last timestep.</param>
	/// <param name="dpitch">Angle variation from odometry in the pitch coordinate since last timestep.</param>
	/// <param name="droll">Angle variation from odometry in the roll coordinate since last timestep.</param>
	public override void Update(GameTime time, double dx, double dy, double dz, double dyaw, double dpitch, double droll)
	{
		// no input, static friction makes the robot stay put (if there is any static friction)
		if (PerfectStill && dx == 0 && dy == 0 && dz == 0 && dyaw == 0 && dpitch == 0 && droll == 0) {
			return;
		}

		base.Update(time, dx, dy, dz, dyaw, dpitch, droll);

		State       = State.Add(time.ElapsedGameTime.TotalSeconds.Multiply(
		                            U.RandomGaussianVector(new double[7] {0, 0, 0, 0, 0, 0, 0}, MotionCovarianceQ)));
		Orientation = Quaternion.Normalize(Orientation);

		WayPoints[WayPoints.Count - 1] = Tuple.Create(time.TotalGameTime.TotalSeconds, Util.SClone(State));
	}
	
	/// <summary>
	/// Obtain a measurement from the hidden state.
	/// It does not use any randomness or misdetection, 
	/// as it is designed to be used with UKF filters.
	/// </summary>
	/// <param name="landmark">Landmark 3d location against which the measurement is performed.</param>
	/// <returns>Pixel-range measurements.</returns>
	public double[] MeasurePerfect(double[] landmark)
	{
		double[]   diff  = landmark.Subtract(Location);
		Quaternion local = Quaternion.Conjugate(Orientation) *
			                    new Quaternion((float) diff[0], (float) diff[1], (float) diff[2], 0) * Orientation;

		double range  = Math.Sign(local.Z) * diff.Euclidean();
		double px     = VisionFocal * local.X / local.Z;
		double py     = VisionFocal * local.Y / local.Z;

		return new double[3] {px, py, range};
	}

	/// <summary>
	/// Obtain a measurement from the hidden state. It follows a
	/// pixel-range model where the range and pixel index from a
	/// map landmark m are:
	/// 
	/// [r, px, py] = [|m-x|, (f x/z)L, (f y/z)L] + N(0, R)
	/// 
	/// where |a-b| is the euclidean distance between a and b and
	/// (.)L is performed on the local axis
	/// This method always detects the landmark (misdetection
	/// probability is ignored).
	/// </summary>
	/// <param name="landmark">Landmark 3d location against which the measurement is performed.</param>
	/// <returns>Pixel-range measurements.</returns>
	public double[] MeasureDetected(double[] landmark)
	{
		return MeasurePerfect(landmark).Add(U.RandomGaussianVector(new double[3] {0, 0, 0}, MeasurementCovariance));
	}

	/// <summary>
	/// Obtain several measurements from the hidden state.
	/// This method always detects the landmark (misdetection
	/// probability is ignored).
	/// </summary>
	/// <returns>Pixel-range measurements.</returns>
	public double[][] MeasureDetected()
	{
		double[][] measurements = new double[Landmarks.Count][];

		for (int i = 0; i < Landmarks.Count; i++) {
			measurements[i] = MeasureDetected(Landmarks[i]);
		}

		return measurements;
	}

	/// <summary>
	/// Obtain several measurements from the hidden state.
	/// Ladmarks may be misdetected.
	/// </summary>
	/// <returns>Pixel-range measurements.</returns>
	public override List<double[]> Measure()
	{
		List<double[]> measurements = new List<double[]>();
		DataAssociation = new List<int>();

		// add every measurement with probability = DetectionProbbility
		for (int i = 0; i < Landmarks.Count; i++) {
			if (Visible(Landmarks[i])) {
				if (U.Uniform.Next() < detectionProbability) {
					measurements   .Add(MeasureDetected(Landmarks[i]));
					DataAssociation.Add(i);
				}
			}
		}

		// poisson distributed clutter measurement count
		// a cap of 10 lambda is enforced because technically
		// the number is unbounded so it could freeze the system
		int nclutter;
		try {
			// the poisson generator may underflow if lambda is too small
			nclutter = Math.Min(clutterGen.Generate(), (int)(ClutterCount * 10));
		}
		catch (ArithmeticException)
		{
			nclutter = 0;
		}

		for (int i = 0; i < nclutter; i++) {
			double px    = U.Uniform.Next() * FilmArea.Width + FilmArea.Left;
			double py    = U.Uniform.Next() * FilmArea.Height + FilmArea.Top;
			double range = U.Uniform.Next() * RangeClip.Length + RangeClip.Min;
			measurements.Add(new double[3] {px, py, range});
			DataAssociation.Add(int.MinValue);
		}

		MappedMeasurements.Clear();
		foreach (double[] z in measurements) {
			MappedMeasurements.Add(MeasureToMap(z));
		}

		return measurements;
	}

	/// <summary>
	/// Obtain the jacobian of the measurement model.
	/// Designed to be used with EKF filters.
	/// </summary>
	/// <param name="landmark">Landmark 3d location against which the measurement is performed.</param>
	/// <returns>Measurement model linearization jacobian.</returns>
	public double[][] MeasurementJacobian(double[] landmark)
	{
		double[]   diff  = landmark.Subtract(Location);
		Quaternion local = Quaternion.Conjugate(Orientation) *
			                    new Quaternion((float) diff[0], (float) diff[1], (float) diff[2], 0) * Orientation;

		// the jacobian of the homography projection part is given by
		// f/z I 0
		//  X/|X|
		double mag = Math.Sqrt(local.X * local.X + local.Y * local.Y + local.Z * local.Z);
		double[][] jprojection = {new double[] {VisionFocal / local.Z, 0,                     -VisionFocal * local.X / (local.Z * local.Z)},
		                          new double[] {0,                     VisionFocal / local.Z, -VisionFocal * local.Y / (local.Z * local.Z)},
		                          new double[] {local.X / mag,         local.Y / mag,         local.Z /mag}};

		// the jacobian of the change of coordinates part of the measurement process is the rotation matrix 
		double[][] jrotation = ME.MatrixFromQuaternion(Orientation);

		return jprojection.Multiply(jrotation);
	}

	/// <summary>
	/// Find if a given ladmark is visible from the current pose of the vehicle.
	/// </summary>
	/// <param name="landmark">Queried landmark.</param>
	/// <returns>True if the landmark is visible; false otherwise.</returns>
	public bool Visible(double[] landmark)
	{
		return VisibleM(MeasurePerfect(landmark));
	}

	/// <summary>
	/// Find if a given ladmark is visible from the current pose of the vehicle
	/// using pixel-range coordinates to express the landmark.
	/// </summary>
	/// <param name="measurement">Queried landmark in pixel-range coordinates.</param>
	/// <returns>True if the landmark is visible; false otherwise.</returns>
	public virtual bool VisibleM(double[] measurement)
	{
		return FilmArea.Left < measurement[0] && measurement[0] < FilmArea.Right &&
		       FilmArea.Top  < measurement[1] && measurement[1] < FilmArea.Bottom &&
		       RangeClip.Min < measurement[2] && measurement[2] < RangeClip.Max;
	}

	/// <summary>
	/// Get the probability of detection of a particular landmark.
	/// It is modelled as a constant if it's on the FOV and zero if not. 
	/// </summary>
	/// <param name="landmark">Queried landmark.</param>
	/// <returns></returns>
	public double DetectionProbability(double[] landmark)
	{
		return Visible(landmark) ? detectionProbability : 0;
	}

	/// <summary>
	/// Get the probability of detection of a particular landmark
	/// using pixel-range coordinates to express the landmark.
	/// It is modelled as a constant if it's on the FOV and zero if not.
	/// </summary>
	/// <param name="measurement">Queried landmark in pixel-range coordinates.</param>
	/// <returns></returns>
	public double DetectionProbabilityM(double[] measurement)
	{
		return VisibleM(measurement) ? detectionProbability : 0;
	}
}
}
